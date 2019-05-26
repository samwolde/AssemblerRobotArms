#include <robot_gazebo/wheel_controller.h>

namespace gazebo
{
    bool WheelPlugin::HasStoped(){
            geometry_msgs::Twist current_vel = this->odometry.twist.twist;
            current_vel.linear.x=trunc(current_vel.linear.x*100);
            current_vel.linear.y=trunc(current_vel.linear.y*100);
            current_vel.linear.z=trunc(current_vel.linear.z*100);
            current_vel.angular.z=trunc(current_vel.angular.z*100);			
            bool stoped = 
            (current_vel.linear.x == 0 && current_vel.linear.y == 0 && current_vel.linear.z == 0 && current_vel.angular.z == 0);
            return stoped;
    }
	/*The turning angle overshoots if the distance between the starting yaw and the goal yaw is 
	  less than the error between this->yaw and goal yaw. 
	  Based on the assumption that this yaw should be converging to goal yaw, in the direction defined
	  by right.
	*/
    bool WheelPlugin::Overshoot(double init_yaw, double goal_yaw,bool right){
		if ( GetError(init_yaw, goal_yaw, right) < GetError(this->yaw,goal_yaw,right) - turnAccuracy * 2){
			ROS_INFO("Overshoot init_yaw %f, this->yaw %f", init_yaw, this->yaw);
			ROS_INFO("Overshoot init_yaw_err %f, this->yaw_err %f",GetError(init_yaw, goal_yaw, right),
			GetError(this->yaw, goal_yaw,right)- turnAccuracy * 1.5);
			return true;
		}
		return false;
	}
    /**
	 * rad should be +ve for left turn and -ve for right turn
	 */
	double WheelPlugin::GetGoalRad(double rad, double yaw, bool right){
		assert(right ? rad <= 0 : rad >= 0);
		assert(rad <= M_PI && rad >= -M_PI);
		double goal = yaw + rad;
		if ( right && goal < - M_PI){
			//Normalize goal, -pi <= goal <= pi
			goal += 2  * M_PI;
		}
		if ( !right && goal > M_PI){
			//Normalize,  -pi <= goal <= pi
			goal -= 2* M_PI;
		}
		return goal;
	};

	double WheelPlugin::GetError(double prev_yaw, double goal_yaw, bool right){
		//Distance
		double err = std::fabs(goal_yaw - prev_yaw);
		if( right && prev_yaw < goal_yaw ){
			err = 2 * M_PI -err;
		}
		if ( !right && goal_yaw < prev_yaw){
			err  = 2 * M_PI -err;
		}
		return err;
	};
    //Create the Subscribers and Publishers
	void WheelPlugin::InitNode(){
		turnLServ = this->rosNode->advertiseService("/wheely/steering/cmd_turnLeft",&WheelPlugin::TurnLeft, this);
		turnRSer = this->rosNode->advertiseService("/wheely/steering/cmd_turnRight",&WheelPlugin::TurnRight, this);
		brakeServ = this->rosNode->advertiseService("/wheely/steering/cmd_brake",&WheelPlugin::Brake, this);
		forwardService = this->rosNode->advertiseService("/wheely/steering/cmd_moveForward", &WheelPlugin::MoveForward, this); 
		backService = this->rosNode->advertiseService("/wheely/nav/moveBackward_srv", &WheelPlugin::MoveBackward, this); 

		ros::SubscribeOptions odomSo = ros::SubscribeOptions::create<nav_msgs::Odometry>(
			this->subTopic,
			100,
			boost::bind(&WheelPlugin::odometryMsg, this, _1),
			ros::VoidPtr(), &this->rosOdomQueue
		);

		this->odomSub = this->rosNode->subscribe(odomSo);
		//Create A publisher
		this->rosPub = this->rosNode->advertise<geometry_msgs::Twist>(this->pubTopic, 1000);
	}
	/**
	 * Get the neccessary plugin parameters from sdf file.
	 */
	void WheelPlugin::GetParams(sdf::ElementPtr sdf_ptr){
		if(sdf_ptr->HasElement("prefix")){
			robotNS = "/"+ sdf_ptr->GetElement("prefix")->GetValue()->GetAsString();
			ROS_INFO("Using Topic %s For robots name space\n", this->robotNS.c_str());
		}

		if( sdf_ptr->HasElement("odometrySubTopic") ){
			subTopic = this->robotNS +"/"+ sdf_ptr->GetElement("odometrySubTopic")->GetValue()->GetAsString() ;
			ROS_INFO("Using topic %s for odometry\n",this->subTopic.c_str()) ;
		}
		else{
			subTopic = this->robotNS + "/"+ this->subTopic;
		}
		if ( sdf_ptr->HasElement("velPubTopic")){
			pubTopic = this->robotNS + "/"+sdf_ptr->GetElement("velPubTopic")->GetValue()->GetAsString();
			ROS_INFO("Using Topic %s For Velocity Publishing \n", this->pubTopic.c_str());
		}
		else{
			pubTopic = this->robotNS + "/"+ this->pubTopic;
		}
		if(sdf_ptr->HasElement("angularVelocity") && sdf_ptr->HasElement("velocity")){
			angularVel = atof(sdf_ptr->GetElement("angularVelocity")->GetValue()->GetAsString().c_str());
			velocity = atof(sdf_ptr->GetElement("velocity")->GetValue()->GetAsString().c_str());
			ROS_INFO("Using Velocity %f, Angular Velocity %f\n",this->velocity, this->angularVel);
		}
		if(sdf_ptr->HasElement("kp")){
			kp = atof(sdf_ptr->GetElement("kp")->GetValue()->GetAsString().c_str());
			ROS_INFO("Using KP %f.\n", this->kp);
		}
		if(sdf_ptr->HasElement("ki")){
			ki = atof(sdf_ptr->GetElement("ki")->GetValue()->GetAsString().c_str());
			ROS_INFO("Using ki %f.\n", this->ki);
		}
		if(sdf_ptr->HasElement("kd")){
			kd = atof(sdf_ptr->GetElement("kd")->GetValue()->GetAsString().c_str());
			ROS_INFO("Using KD %f.\n", this->kd);
		}
		if(sdf_ptr->HasElement("dt")){
			dt = atof(sdf_ptr->GetElement("dt")->GetValue()->GetAsString().c_str());
			ROS_INFO("Using dt %f.\n", this->dt);
		}
		if(sdf_ptr->HasElement("turnAccuracy")){
			turnAccuracy = atof(sdf_ptr->GetElement("turnAccuracy")->GetValue()->GetAsString().c_str());
			ROS_INFO("Using Turn Accuracy %f.\n", this->turnAccuracy);

		}
	}
	void WheelPlugin::QueeThreadOdom(){
		static const double timeout = 0.01;
		while (this->rosNode->ok())
		{
			this->rosOdomQueue.callAvailable(ros::WallDuration(timeout));
		}
	};
}