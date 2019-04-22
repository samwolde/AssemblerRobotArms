#include <robot_gazebo/wheel_controller.h>

namespace gazebo
{
    bool WheelPlugin::HasStoped(){
            geometry_msgs::Twist current_vel = this->odometry.twist.twist;
            current_vel.linear.x=trunc(current_vel.linear.x*1000);
            current_vel.linear.y=trunc(current_vel.linear.y*1000);
            current_vel.linear.z=trunc(current_vel.linear.z*1000);
            current_vel.angular.z=trunc(current_vel.angular.z*1000);			
            bool stoped = 
            (current_vel.linear.x == 0 && current_vel.linear.y == 0 && current_vel.linear.z == 0 && current_vel.angular.z == 0);
            return stoped;
    }
    bool WheelPlugin::Overshoot(double init_yaw, double goal_yaw,bool right){
		if ( GetError(init_yaw, goal_yaw, right) < GetError(this->yaw, goal_yaw,right)- turnAccuracy * 1.2){
			return true;
		}
		return false;
	}
    /**
	 * rad should be +ve for left turn and -ve for right turn
	 */
	double WheelPlugin::GetGoalRad(double rad, double yaw, bool right){
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
		ros::SubscribeOptions turnSo = ros::SubscribeOptions::create<std_msgs::Float32>(
      robotNS + "/cmd_turnRight",
      10,
      boost::bind(&WheelPlugin::TurnRight, this, _1),
      ros::VoidPtr(), &this->rosQueue);

	  ros::SubscribeOptions turnLSo = ros::SubscribeOptions::create<std_msgs::Float32>(
      robotNS + "/cmd_turnLeft",
      10,
      boost::bind(&WheelPlugin::TurnLeft, this, _1),
      ros::VoidPtr(), &this->rosQueue);

	  ros::SubscribeOptions turnF = ros::SubscribeOptions::create<std_msgs::Float32>(
      robotNS + "/cmd_moveForward",
      10,
      boost::bind(&WheelPlugin::MoveForward, this, _1),
      ros::VoidPtr(), &this->rosQueue);

		ros::SubscribeOptions turnB = ros::SubscribeOptions::create<std_msgs::Float32>(
      robotNS + "/cmd_moveBack",
      10,
      boost::bind(&WheelPlugin::MoveBackward, this, _1),
      ros::VoidPtr(), &this->rosQueue);

	  ros::SubscribeOptions bso = ros::SubscribeOptions::create<std_msgs::Float32>(
      robotNS + "/cmd_brake",
      10,
      boost::bind(&WheelPlugin::Brake, this, _1),
      ros::VoidPtr(), &this->rosQueue);
		ros::SubscribeOptions odomSo = ros::SubscribeOptions::create<nav_msgs::Odometry>(
			this->subTopic,
			100,
			boost::bind(&WheelPlugin::odometryMsg, this, _1),
			ros::VoidPtr(), &this->rosOdomQueue
		);

		this->odomSub = this->rosNode->subscribe(odomSo);
		this->turnRightSub = this->rosNode->subscribe(turnSo);
		this->turnLeftSub = this->rosNode->subscribe(turnLSo);
		this->forwardSub = this->rosNode->subscribe(turnF);
		this->bacSub = this->rosNode->subscribe(turnB);
		this->brakeSub = this->rosNode->subscribe(bso);
		//Create A publisher
		this->rosPub = this->rosNode->advertise<geometry_msgs::Twist>(this->pubTopic, 1000);

	}
	/**
	 * Get the neccessary plugin parameters from sdf file.
	 */
	void WheelPlugin::GetParams(sdf::ElementPtr sdf_ptr){
		rosNode.reset(new ros::NodeHandle("gazebo_client"));
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
			kp = kp/10.0;
			ROS_INFO("Using KP %f.\n", this->kp);
		}
		if(sdf_ptr->HasElement("turnMargin")){
			turnMargin = atof(sdf_ptr->GetElement("turnMargin")->GetValue()->GetAsString().c_str());
			ROS_INFO("Using Turn Margin %f.\n", this->turnMargin);

		}
		if(sdf_ptr->HasElement("turnAccuracy")){
			turnAccuracy = atof(sdf_ptr->GetElement("turnAccuracy")->GetValue()->GetAsString().c_str());
			ROS_INFO("Using Turn Accuracy %f.\n", this->turnAccuracy);

		}
	}

	void WheelPlugin::QueueThread(){
		static const double timeout = 0.01;
		while (this->rosNode->ok())
		{
			this->rosQueue.callAvailable(ros::WallDuration(timeout));
		}
	};
	void WheelPlugin::QueeThreadOdom(){
		static const double timeout = 0.01;
		while (this->rosNode->ok())
		{
			this->rosOdomQueue.callAvailable(ros::WallDuration(timeout));
		}
	};
}