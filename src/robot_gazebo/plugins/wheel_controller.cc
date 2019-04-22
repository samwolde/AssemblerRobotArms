#include <robot_gazebo/wheel_controller.h>

using namespace std;

namespace gazebo
{
	void WheelPlugin::Load(physics::ModelPtr parent, sdf::ElementPtr sdf_ptr)
	{
		this->model = parent;
		ROS_INFO("Wheel Plugin Loaded");

		if(!ros::isInitialized()){
			ROS_INFO("Ros not initialiazed");
			int argc = 0;
			char **argv = NULL;
			ros::init(argc, argv, "gazebo_client",ros::init_options::NoSigintHandler);
		}
		GetParams(sdf_ptr);
		InitNode();
		
		this->callBackThread = std::thread(std::bind(&WheelPlugin::QueueThread, this));
		this->cbThreadOdom = std::thread(std::bind(&WheelPlugin::QueeThreadOdom, this));
		this->con = event::Events::ConnectWorldUpdateBegin(std::bind(&WheelPlugin::OnUpdate, this));
	};

	void WheelPlugin::MoveForward(const std_msgs::Float32ConstPtr vel){
		geometry_msgs::Twist v;
		v.linear.x = vel->data;
		v.linear.y = v.linear.z = v.angular.x = 0;
		rosPub.publish(v);
	};

	void WheelPlugin::MoveBackward(const std_msgs::Float32ConstPtr vel){
		geometry_msgs::Twist v;
		v.linear.x = -1 * vel->data;
		v.linear.y = v.linear.z = v.angular.x = 0;
		rosPub.publish(v);
	};

	void WheelPlugin::TurnRight(const std_msgs::Float32ConstPtr msg){
		if ( msg->data > 180 || msg->data < 0){
			ROS_INFO("Error angle should be [0,180]");
			return;
		}
		Turn(-1 * msg->data, true);
	}
	void WheelPlugin::TurnLeft(const std_msgs::Float32ConstPtr msg){
		if ( msg->data > 180 || msg->data < 0){
			ROS_INFO("Error angle should be [0,180]");
		}
		Turn(1 * msg->data, false);
	}
	void WheelPlugin::Turn(double angle,bool right){
			geometry_msgs::Twist velocity;
			velocity.linear.x = velocity.linear.y = velocity.linear.z = 0;
			
			/*Yaw and angular velocity opposite signs*/
			velocity.angular.z = right ? this->angularVel : -1 *  this->angularVel;
			double rad_ang =  M_PI * angle/180;
			ROS_INFO("Publishing to topic %s", this->pubTopic.c_str());
			//save the current Orientation
			double  goal_yaw=GetGoalRad(rad_ang, this->yaw,right);
			double init_yaw = this->yaw, prev_yaw = this->yaw;
			double err=GetError(prev_yaw, goal_yaw, right);
			double temp_goal= GetGoalRad(right ? -1 * this->kp*err:this->kp*err , prev_yaw,right);
			double temp_err,kp= this->kp;
			ros::Rate r(30);
			ROS_INFO("Goal Yaw is %f,Temp Goal is %f, this yaw is %f",goal_yaw, temp_goal, this->yaw);
			//Publish velocity continously then examine odometry to know when to stop
			while(true){
				 //Implements a proportional controller
				temp_err = GetError(this->yaw,temp_goal, right);
				if( temp_err <= turnMargin){	
					Brake();
					r.sleep();		
					err  =GetError(this->yaw, goal_yaw, right);
					if( err <= turnAccuracy){
						ROS_INFO("Done! Goal Yaw is %f,this yaw is %f\n",goal_yaw, this->yaw);
						Brake();
						velocity.angular.z = 0;
						return;
					}
					temp_goal = GetGoalRad(right ? -1 * kp*err:kp*err, this->yaw,right);
				}
				if ( this->Overshoot(init_yaw, goal_yaw,right)){
					//Reversse back to the goal yaw
					Brake();
					init_yaw = this->yaw;
					velocity.angular.z *= -1;
					right = !right;
					err  =GetError(this->yaw, goal_yaw, right);
					//So that it eventually gets closer to the goal yaw.
					kp = kp > 0.05 ? kp * 0.9: 0.05;
					temp_goal = GetGoalRad(right ? -1 * kp*err:kp*err, this->yaw,right);
				}
				rosPub.publish(velocity);
			}
			ROS_INFO("Finished Turning an angle of %f\n\n",rad_ang);
			Brake();
	};
	//is Called back when odometry messages are available.
	void WheelPlugin::odometryMsg(nav_msgs::OdometryConstPtr odom){
		this->odometry = *odom;
		geometry_msgs::Quaternion q = odom->pose.pose.orientation;
		tf::Quaternion tf_qut(q.x, q.y, q.z, q.w); 
		tf::Matrix3x3(tf_qut).getRPY(this->roll, this->pitch, this->yaw);
	};	
	void WheelPlugin::Brake(){
			ros::Rate r(400);
			geometry_msgs::Twist velocity;
			geometry_msgs::Twist current_vel = this->odometry.twist.twist;

			velocity.linear.x = velocity.linear.y = velocity.linear.z = 
			velocity.angular.x = velocity.angular.y = velocity.angular.z = 0;
			while( !this->HasStoped()){
				this->rosPub.publish(velocity);
				r.sleep();	
			}
	};
GZ_REGISTER_MODEL_PLUGIN(WheelPlugin)
}