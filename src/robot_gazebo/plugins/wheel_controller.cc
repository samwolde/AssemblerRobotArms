#include <robot_gazebo/wheel_controller.h>

using namespace std;

namespace gazebo
{
	void WheelPlugin::Load(physics::ModelPtr parent, sdf::ElementPtr sdf_ptr)
	{
		this->model = parent;
		ROS_INFO("Wheel Plugin Loaded...");

		if(!ros::isInitialized()){
			ROS_INFO("Ros not initialiazed");
			int argc = 0;
			char **argv = NULL;
			ros::init(argc, argv, "gazebo_client",ros::init_options::NoSigintHandler);
		}
		rosNode.reset(new ros::NodeHandle("Wheel_Ctrlr"));
		this->cbThreadOdom = std::thread(std::bind(&WheelPlugin::QueeThreadOdom, this));
		
		GetParams(sdf_ptr);
		InitNode();
		this->con = event::Events::ConnectWorldUpdateBegin(std::bind(&WheelPlugin::OnUpdate, this));
	};

	bool WheelPlugin::Test(robot_lib::MinTour::Request& req, robot_lib::MinTour::Response& res){
		ROS_INFO("Worked");
	}
	bool WheelPlugin::MoveForward(robot_lib::Steering::Request& req, robot_lib::Steering::Response& res){
		geometry_msgs::Twist v;
		v.linear.x =  req.val;
		v.linear.y = v.linear.z = v.angular.x = 0;
		rosPub.publish(v);
		return true;
	};

	bool WheelPlugin::MoveBackward(robot_lib::Steering::Request& req, robot_lib::Steering::Response& res){
		geometry_msgs::Twist v;
		v.linear.x = -1 * req.val;
		v.linear.y = v.linear.z = v.angular.x = 0;
		rosPub.publish(v);
		return true;
	};

	bool WheelPlugin::TurnRight(robot_lib::Steering::Request& req, robot_lib::Steering::Response& res){

		assert(req.val > 0 && req.val < 180);
		Turn(-1 * req.val, true);
		return true;
	}
	bool WheelPlugin::TurnLeft(robot_lib::Steering::Request& req, robot_lib::Steering::Response& res){
		assert(req.val > 0 && req.val < 180);
		Turn(req.val, false);
		return true;
	}
	void WheelPlugin::Turn(double angle,bool right){
			Brake();
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
			ros::Rate r(300);
			ROS_INFO("Goal Yaw is %f,err is %f, this yaw is %f",goal_yaw, err, this->yaw);
			//Publish velocity continously then examine odometry to know when to stop
			while(true){
				err  =GetError(this->yaw, goal_yaw, right);
				//Preserve the sign, make magnitude 1
				velocity.angular.z /= fabsf64(velocity.angular.z);
				velocity.angular.z *= this->kp * err;
				if( err <= turnAccuracy){
					ROS_INFO("Done! Goal Yaw is %f,this yaw is %f\n",goal_yaw, this->yaw);
					Brake();
					velocity.angular.z = 0;
					return;
				}
				if ( this->Overshoot(init_yaw, goal_yaw,right)){
					Brake();
					init_yaw = this->yaw;
					right = !right;
					err  =GetError(this->yaw, goal_yaw, right);
					//Preserve the sign, make magnitude 1
					velocity.angular.z /= fabsf64(velocity.angular.z);
					//Reversse back to the goal yaw
					velocity.angular.z *= -1 *this->kp * err;
				}
				rosPub.publish(velocity);
				r.sleep();
			}
	};
	//is Called back when odometry messages are available.
	void WheelPlugin::odometryMsg(nav_msgs::OdometryConstPtr odom){
		this->odometry = *odom;
		geometry_msgs::Quaternion q = odom->pose.pose.orientation;
		tf::Quaternion tf_qut(q.x, q.y, q.z, q.w); 
		tf::Matrix3x3(tf_qut).getRPY(this->roll, this->pitch, this->yaw);
	};	
	void WheelPlugin::Brake(){
			ros::Rate r(500);
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