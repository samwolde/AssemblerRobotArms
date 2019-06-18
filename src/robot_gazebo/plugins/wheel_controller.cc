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
	bool WheelPlugin::MoveForward(robot_lib::Steering::Request& req, robot_lib::Steering::Response& res){
		geometry_msgs::Twist v;
		v.linear.x =  req.val;
		v.linear.y = v.linear.z = v.angular.x = 0;
		rosPub.publish(v);
		return res.suc = true;
	};

	bool WheelPlugin::MoveBackward(robot_lib::Steering::Request& req, robot_lib::Steering::Response& res){
		geometry_msgs::Twist v;
		v.linear.x = -1 * req.val;
		v.linear.y = v.linear.z = v.angular.x = 0;
		rosPub.publish(v);
		return res.suc = true;
	};

	bool WheelPlugin::TurnRight(robot_lib::Steering::Request& req, robot_lib::Steering::Response& res){

		assert(req.val >= 0 && req.val <= 180);
		Turn(-1 * req.val, true);
		return res.suc = true;
	}
	bool WheelPlugin::TurnLeft(robot_lib::Steering::Request& req, robot_lib::Steering::Response& res){
		assert(req.val >= 0 && req.val <= 180);
		Turn(req.val, false);

		return res.suc = true;
	}
	void WheelPlugin::Turn(double angle,bool right){
			geometry_msgs::Twist velocity = this->odometry.twist.twist;			
			/*Yaw and angular velocity opposite signs*/
			velocity.angular.z = right ? this->angularVel : -1 *  this->angularVel;
			double rad_ang =  M_PI * angle/180;
			//save the current Orientation
			double  goal_yaw=GetGoalRad(rad_ang, this->yaw,right);
			double init_yaw = this->yaw, prev_yaw = this->yaw;
			double err=GetError(prev_yaw, goal_yaw, right);
			ros::Rate r(300);
			double integral=0,Iout=0,derivative,prev_err=err;
			//Publish velocity continously then examine odometry to know when to stop
			while(true){
				err  =GetError(this->yaw, goal_yaw, right);
				integral += dt * err;
				Iout =integral * ki;
				derivative = dt == 0? 0: (err - prev_err)/dt;
				derivative *= kd;
				//Preserve the sign, make magnitude 1
				velocity.angular.z /= fabsf64(velocity.angular.z);
				velocity.angular.z *= this->kp * err+ Iout + derivative;
				if( err <= turnAccuracy){
					Brake();
					velocity.angular.z = 0;
					return;
				}
				if ( this->Overshoot(init_yaw, goal_yaw,right)){
					Brake();
					init_yaw = this->yaw;
					right = !right;
					err  =GetError(this->yaw, goal_yaw, right);
					integral = 0;
					Iout =0;
					derivative = 0;
					//Preserve the sign, make magnitude 1
					velocity.angular.z /= fabsf64(velocity.angular.z);
					//Reversse back to the goal yaw
					velocity.angular.z *= -1 *(this->kp * err + Iout + derivative);
				}
				rosPub.publish(velocity);
				r.sleep();
				prev_err = err;
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
			static auto stopedTurning = [](geometry_msgs::Twist current_vel){
				current_vel.angular.z=trunc(current_vel.angular.z*100);			
				return ((int)current_vel.angular.z == 0);
			};
			ros::Rate r(500);
			geometry_msgs::Twist velocity;
			geometry_msgs::Twist current_vel = this->odometry.twist.twist;
			velocity = current_vel;
			velocity.angular.z = 0;
			while( !stopedTurning(current_vel)){
				this->rosPub.publish(velocity);
				current_vel = this->odometry.twist.twist;
				r.sleep();	
			}
	};
GZ_REGISTER_MODEL_PLUGIN(WheelPlugin)
}