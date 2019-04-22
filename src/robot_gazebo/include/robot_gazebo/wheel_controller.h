#include <functional>
#include <gazebo/gazebo.hh>
#include <gazebo/physics/physics.hh>
#include <gazebo/common/common.hh>
#include <ignition/math/Vector3.hh>
#include "stdio.h"
#include "ros/ros.h"
#include "ros/callback_queue.h"
#include "ros/subscribe_options.h"
#include <std_msgs/Float32.h>
#include <geometry_msgs/Twist.h>
#include <nav_msgs/Odometry.h>
#include <tf/LinearMath/Matrix3x3.h>
#include <tf/LinearMath/Quaternion.h>

namespace gazebo
{
class WheelPlugin : public ModelPlugin
{
  public:
		void Load(physics::ModelPtr parent, sdf::ElementPtr sdf_ptr);
		void OnUpdate(){
		};
		/*Go Forward with the given velocity*/
		void MoveForward(const std_msgs::Float32ConstPtr vel);
		/*Go back wards with the given velocity*/
		void MoveBackward(const std_msgs::Float32ConstPtr vel);
		/*Given some deegre set the angular velocity for some time
			to achive a right turn of the degree*/
		void TurnRight(const std_msgs::Float32ConstPtr msg);
		/*Given some deegre set the angular velocity for some time
			to achive a left turn of the degree*/
		void TurnLeft(const std_msgs::Float32ConstPtr msg);
		void Brake(const std_msgs::Float32ConstPtr msg){this->Brake();};
		void Brake();
		void odometryMsg(nav_msgs::OdometryConstPtr odom);
  
  private:		
		void InitNode();
		void GetParams(sdf::ElementPtr sdf_ptr);
		void QueueThread();
		void QueeThreadOdom();

		bool HasStoped();
		void Turn(double angle,bool right);
		double GetGoalRad(double rad,double yaw,bool right);
		double GetError(double prev_yaw, double goal_yaw, bool right);
		bool Overshoot(double init_yaw, double goal_yaw,bool right);

		std::unique_ptr<ros::NodeHandle> rosNode;
		ros::Subscriber turnRightSub, odomSub,turnLeftSub, forwardSub, bacSub, brakeSub;
		ros::Publisher rosPub;

		ros::CallbackQueue rosQueue;
		ros::CallbackQueue rosOdomQueue;

		std::thread callBackThread; 
		std::thread cbThreadOdom;
		nav_msgs::Odometry odometry;
		
		double turnAccuracy=0.003, turnMargin=0.003;
		double roll,pitch, yaw,kp=0.2;
		double velocity=0.2,angularVel=3.14;
		physics::ModelPtr model;
		event::ConnectionPtr con;
		std::string robotNS="/",	pubTopic="/cmd_vel", subTopic="/odom";
    };
}