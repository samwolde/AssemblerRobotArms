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

using namespace std;

namespace gazebo
{
class WheelPlugin : public ModelPlugin
{
  public:
	void Load(physics::ModelPtr parent, sdf::ElementPtr sdf_ptr)
	{
		this->model = parent;
		ROS_INFO("\n\nPlugin Loaded Model Name : %s\n\n", this->model->GetName().c_str());

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
	void OnUpdate(){
	};
	/*Given some deegre set the angular velocity for some time
		 to achive a right turn of the degree*/
	// void Turn(const std_msgs::Float32ConstPtr msg);
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
		void turnTimeExpired(const ros::TimerEvent& e);
		bool HasStoped();
		void Turn(double angle,bool right);
		double GetGoalRad(double rad,bool right);
		double GetError(double prev_yaw, double goal_yaw, bool right);
	/*Given some deegre set the angular velocity for some time
			to achive a left turn of the degree*/
		std::unique_ptr<ros::NodeHandle> rosNode;
		ros::Subscriber turnRightSub, odomSub,turnLeftSub, forwardSub, bacSub, brakeSub;
		ros::Publisher rosPub;

		ros::CallbackQueue rosQueue;
		ros::CallbackQueue rosOdomQueue;

		std::thread callBackThread; 
		std::thread cbThreadOdom;
		bool stopTurning=false;
		ros::Timer timer;
		int update_num=0;
		nav_msgs::Odometry odometry;
		
		double roll,pitch, yaw,kp=0.1;
		double velocity=0.2,angularVel=3.14;
		physics::ModelPtr model;
		event::ConnectionPtr con;
		std::string robotNS="/",	pubTopic="/cmd_vel", subTopic="/odom";
};
	void WheelPlugin::MoveForward(const std_msgs::Float32ConstPtr vel){
		geometry_msgs::Twist v;
		v.linear.x = vel->data;
		v.linear.y = v.linear.z = v.angular.x = 0;
		this->rosPub.publish(v);
	};

	void WheelPlugin::MoveBackward(const std_msgs::Float32ConstPtr vel){
		geometry_msgs::Twist v;
		v.linear.x = -1 * vel->data;
		v.linear.y = v.linear.z = v.angular.x = 0;
		this->rosPub.publish(v);
	};
	/**
	 * rad should be +ve for left turn and -ve for right turn
	 */
	double WheelPlugin::GetGoalRad(double rad, bool right){
		double goal = this->yaw + rad;
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

	void WheelPlugin::TurnRight(const std_msgs::Float32ConstPtr msg){
		if ( msg->data > 180 || msg->data < 0){
			ROS_INFO("Error angle should be [0,180]");
		}
		this->Turn(-1 * msg->data, true);
	}
	void WheelPlugin::TurnLeft(const std_msgs::Float32ConstPtr msg){
		if ( msg->data > 180 || msg->data < 0){
			ROS_INFO("Error angle should be [0,180]");
		}
		this->Turn(1 * msg->data, false);
	}
	void WheelPlugin::Turn(double angle,bool right){
			geometry_msgs::Twist velocity;
			velocity.linear.x = velocity.linear.y = velocity.linear.z = 0;
			
			/*Yaw and angular velocity opposite signs*/
			velocity.angular.z = right ? this->angularVel : -1 *  this->angularVel;
			double rad_ang =  M_PI * angle/180;
			ROS_INFO("Publishing to topic %s\n\n", this->pubTopic.c_str());
			//save the current Orientation
			double  goal_yaw=this->GetGoalRad(rad_ang, right);
			double prev_yaw = this->yaw;
			double err=this->GetError(prev_yaw, goal_yaw, right);
			ROS_INFO("Goal Yaw is %f,this yaw is %f\n",goal_yaw, this->yaw);
			//Publish velocity continously then examine odometry to know when to stop
			while(true){
				err  = this->GetError(prev_yaw, goal_yaw, right);
				 //Implements a proportional controller
				if( this->GetError(prev_yaw, this->yaw,right) >= this->kp * err){	
					this->Brake();
					prev_yaw = this->yaw;
					err  =this->GetError(prev_yaw, goal_yaw, right);
					if( err <= 0.00025){
						ROS_INFO("Done! Goal Yaw is %f,this yaw is %f\n",goal_yaw, this->yaw);
						break;
					}
				}
				this->rosPub.publish(velocity);
			}
			ROS_INFO("Finished Turning an angle of %f\n\n",rad_ang);
	};
	//is Called back when odometry messages are available.
	void WheelPlugin::odometryMsg(nav_msgs::OdometryConstPtr odom){
		this->odometry = *odom;
		geometry_msgs::Quaternion q = odom->pose.pose.orientation;
		tf::Quaternion tf_qut(q.x, q.y, q.z, q.w); 
		tf::Matrix3x3(tf_qut).getRPY(this->roll, this->pitch, this->yaw);
		// ROS_INFO("(%f, %f, %f) \n", this->roll, this->pitch, this->yaw);
	};	
	void WheelPlugin::Brake(){
			geometry_msgs::Twist velocity;
			geometry_msgs::Twist current_vel = this->odometry.twist.twist;

			velocity.linear.x = velocity.linear.y = velocity.linear.z = velocity.angular.z = 0;
			while( !this->HasStoped()){
				this->rosPub.publish(velocity);
			}
	};
	bool WheelPlugin::HasStoped(){
				geometry_msgs::Twist current_vel = this->odometry.twist.twist;
				current_vel.linear.x=trunc(current_vel.linear.x*10000);
				current_vel.linear.y=trunc(current_vel.linear.y*10000);
				current_vel.linear.z=trunc(current_vel.linear.z*10000);
				current_vel.angular.z=trunc(current_vel.angular.z*10000);			
				bool stoped = 
				(current_vel.linear.x == 0 && current_vel.linear.y == 0 && current_vel.linear.z == 0 && current_vel.angular.z == 0);
				return stoped;
	}
	//Create the Subscribers and Publishers
	void WheelPlugin::InitNode(){
		ros::SubscribeOptions turnSo = ros::SubscribeOptions::create<std_msgs::Float32>(
      "/TurnRight",
      10,
      boost::bind(&WheelPlugin::TurnRight, this, _1),
      ros::VoidPtr(), &this->rosQueue);

	  ros::SubscribeOptions turnLSo = ros::SubscribeOptions::create<std_msgs::Float32>(
      "/TurnLeft",
      10,
      boost::bind(&WheelPlugin::TurnLeft, this, _1),
      ros::VoidPtr(), &this->rosQueue);

	  ros::SubscribeOptions turnF = ros::SubscribeOptions::create<std_msgs::Float32>(
      "/MoveForward",
      10,
      boost::bind(&WheelPlugin::MoveForward, this, _1),
      ros::VoidPtr(), &this->rosQueue);

		ros::SubscribeOptions turnB = ros::SubscribeOptions::create<std_msgs::Float32>(
      "/MoveBack",
      10,
      boost::bind(&WheelPlugin::MoveBackward, this, _1),
      ros::VoidPtr(), &this->rosQueue);

	  ros::SubscribeOptions bso = ros::SubscribeOptions::create<std_msgs::Float32>(
      "/Brake",
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
		std::string prefix = "";
		this->rosNode.reset(new ros::NodeHandle("gazebo_client"));
		if(sdf_ptr->HasElement("robotNamespace")){
			this->robotNS = "/"+ sdf_ptr->GetElement("robotNamespace")->GetValue()->GetAsString();
			ROS_INFO("Using Topic %s For robots name space\n", this->robotNS.c_str());
		}

		if( sdf_ptr->HasElement("odometrySubTopic") ){
			this->subTopic = this->robotNS +"/"+ sdf_ptr->GetElement("odometrySubTopic")->GetValue()->GetAsString() ;
			ROS_INFO("Using topic %s for odometry\n",this->subTopic.c_str()) ;
		}
		else{
			this->subTopic = this->robotNS + "/"+ this->subTopic;
		}
		if ( sdf_ptr->HasElement("velPubTopic")){
			this->pubTopic = this->robotNS + "/"+sdf_ptr->GetElement("velPubTopic")->GetValue()->GetAsString();
			ROS_INFO("Using Topic %s For Velocity Publishing \n", this->pubTopic.c_str());
		}
		else{
			this->pubTopic = this->robotNS + "/"+ this->pubTopic;
		}
		if(sdf_ptr->HasElement("angularVelocity") && sdf_ptr->HasElement("velocity")){
			this->angularVel = atof(sdf_ptr->GetElement("angularVelocity")->GetValue()->GetAsString().c_str());
			this->velocity = atof(sdf_ptr->GetElement("velocity")->GetValue()->GetAsString().c_str());
			ROS_INFO("Using Velocity %f, Angular Velocity %f\n",this->velocity, this->angularVel);
		}
		if(sdf_ptr->HasElement("kp")){
			this->kp = atof(sdf_ptr->GetElement("kp")->GetValue()->GetAsString().c_str());
			ROS_INFO("Using KP %f.\n", this->kp);
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
GZ_REGISTER_MODEL_PLUGIN(WheelPlugin)
}