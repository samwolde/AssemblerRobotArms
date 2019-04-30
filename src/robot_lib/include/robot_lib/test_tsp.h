#include <functional>
#include <robot_lib/MinTour.h>
#include <robot_lib/GoTo.h>
#include <iostream>
#include <regex>
#include <ignition/math/Vector3.hh>
#include "stdio.h"
#include "ros/ros.h"
#include "ros/callback_queue.h"
#include "ros/subscribe_options.h"
#include <std_msgs/Float32.h>
#include <gazebo_msgs/LinkStates.h>
#include <geometry_msgs/Twist.h>
#include <nav_msgs/Odometry.h>
#include <tf/LinearMath/Matrix3x3.h>
#include <tf/LinearMath/Quaternion.h>
#include <eigen3/Eigen/Dense>
#include <eigen3/Eigen/Geometry>
#include <eigen3/Eigen/Core>
#include <thread>

//An order array of Nodes.
typedef  geometry_msgs::Point* Tour_t;
class tsp 
{
    public:
        tsp(int argc, char ** argv);
        bool GetMinTour(robot_lib::MinTour::Request& req, robot_lib::MinTour::Response& res);
        double GetError(double prev_yaw, double goal_yaw, bool right);
        bool HasStoped();
        double GetGoalRad(double rad, double yaw, bool right);
        void controlSpeed(Eigen::Vector3d v);
        void adjustOrientation(Eigen::Vector3d dest_vect);
        void odometryMsg(nav_msgs::OdometryConstPtr odom);
        bool goTo(robot_lib::GoTo::Request& req, robot_lib::GoTo::Response& res);
        void linkState(gazebo_msgs::LinkStatesConstPtr);
        void cmdQueueCb();
        void odomQueueCb();
        void linkQueueCb();
        void OnUpdate(){};
        std::unique_ptr<ros::NodeHandle> rosNode;

    private:
        ros::Publisher mvFrwdPub,mvBackPub,turnRPub,turnLPub,brakePub;
        ros::CallbackQueue cmd_queue, odom_queue,linkQueue;
        ros::Subscriber sub1,sub2,link_state_sub ;
        std::thread odomQueueThread, cmdQueueThread, linkThread;
        nav_msgs::Odometry odometry;
        geometry_msgs::Point pose;
        double roll, pitch, yaw,x,y,z;
        Eigen::Vector3d robo_axis_init;
        gazebo_msgs::LinkStates linkStates;
        double turnAccuracy=0.001,distanceAccuracy=0.45,kp=0.25;
        
};
