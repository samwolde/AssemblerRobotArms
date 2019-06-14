#include <functional>
#include <robot_lib/MinTour.h>
#include <robot_lib/GoTo.h>
#include <robot_lib/Steering.h>
#include <sensor_msgs/Range.h>
#include <iostream>
#include <signal.h>
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

#define DISTANCE_THRESHOLD 0.45
#define CLOSE_RANGE 0.23

class nav 
{
    public:
        nav(int argc, char ** argv);
        bool GetMinTour(robot_lib::MinTour::Request& req, robot_lib::MinTour::Response& res);
        double GetError(double prev_yaw, double goal_yaw, bool right);
        bool HasStoped();
        double GetGoalRad(double rad, double yaw, bool right);
        bool controlSpeed(Eigen::Vector3d dest,bool isBegin, bool isFinalDest);
        bool adjustOrientation(Eigen::Vector3d dest_vect);
        float getAngleDiff(Eigen::Vector3d dest_vect);
        void odometryMsg(nav_msgs::OdometryConstPtr odom);
        void shortSensorMsg(sensor_msgs::RangeConstPtr);
        bool goTo(robot_lib::GoTo::Request& req, robot_lib::GoTo::Response& res);
        void linkState(gazebo_msgs::LinkStatesConstPtr);
        void odomQueueCb();
        void linkQueueCb();
        void shrtSensorQueueCb();
        void OnUpdate(){};
        std::unique_ptr<ros::NodeHandle> rosNode;

    private:
        bool pointsEqual(geometry_msgs::Point* p1, geometry_msgs::Point* p2);
        ros::ServiceClient mvFrwdC, turnRC, turnLC, brakeC,mvBack;
        ros::CallbackQueue cmd_queue, odom_queue,linkQueue,shrtSensorqueue;
        ros::Subscriber sub1,sub2,link_state_sub,sensor_sub ;
        std::thread odomQueueThread, cmdQueueThread, linkThread,shrtSensorTh;
        nav_msgs::Odometry odometry;
        geometry_msgs::Point pose;
        double roll, pitch, yaw,x,y,z;
        Eigen::Vector3d robo_axis_init;
        gazebo_msgs::LinkStates linkStates;
        double distanceAccuracy=0.5,kp=0.08,range;
        
};
