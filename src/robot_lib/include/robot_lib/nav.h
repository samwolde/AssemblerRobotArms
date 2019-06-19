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

#define CLOSE_RANGE 0.23
#define FRONT "short_ir_front_link"
#define REAR "short_ir_rear_link"
#define LEFT "short_ir_left_link"
#define RIGHT "short_ir_right_link"

#define FRONT_RANGE this->range[0]
#define RIGHT_RANGE this->range[1]
#define LEFT_RANGE this->range[2]
#define REAR_RANGE this->range[3]

#define SET_FRONT_RANGE(range,current) FRONT_RANGE = current;
#define SET_RIGHT_RANGE(range,current) RIGHT_RANGE = current;
#define SET_LEFT_RANGE(range,current)  LEFT_RANGE = current;
#define SET_REAR_RANGE(range,current)  REAR_RANGE = current;

#define CHECK_FRONT_RANGE(range,k) FRONT_RANGE <= k * CLOSE_RANGE
#define CHECK_RIGHT_RANGE(range,k) RIGHT_RANGE <= k * CLOSE_RANGE
#define CHECK_LEFT_RANGE(range,k)  LEFT_RANGE <= k * CLOSE_RANGE
#define CHECK_REAR_RANGE(range,k)  REAR_RANGE <= k * CLOSE_RANGE


//An order array of Nodes.
typedef  geometry_msgs::Point* Tour_t;
class nav 
{
    public:
        nav(int argc, char ** argv);
        bool GetMinTour(robot_lib::MinTour::Request& req, robot_lib::MinTour::Response& res);
        double GetError(double prev_yaw, double goal_yaw, bool right);
        bool HasStoped();
        double GetGoalRad(double rad, double yaw, bool right);
        bool controlSpeed(Eigen::Vector3d dest,bool isBegin, bool isFinalDest,bool detectObstacles);
        bool checkForObstacle();
        bool adjustOrientation(Eigen::Vector3d dest_vect);
        float getAngleDiff(Eigen::Vector3d dest_vect);
        void odometryMsg(nav_msgs::OdometryConstPtr odom);
        void shortSensorMsg(sensor_msgs::RangeConstPtr);
        bool goTo(robot_lib::GoTo::Request& req, robot_lib::GoTo::Response& res);
        void odomQueueCb();
        void shrtSensorQueueCb(int);
        void OnUpdate(){};
        std::unique_ptr<ros::NodeHandle> rosNode;

    private:
        bool pointsEqual(geometry_msgs::Point* p1, geometry_msgs::Point* p2);
        ros::ServiceClient mvFrwdC, turnRC, turnLC, brakeC,mvBack;
        ros::CallbackQueue cmd_queue, odom_queue,shrtSensorqueue[4];
        ros::Subscriber sub1,sub2,sensor_sub[4] ;
        std::thread odomQueueThread, cmdQueueThread,shrtSensorTh[4];
        nav_msgs::Odometry odometry;
        geometry_msgs::Point pose;
        double roll, pitch, yaw,x,y,z;
        Eigen::Vector3d robo_axis_init;
        double distanceAccuracy=0.45,kp=0.08,range[4]={0};
        
};
