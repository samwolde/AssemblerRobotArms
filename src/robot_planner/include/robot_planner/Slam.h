#ifndef SLAM_H
#define SLAM_H

#include <robot_planner/GridMap.h>
#include <robot_planner/MapBuilder.h>
#include <robot_planner/PathPlanner.h>
#include <functional>
#include <dirent.h>
#include <robot_planner/BuildMap.h>
#include <robot_lib/Sensor.h>
#include <robot_lib/GoTo.h>
#include <iostream>
#include <regex>
#include <ignition/math/Vector3.hh>
#include <robot_planner/ObjectPick.h>
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
#include <signal.h>
#include <std_msgs/Float32.h>
#include <thread>

namespace wheely_planner{
class Slam{
    public:
        Slam();
        Slam(MapFile map);
        void init();
        void A_S_PlanPath(Coordinate to);
        void constructPath(Cell* node);
        void gotoPt(geometry_msgs::PointConstPtr);
        //ROS subscriptions
        void odometryMsg(nav_msgs::OdometryConstPtr odom);
        void laserMsg(sensor_msgs::LaserScanConstPtr laser);
        void senseMsg(std_msgs::Float32ConstPtr state);
        void odomQueueCb();
        void senseQueueCb();
        void laserQueueCb();
        void clicked_sub(geometry_msgs::PointConstPtr);
    private:
        GridMap* gridMap;
        MapBuilder* mapBuilder;
        PathPlanner* pathPlanner;
        ObjectPick*  objectPicker;
        int sampleSize,kh,mapSize;
        double roll,  pitch, yaw,x,y,z,sensor_offset,robo_radius,cellSize/*Change this if robots height(along the Y-axis changes)*/;
        double sensor_angle;
        sensor_msgs::LaserScan l;
        ros::Publisher pub,marker_pub;
        visualization_msgs::Marker begin_end,explored_pts,path_found;
        ros::NodeHandlePtr rosNode;
        ros::ServiceClient sensorTurn, mvFrwdC, turnRC, turnLC, brakeC,goto_cl;   
        ros::CallbackQueue odom_queue,laser_queue,sense_queue;
        ros::Subscriber sub,laser_sub,sens_sub,cmd_go_sub,updateMapSub,mapSave;
        std::thread odomQueueThread,laserThread,sense_th;
        nav_msgs::Odometry odometry;
        geometry_msgs::Point pose;
};

}

#endif