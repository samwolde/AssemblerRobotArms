
#ifndef PATH_PLANNER_H
#define PATH_PLANNER_H
#include <robot_planner/GridMap.h>
#include <functional>
#include <dirent.h>
#include <robot_planner/BuildMap.h>
#include <robot_lib/Sensor.h>
#include <robot_lib/Steering.h>
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
#include <sensor_msgs/Range.h>
#include <std_msgs/Float32.h>
#include <bits/stdc++.h>
#include <unordered_map>
#include <thread>

namespace wheely_planner
{
    class PathPlanner
    {
    public:
        PathPlanner(ros::NodeHandlePtr,GridMap* grid, double kh );
        ~PathPlanner(){};
        Cell A_S_PlanPath(Coordinate from,Coordinate to);
        std::stack<Cell*> constructPath(Cell * node);
        bool FollowPath(std::stack<Cell*>* path);
        void clicked_sub(geometry_msgs::PointConstPtr pt);
        void setXY(double _x, double _y){
            x =_x;
            y=_y;
        };
    private:
        visualization_msgs::Marker explored_pts,begin_end,path_found;
        ros::ServiceClient goto_cl;
        GridMap* gridMap;
        double x,y;
        ros::Subscriber cmd_go_sub;
        int kh;
    };    
} // namespace wheely_planner

#endif