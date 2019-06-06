#ifndef SLAM_H
#define SLAM_H

#include <robot_planner/GridMap.h>
#include <functional>
#include <robot_lib/MinTour.h>
#include <robot_lib/GoTo.h>
#include <robot_lib/Steering.h>
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

namespace wheely_planner{
class Slam{
    public:
        Slam();
        /*Take N samples turning a 360 degrees*/
        bool UpdateMap();
    private:
        GridMap* gridMap;
};
}
#endif