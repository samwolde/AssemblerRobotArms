#ifndef MAP_BUILDER_H
#define MAP_BUILDER_H
#include <robot_planner/GridMap.h>
#include <functional>
#include <dirent.h>
#include <robot_planner/GridMap.h>
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
#include <thread>

namespace wheely_planner{
    class MapBuilder{
    public:
        MapBuilder(ros::NodeHandlePtr rosNode, GridMap* g,double sensor_of, double sample);
        /*Take N samples turning a 360 degrees*/
        void UpdateMap(geometry_msgs::PointConstPtr p);
        //For coordinate transformations.
        void Tf_From_Sensor_Robo(Coordinate_t c, double angle);
        void Tf_From_Robo_World(Coordinate_t c);
        void SetXYZ_Yaw(double,double,double,double);
        void SetRange(double _r){range = _r;};
    private:        
        int sampleSize;
        ros::Publisher marker_pub;
        ros::ServiceClient sensorTurn;
        ros::Subscriber  updateMapSub,mapSave;
        GridMap* gridMap;
        double yaw,x,y,z,sensor_offset;
        double sensor_angle,range;
    };
}
#endif
