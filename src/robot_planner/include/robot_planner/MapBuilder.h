#ifndef MAP_BUILDER_H
#define MAP_BUILDER_H
#include <robot_planner/GridMap.h>
#include <robot_planner/BuildMap.h>
#include <robot_planner/PathPlanner.h>
#include <functional>
#include <dirent.h>
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
#include <sensor_msgs/LaserScan.h>
#include <std_msgs/Float32.h>
#include <thread>

namespace wheely_planner{
    class MapBuilder{
    public:
        MapBuilder(ros::NodeHandlePtr rosNode, GridMap* g,PathPlanner* pathPlanner,double sensor_of, double sample);
        void BuildMap(geometry_msgs::PointConstPtr unreachablePt);
        /*Take N samples turning a 360 degrees*/
        void UpdateMap();
        bool CheckBlocked(Coordinate_t);
        //For coordinate transformations.
        void Tf_From_Sensor_Robo(Coordinate_t c, double angle);
        void Tf_From_Robo_World(Coordinate_t c);
        void SetXYZ_Yaw(double,double,double,double);
        void SetRange(double _r){range = _r;};
        void SetLaserScan(sensor_msgs::LaserScanConstPtr las){ laserScan = *las;}
    private:        
        int sampleSize;
        sensor_msgs::LaserScan laserScan;
        ros::ServiceClient sensorTurn,mvBack;
        ros::Subscriber  updateMapSub,mapSave;
        GridMap* gridMap,*localMap;
        PathPlanner* pathPlanner;
        double yaw,x,y,z,sensor_offset;
        double sensor_angle,range,robo_radius;
    };
}
#endif
