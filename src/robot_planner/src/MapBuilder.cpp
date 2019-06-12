#include <robot_planner/MapBuilder.h>

namespace wheely_planner
{
    MapBuilder::MapBuilder(ros::NodeHandlePtr rosNode, GridMap* g,double sensor_of, double sample){
        sensorTurn = rosNode->serviceClient<robot_lib::Sensor>("/wheely/ir_sensor/cmd_turn");
        updateMapSub = rosNode->subscribe("/wheely/slam/updateMap",10,&MapBuilder::UpdateMap,this);
        mapSave = rosNode->subscribe("/wheely/slam/saveMap", 10 , &GridMap::saveMap,g );
        sensor_offset = sensor_of;
        gridMap = g;
        sampleSize = sample;
        marker_pub = rosNode->advertise<visualization_msgs::Marker>("visualization_marker", 10);
    }
    void MapBuilder::UpdateMap(geometry_msgs::PointConstPtr p){
        robot_lib::Sensor sensor;
        ROS_INFO("UpdatingMap...");
        double intervalAngle = 2 * M_PI/sampleSize;
        for (size_t i = 0; i < sampleSize; i++)
        {
            //set sensor angle to i*intervalAngle
            sensor.request.angle = -1 * (i * intervalAngle ) * 180/M_PI; 
            // sensor.request.angle  = (M_PI_2 - i*intervalAngle) * 180/M_PI;
            if ( !sensorTurn.call(sensor) ){
                ROS_INFO("Sensor turning service Failed.");
                return;
            }
            //change the range reading to coordinates of the world frame
            //1st transform from the sensor coordinates to robots
            Coordinate_t c = new(Coordinate);
            c->x = range;
            c->y = 0;
            Tf_From_Sensor_Robo(c,sensor.request.angle * M_PI/180);       
            Tf_From_Robo_World(c);
            // gridMap->storePtForVis(c, &marker);
            //Register on the gridMap
            gridMap->setCellStatus(c->x, c->y);
            delete c;
        }
        gridMap->visualizeMap(&marker_pub);
    }
    void MapBuilder::Tf_From_Sensor_Robo(Coordinate_t c, double angle){
        //Translation Vector S_origin - Robo_Origin
        static Eigen::Affine3d Mt (Eigen::Translation3d(Eigen::Vector3d(sensor_offset,0,0)) );
        //Rotation vector
        Eigen::Affine3d Mr(Eigen::AngleAxisd(angle, Eigen::Vector3d::UnitZ()));

        Eigen::Vector3d co = (Mt * Mr) * Eigen::Vector3d(c->x,0 ,0);
        c->x = co.x();
        c->y = co.y();
    }
    void MapBuilder::Tf_From_Robo_World(Coordinate_t c){
        Eigen::Affine3d Mt (Eigen::Translation3d(Eigen::Vector3d(x,y,0)) );
        //yaw is the angle between -ve Y axis and Robo-X Axis
        //Need to get angle between -ve Y axis and RObo--veY Axis
        double angle = yaw - M_PI/2;   
        //Rotation vector
        Eigen::Affine3d Mr(Eigen::AngleAxisd(angle, Eigen::Vector3d::UnitZ()));
        Eigen::Vector3d co = (Mt * Mr )* Eigen::Vector3d(c->x, c->y ,0);
        c->x = co.x();
        c->y = co.y();
    }
    void MapBuilder::SetXYZ_Yaw(double _x,double _y,double _z,double _yaw){
        x = _x;
        y = _y;
        z = _z;
        yaw = _yaw;
    }
}