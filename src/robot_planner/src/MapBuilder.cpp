#include <robot_planner/MapBuilder.h>

namespace wheely_planner
{
    MapBuilder::MapBuilder(ros::NodeHandlePtr rosNode, GridMap* g,PathPlanner* pathPlanner,double sensor_of, double sample){
        sensorTurn = rosNode->serviceClient<robot_lib::Sensor>("/wheely/ir_sensor/cmd_turn");
        updateMapSub = rosNode->subscribe("/wheely/slam/BuildMap",10,&MapBuilder::BuildMap,this);
        localMap = new GridMap(g);
        mapSave = rosNode->subscribe("/wheely/slam/saveMap", 10 , &GridMap::saveMap,localMap );
        gridMap = g;
        sensor_offset = sensor_of;
        rosNode->getParam("robo_radius",robo_radius);
        sampleSize = sample;
        this->pathPlanner = pathPlanner;
    }
    void MapBuilder::BuildMap(geometry_msgs::PointConstPtr unreachablePt){
        auto goal_pt =Coordinate(unreachablePt->x, unreachablePt->y);
        while ( true ){
            UpdateMap();
            auto current = Coordinate(x,y);
            auto goal = pathPlanner->A_S_PlanPath(current,goal_pt);
            if ( !goal.parent  ){
                if( !CheckBlocked(&goal_pt) ){
                    break;
                }
                //Robot might be caught in some obstacle on the map
                continue;
            }
            auto path = pathPlanner->constructPath(&goal);
            if (!pathPlanner->FollowPath(&path)){   
                //Until an obstacle avoidance algorithm, just backup a bit 
                continue;
            }
            ROS_INFO("Reachead at goal ... ");
            break;
        }
        ROS_INFO("YAY.. DONE BUILDING MAP.");
        localMap->visualizeMap(COLOR_DARK_RED,5);
        gridMap->visualizeMap();
    }   
    bool MapBuilder::CheckBlocked(Coordinate_t goal_pt){
        static auto last_blocked = ros::Time::now();
        static std::queue<Coordinate>  valids;
        static bool first = true;

        auto canUnblock = false;
        auto c = Coordinate(x,y);
        if( gridMap->getCellStatus(&c) ){
            //UnBlock robot,find nearest unoccupied cell and plan from there
            auto timeout = ros::Time::now().toSec() - last_blocked.toSec();
            valids = timeout > 4 || first?  gridMap->getNearestUnoccupied(&c): valids;
            first = false;
            ROS_INFO("timeout i %f,valids size %ld",timeout,valids.size( ));
            if( !valids.empty() ) {
                auto done = false;
                std::stack<Cell*> path;
                while ( !done && !valids.empty()  ){
                    auto goal = pathPlanner->A_S_PlanPath(valids.front(),*goal_pt);
                    valids.pop();
                    path = pathPlanner->constructPath(&goal);
                    done = !path.empty();
                }
                if( done)
                    pathPlanner->FollowPath(&path);
                canUnblock = done;
            }
        }
        last_blocked = ros::Time::now();
        return canUnblock;
    }
    void MapBuilder::UpdateMap(){
        robot_lib::Sensor sensor;
        std::unordered_map<Index,bool,IndexKeyHasher> init;

        ROS_INFO("UpdatingMap...");
        /**
         * Perform 4 scans 2 at 0 degrees and 2 at 180 degrees,
         * In order for a cell to be occupied must exist in two of the scans.
         */
        for (size_t j = 0; j < 4; j++)
        {
            for (size_t i = 0; i < laserScan.ranges.size(); i++)
            {
                double sens_angle =  j < 2 ? 0 : M_PI;
                double angle = laserScan.angle_min + i * laserScan.angle_increment + sens_angle;
                //change the range reading to coordinates of the world frame
                //1st transform from the sensor coordinates to robots
                Coordinate_t c = new(Coordinate);
                if ( laserScan.ranges[i] < laserScan.range_min || laserScan.ranges[i] >laserScan.range_max )
                    continue;

                c->x = laserScan.ranges[i];
                c->y = 0;
                Tf_From_Sensor_Robo(c,angle);       
                Tf_From_Robo_World(c);
                //Register on the gridMap, if point exists on both scans
                Index * index = localMap->computeCellIndex(c->x,c->y);            
                if ( !index ) continue;
                // //If first scan 
                if ( j % 2 == 0 && init.find(*index) == init.end() ){
                    init[*index] = true;
                }
                else if( j%2 == 1 ){
                    //Filter scans they must exist in both scans inorder to be valid
                    if ( !(init.find(*index) == init.end() )){
                        localMap->setCellStatus(index->cx,index->cy);
                    }
                }
                delete index;
                delete c;
            }
            if ( j %2 == 1) {
                //Compare the two scans setCellStatus of common elements
                init.clear();
            }
            sensor.request.angle = j < 1 || j == 3 ? 0 : 180;
            sensorTurn.call(sensor);
        }
        localMap->EnlargeObstacles(gridMap);
        localMap->visualizeMap(COLOR_DARK_RED,5);
        gridMap->visualizeMap();
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