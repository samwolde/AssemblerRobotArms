#include <robot_planner/Slam.h>
#include <sensor_msgs/LaserScan.h>


namespace wheely_planner
{
    void shh(int a){
    ROS_INFO("cAUGHt signal");
    }
    Slam::Slam(){
        ROS_INFO("Slam Node Started");
        init();
        gridMap = new GridMap(mapSize,cellSize);
        mapBuilder = new MapBuilder(rosNode,gridMap, sensor_offset,sampleSize);
    }
    Slam::Slam(MapFile map){
        ROS_INFO("Slam Node Started");
        init();
        signal(10, shh);
        gridMap = new GridMap(map,robo_radius);
        mapBuilder = new MapBuilder(rosNode,gridMap, sensor_offset,sampleSize);
        pathPlanner  = new PathPlanner(rosNode,gridMap,kh);
    }
    void Slam::init(){
        rosNode.reset(new ros::NodeHandle("~"));
        rosNode->getParam("robo_radius",robo_radius);
        rosNode->getParam("sensor_offset",sensor_offset);
        rosNode->getParam("sampleSize",sampleSize);
        rosNode->getParam("mapSize",mapSize);
        rosNode->getParam("cellSize",cellSize);
        rosNode->getParam("kh",kh);
        // rosNode->para
        ROS_INFO("\nParameters kh %d, robo_radius %lf, sensor_offset %lf\n", kh, robo_radius, sensor_offset);
        odomQueueThread = std::thread(std::bind(&Slam::odomQueueCb, this));
        rangeThread = std::thread(std::bind(&Slam::rangeQueueCb, this));
        sense_th = std::thread(std::bind(&Slam::senseQueueCb, this));
        std::string  odom_topic = std::string("/wheely/steering/odom");
        ROS_INFO("Slam : Using %s for odometry", odom_topic.c_str());
        ros::SubscribeOptions so_od = ros::SubscribeOptions::create<nav_msgs::Odometry>(
            odom_topic,1,
            boost::bind(&Slam::odometryMsg, this, _1),
            ros::VoidPtr(), &odom_queue
        );
        ros::SubscribeOptions so_range = ros::SubscribeOptions::create<sensor_msgs::Range>(
            "/wheely/sensor/ir",1,
            boost::bind(&Slam::rangeMsg, this, _1),
            ros::VoidPtr(), &range_queue
        );
        ros::SubscribeOptions so_sens = ros::SubscribeOptions::create<std_msgs::Float32>(
            "/wheely/ir_sensor/state",1,
            boost::bind(&Slam::senseMsg, this, _1),
            ros::VoidPtr(), &sense_queue
        );
        sub =  rosNode->subscribe(so_od);
        range_sub = rosNode->subscribe(so_range);
        sens_sub = rosNode->subscribe(so_sens);
        pub = rosNode->advertise<geometry_msgs::Point>("/wheely/slam/world_coordinates",10);
    }
    void Slam::senseMsg(std_msgs::Float32ConstPtr state){
        sensor_angle = state->data;
        auto c = new(Coordinate);
        c->x = range;
        c->y = 0;
        mapBuilder->Tf_From_Sensor_Robo(c,sensor_angle);       
        mapBuilder->Tf_From_Robo_World(c);
        geometry_msgs::Point p;
        p.x = c->x;
        p.y = c->y;
        pub.publish(p);
    }
    void Slam::rangeMsg(sensor_msgs::RangeConstPtr r){
        this->range = r->range;
        mapBuilder->SetRange(range);
    }
    void Slam::odometryMsg(nav_msgs::OdometryConstPtr odom){
        this->odometry = *odom;
        this->pose = odom->pose.pose.position;
        geometry_msgs::Quaternion q = odom->pose.pose.orientation;
        tf::Quaternion tf_qut(q.x, q.y, q.z, q.w); 
        tf::Matrix3x3(tf_qut).getRPY(this->roll, this->pitch, this->yaw);
        x = odom->pose.pose.position.x;
        y = odom->pose.pose.position.y;
        z = odom->pose.pose.position.z;   
        mapBuilder->SetXYZ_Yaw(x,y,z,yaw);
        pathPlanner->setXY(x,y);
        // ROS_INFO("x: %f,y: %f,z: %f, yaw: %f", x,y,z,yaw);
    }
    void Slam::odomQueueCb(){
        static const double timeout = 0.01;
        while (this->rosNode->ok())
        {
            this->odom_queue.callAvailable(ros::WallDuration(timeout));
        }
    }
    void Slam::rangeQueueCb(){
        static const double timeout = 0.01;
        while (this->rosNode->ok())
        {
            this->range_queue.callAvailable(ros::WallDuration(timeout));
        }
    }

    void Slam::senseQueueCb(){
        static const double timeout = 0.01;
        while (this->rosNode->ok())
        {
            this->sense_queue.callAvailable(ros::WallDuration(timeout));
        }
    }
} // namespace wheely_planner

static int mapFilter(const struct dirent* dir_ent);
int main(int argc, char** argv){    
    ros::init(argc,argv,"Slam_ros");
    //Search for a map file
    wheely_planner::Slam *slam;
    struct dirent **names;
    int n = scandir(ros::package::getPath(THIS_ROS_PACKAGE_NAME).append("/map/").c_str(), &names,mapFilter, alphasort);
    if ( n <= 0) {
        ROS_INFO("Slam :: No Map found.");
        slam = new wheely_planner::Slam();
    }
    else if( n > 0){
        ROS_INFO("Found a map file, Using %s as a map file.",names[0]->d_name);
        slam = new wheely_planner::Slam(names[0]->d_name);
    }
    ros::spin();
}

static int mapFilter(const struct dirent* dir_ent)
{
    //Skip . and ..
    if (!strcmp(dir_ent->d_name, ".") || !strcmp(dir_ent->d_name, "..")) return 0;
    std::string fname = dir_ent->d_name;
    if (fname.find_last_of(".map") != std::string::npos ) {
        return 1;
    }
    return 0;
}