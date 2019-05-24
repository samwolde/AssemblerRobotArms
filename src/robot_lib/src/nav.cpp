#include <robot_lib/nav.h>

//Navigation Commands Implementation
nav::nav(int argc, char ** argv){
    robo_axis_init << 0,-1,0;
    ROS_INFO("nav Node Started");
    ros::init(argc, argv, "sth",ros::init_options::NoSigintHandler);
    rosNode.reset(new ros::NodeHandle("nav_ctr"));
    odomQueueThread = std::thread(std::bind(&nav::odomQueueCb, this));
    linkThread = std::thread(std::bind(&nav::linkQueueCb,this));
    ros::SubscribeOptions so_l = ros::SubscribeOptions::create<gazebo_msgs::LinkStates>(
        "/gazebo/link_states",1,
        boost::bind(&nav::linkState, this, _1),
        ros::VoidPtr(), &linkQueue
    );
    std::string  odom_topic = std::string("/wheely/steering/odom");
    ROS_INFO("Using %s for odometry", odom_topic.c_str());
    ros::SubscribeOptions so_od = ros::SubscribeOptions::create<nav_msgs::Odometry>(
        odom_topic,1,
        boost::bind(&nav::odometryMsg, this, _1),
        ros::VoidPtr(), &odom_queue
    );

    // sub1 = rosNode->subscribe(so);
    sub2 =  rosNode->subscribe(so_od);
    link_state_sub = rosNode->subscribe(so_l);

    mvFrwdC = rosNode->serviceClient<robot_lib::Steering>("/wheely/steering/cmd_moveForward");
    turnLC = rosNode->serviceClient<robot_lib::Steering>("/wheely/steering/cmd_turnLeft");
    turnRC = rosNode->serviceClient<robot_lib::Steering>("/wheely/steering/cmd_turnRight");
    brakeC = rosNode->serviceClient<robot_lib::Steering>("/wheely/steering/cmd_brake");
}


void swap(geometry_msgs::Point &a, geometry_msgs::Point &b){
    geometry_msgs::Point t = a;
    a = b;
    b = t;
}
double getCost(Tour_t tour,int size){
    double cost = sqrt(pow(tour[0].x, 2) + pow(tour[0].y,2));
    for(size_t i = 1; i < size; i++)   
    {
        auto diffx = tour[i].x - tour[i-1].x;
        auto diffy = tour[i].y - tour[i-1].y;
        cost += sqrt(pow(diffx, 2) + pow(diffy,2));
    }
    return cost;
}
void heapPermutation(Tour_t tour, int size, int n, Tour_t shortest) 
{ 
    if (size == 1) 
    { 
        //new Permtuation test if shortest tour
        if ( getCost(tour,n) <  getCost(shortest, n)){
            ROS_INFO("cost current_tour shortest(%f,%f)",getCost(tour,n),getCost(shortest, n));
            memcpy(shortest, tour, n * sizeof(geometry_msgs::Point));
            ROS_INFO("cost current_tour shortest(%f,%f)",getCost(tour,n),getCost(shortest, n));
        }
        return; 
    } 

    for (int i=0; i<size; i++) 
    { 
        heapPermutation(tour,size-1,n,shortest); 
        if (size%2==1) 
            swap(tour[0], tour[size-1]); 
        else
            swap(tour[i], tour[size-1]); 
    } 
} 
bool nav::GetMinTour(robot_lib::MinTour::Request& req, robot_lib::MinTour::Response& res){
    Tour_t tour = (Tour_t)&req.points[0];
    Tour_t shortest = (Tour_t)malloc(sizeof(geometry_msgs::Point) * req.points.size());
    memcpy(shortest, tour, sizeof(geometry_msgs::Point) * req.points.size());
    heapPermutation(tour,  req.points.size(),  req.points.size() ,shortest);
    for(size_t i = 0; i < req.points.size(); i++)
    {
        std::cout << "[*] Node " << i <<" : " << shortest[i].x <<" , "<<  shortest[i].y << std::endl;
    }
    res.orderedPoints.assign(shortest, shortest + req.points.size());
    return true;
}

/* Adjust the orientation betweeen The robots current pose
   and the destination vector dest_vect
*/
bool nav::adjustOrientation(Eigen::Vector3d dest_vect){
    std_msgs::Float32 theta;
    theta.data =  getAngleDiff(dest_vect);
    robot_lib::Steering str;
    ROS_INFO("Theta is %f", theta.data);
    if (theta.data < 0){
        theta.data *= -1;
        str.request.val = theta.data;
        return this->turnRC.call(str);
    }
    else{
        str.request.val = theta.data;
        return this->turnLC.call(str);
    }
    return false;
}
/* Return the angle difference betweeen The robots current pose
   and the destination vector dest_vect
*/
float nav::getAngleDiff(Eigen::Vector3d dest_vect){
    dest_vect -= Eigen::Vector3d(x,y,0.0);
    Eigen::Affine3d rot_mat(Eigen::AngleAxisd(this->yaw, Eigen::Vector3d::UnitZ()));
    double goal_rad;
    bool right = false;
    //Calculate the new robo axis
    //Its axis is rotated by this->yaw from the initial robot axis
    auto robo_axis = rot_mat * robo_axis_init;
    float dot = robo_axis.dot(dest_vect);
    float det  = robo_axis.cross(dest_vect).dot(Eigen::Vector3d::UnitZ());
    return std::atan2(det,dot) * 180/M_PI;
}
/*Given the destination point (vector with initials (0,0,0))
    *It Controls the speed using a proportionla controller
*/
bool nav::controlSpeed(Eigen::Vector3d dest){
    std_msgs::Float32 vel;
    bool accelerate = true;
    double kp = this->kp;
    robot_lib::Steering str;
    // ros::Rate r(120);
    auto distance = (Eigen::Vector3d(x,y,0.0) - dest).norm(); //distance
    ROS_INFO("Controlling Speed");
    while(true){
        auto temp_distance = (Eigen::Vector3d(x,y,0.0) - dest).norm();
        vel.data = kp * temp_distance;
        if( temp_distance <= distanceAccuracy){
            ROS_INFO("Destination Reached!");
            robot_lib::Steering s;
            bool ret = brakeC.call(s);        
            ROS_INFO("Robot state yaw = %f, (%f,%f,%f)", yaw, x,y,z);
            return ret;
        }
        if( fabsf32( getAngleDiff(dest)) > 10 ){
            ROS_INFO("Angle Diverging, Angle difference > 10 degrees, adjusting orientation");
            adjustOrientation(dest);
        }
        //Accelerate for the last 80% of the distance
        if (accelerate && temp_distance < 0.2  * distance ){
            ROS_INFO("Accelerating");
            kp = this->kp + 0.5;
            accelerate = false;
        }
        str.request.val = vel.data;
        if (!mvFrwdC.call(str)){
            return false;
        };
    }
    return false;
}
bool nav::goTo(robot_lib::GoTo::Request& req, robot_lib::GoTo::Response& res){
    auto * pt = &req.pt;
    ROS_INFO("Starting tour...");   
    Eigen::Vector3d destVect;
        destVect << pt->x,
                pt->y, 
                0.0;
    ROS_INFO("Going to node => Dest is (%f, %f)", 
    pt->x,
    pt->y);
    res.s =adjustOrientation(destVect) && controlSpeed(destVect) ;
    return res.s;
}
//Continously update the odometry
void nav::odometryMsg(nav_msgs::OdometryConstPtr odom){
    this->odometry = *odom;
    this->pose = odom->pose.pose.position;
    geometry_msgs::Quaternion q = odom->pose.pose.orientation;
    tf::Quaternion tf_qut(q.x, q.y, q.z, q.w); 
    tf::Matrix3x3(tf_qut).getRPY(this->roll, this->pitch, this->yaw);
    x = odom->pose.pose.position.x;
    y = odom->pose.pose.position.y;
    z = odom->pose.pose.position.z;   
}

void nav::linkState(gazebo_msgs::LinkStatesConstPtr ls){
    static auto set = false;
    if ( set ){return;}
    this->linkStates = *ls;

    set = true;
}
void nav::odomQueueCb(){
    static const double timeout = 0.01;
    while (this->rosNode->ok())
    {
        this->odom_queue.callAvailable(ros::WallDuration(timeout));
    }
}
void nav::linkQueueCb(){
    static const double timeout = 0.01;
    while (this->rosNode->ok())
    {
        this->linkQueue.callAvailable(ros::WallDuration(timeout));
    }
}