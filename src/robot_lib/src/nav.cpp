#include <robot_lib/nav.h>

//Navigation Commands Implementation
void shh(int sig){
    ROS_INFO("Caught Signal");
}
nav::nav(int argc, char ** argv){
    robo_axis_init << 0,-1,0;
    ROS_INFO("nav Node Started");
    signal(SIGCONT, shh);
    ros::init(argc, argv, "sth",ros::init_options::NoSigintHandler);
    rosNode.reset(new ros::NodeHandle("nav_ctr"));
    odomQueueThread = std::thread(std::bind(&nav::odomQueueCb, this));
    linkThread = std::thread(std::bind(&nav::linkQueueCb,this));
    shrtSensorTh = std::thread(std::bind(&nav::shrtSensorQueueCb,this));
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
    ros::SubscribeOptions so_sens = ros::SubscribeOptions::create<sensor_msgs::Range>(
        "/wheely/sensor/ir_shrt",1,
        boost::bind(&nav::shortSensorMsg, this, _1),
        ros::VoidPtr(), &shrtSensorqueue
    );

    // sub1 = rosNode->subscribe(so);
    sub2 =  rosNode->subscribe(so_od);
    link_state_sub = rosNode->subscribe(so_l);
    sensor_sub = rosNode->subscribe(so_sens);

    mvFrwdC = rosNode->serviceClient<robot_lib::Steering>("/wheely/steering/cmd_moveForward");
    turnLC = rosNode->serviceClient<robot_lib::Steering>("/wheely/steering/cmd_turnLeft");
    turnRC = rosNode->serviceClient<robot_lib::Steering>("/wheely/steering/cmd_turnRight");
    brakeC = rosNode->serviceClient<robot_lib::Steering>("/wheely/steering/cmd_brake");
    mvBack = rosNode->serviceClient<robot_lib::Steering>("/wheely/nav/moveBackward_srv");
    //  /wheely/sensor/ir_shrt
}

void nav::shortSensorMsg(sensor_msgs::RangeConstPtr range){
    this->range = range->range;
}
/* Adjust the orientation betweeen The robots current pose
   and the destination vector dest_vect
*/
bool nav::adjustOrientation(Eigen::Vector3d dest_vect){
    std_msgs::Float32 theta;
    theta.data =  getAngleDiff(dest_vect);
    dest_vect[2] = 0;
    robot_lib::Steering str;
    if( fabs(theta.data) <= 15 ) return true;
    if (theta.data < 0){
        theta.data *= -1;
        str.request.val = theta.data;
        // pause();
        return this->turnRC.call(str);
    }
    else{
        str.request.val = theta.data;
        // pause();
        return this->turnLC.call(str);
    }
    return false;
}
/* Return the angle difference betweeen The robots current pose (front vector)
   and the destination vector dest_vect
*/
float nav::getAngleDiff(Eigen::Vector3d dest_vect){
    dest_vect[2] = 0.0;
    dest_vect -= Eigen::Vector3d(x,y,0.0);
    Eigen::Affine3d rot_mat(Eigen::AngleAxisd(this->yaw, Eigen::Vector3d::UnitZ()));
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
bool nav::controlSpeed(Eigen::Vector3d dest,bool isBegin, bool isFinalDest,bool detectObstacles){
    std_msgs::Float32 vel;
    bool accelerate = true;
    ros::Rate r(100);
    dest[2] = 0.0;
    robot_lib::Steering str;
    // ros::Rate r(120);
    double err = (Eigen::Vector3d(x,y,0.0) - dest).norm(); //distance
    double prev_err = err;
    while(true){
        //if obstacle on the way return.
        if(detectObstacles && range <= CLOSE_RANGE){
            ROS_INFO("CLOSE RANGE OBSTACLE...");
            robot_lib::Steering s;
            s.request.val = 0.6;
            mvBack.call(s);
            sleep(2);
            brakeC.call(s);     
            return false;
        }
        err = (Eigen::Vector3d(x,y,0.0) - dest).norm();  
        if( err <= distanceAccuracy){
            robot_lib::Steering s;
            bool ret = true;
            if ( isFinalDest )
                ret = brakeC.call(s);        
            return ret;
        }
        auto current = odometry.twist.twist.linear.x;
        if( isBegin || current <= 1.2){
            str.request.val = 1.2;
            if (!mvFrwdC.call(str)){
                return false;
            };
        }
        // r.sleep();
        prev_err = err;
    }
    return false;
}
bool nav::goTo(robot_lib::GoTo::Request& req, robot_lib::GoTo::Response& res){
    ROS_INFO("Starting tour...");  
    auto begin =req.path.front();
    auto end =req.path.back();
    ROS_INFO("Going to node beign is (%f,%f) => Dest is (%f, %f)", 
    begin.x,
    begin.y, 
    end.x,
    end.y);
    bool isBegin, isEnd;
    for (auto pt : req.path)
    {
        Eigen::Vector3d destVect;
        destVect << pt.x,
                    pt.y, 
                    0;
        isBegin =pointsEqual(&pt, &begin);
        isEnd = pointsEqual(&pt, &end) ;
        res.s = adjustOrientation(destVect) && controlSpeed(destVect,isBegin,isEnd,req.detectObstacles);
        if ( !res.s) return res.s;   
    }
    return true;
}
bool nav::pointsEqual(geometry_msgs::Point* p1, geometry_msgs::Point* p2){
    return p1->x == p2->x && p1->y == p2->y;
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
void nav::shrtSensorQueueCb(){
    static const double timeout = 0.01;
    while (this->rosNode->ok())
    {
        this->shrtSensorqueue.callAvailable(ros::WallDuration(timeout));
    }
}
