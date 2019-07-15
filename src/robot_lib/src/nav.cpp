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
    shrtSensorTh[0] = std::thread([this](){this->shrtSensorQueueCb(0);});
    shrtSensorTh[1] = std::thread([this](){this->shrtSensorQueueCb(1);});
    shrtSensorTh[2] = std::thread([this](){this->shrtSensorQueueCb(2);});
    shrtSensorTh[3] = std::thread([this](){this->shrtSensorQueueCb(3);});
    std::string  odom_topic = std::string("/wheely/steering/odom");
    ROS_INFO("Using %s for odometry", odom_topic.c_str());
    ros::SubscribeOptions so_od = ros::SubscribeOptions::create<nav_msgs::Odometry>(
        odom_topic,1,
        boost::bind(&nav::odometryMsg, this, _1),
        ros::VoidPtr(), &odom_queue
    );
    ros::SubscribeOptions so_sens_front = ros::SubscribeOptions::create<sensor_msgs::Range>(
        "/wheely/sensor/ir_shrt_front",1,
        boost::bind(&nav::shortSensorMsg, this, _1),
        ros::VoidPtr(), &shrtSensorqueue[0]
    );
    ros::SubscribeOptions so_sens_left = ros::SubscribeOptions::create<sensor_msgs::Range>(
        "/wheely/sensor/ir_shrt_left",1,
        boost::bind(&nav::shortSensorMsg, this, _1),
        ros::VoidPtr(), &shrtSensorqueue[1]
    );
    ros::SubscribeOptions so_sens_right = ros::SubscribeOptions::create<sensor_msgs::Range>(
        "/wheely/sensor/ir_shrt_right",1,
        boost::bind(&nav::shortSensorMsg, this, _1),
        ros::VoidPtr(), &shrtSensorqueue[2]
    );
    ros::SubscribeOptions so_sens_rear = ros::SubscribeOptions::create<sensor_msgs::Range>(
        "/wheely/sensor/ir_shrt_rear",1,
        boost::bind(&nav::shortSensorMsg, this, _1),
        ros::VoidPtr(), &shrtSensorqueue[3]
    );

    // sub1 = rosNode->subscribe(so);
    sub2 =  rosNode->subscribe(so_od);
    sensor_sub[0] = rosNode->subscribe(so_sens_front);
    sensor_sub[1] = rosNode->subscribe(so_sens_left);
    sensor_sub[2] = rosNode->subscribe(so_sens_right);
    sensor_sub[3] = rosNode->subscribe(so_sens_rear);

    mvFrwdC = rosNode->serviceClient<robot_lib::Steering>("/wheely/steering/cmd_moveForward");
    turnLC = rosNode->serviceClient<robot_lib::Steering>("/wheely/steering/cmd_turnLeft");
    turnRC = rosNode->serviceClient<robot_lib::Steering>("/wheely/steering/cmd_turnRight");
    brakeC = rosNode->serviceClient<robot_lib::Steering>("/wheely/steering/cmd_brake");
    mvBack = rosNode->serviceClient<robot_lib::Steering>("/wheely/nav/moveBackward_srv");
    //  /wheely/sensor/ir_shrt
}

void nav::shortSensorMsg(sensor_msgs::RangeConstPtr range){
    static int count[4] = {0};
    if ( !range->header.frame_id.compare(FRONT) ){/* count[0]++*/; SET_FRONT_RANGE(this->range,range->range)}
    if ( !range->header.frame_id.compare(RIGHT) ){/* count[1]++*/; SET_RIGHT_RANGE(this->range,range->range)}
    if ( !range->header.frame_id.compare(LEFT) ){ /*count[2]++;*/ SET_LEFT_RANGE(this->range,range->range)}
    if ( !range->header.frame_id.compare(REAR) ){ /*count[3]++;*/ SET_REAR_RANGE(this->range,range->range)}
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
        if(detectObstacles && checkForObstacle()){     
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
        if( isBegin || current <= 1.5){
            str.request.val = 1.5;
            if (!mvFrwdC.call(str)){
                return false;
            };
        }
        // r.sleep();
        prev_err = err;
    }
    return false;
}
/**
 * Check if the robot is bloked in any of the four direction and unblock it.
 */
bool nav::checkForObstacle(){
    static const double scale = 1.4;
    static const double turnAngle = 30;
    static const double roboRadius = 0.3;

    if ( !( CHECK_FRONT_RANGE(this->range,1) ||  CHECK_RIGHT_RANGE(this->range,1) 
        ||CHECK_LEFT_RANGE(this->range,1)) ){
        //There is no obstacle
        return false;
    }
    ROS_INFO("CLOSE RANGE OBSTACLE...");
    robot_lib::Steering s;
    while ( true ){
        //Unblock robot by a bit, handle individual cases.
        //Break if robot unblocked or is impossible to unblock robot
        ROS_INFO("ADJUSTING ROBOT AWAY FROM OBSTACLE");
        if ( CHECK_FRONT_RANGE(this->range,scale) ){
            //Handle Front blocked case
            ROS_INFO("FRONT OBSTACLE");
            brakeC.call(s);    
            double backUpspace = 1.5 * CLOSE_RANGE +  CLOSE_RANGE - FRONT_RANGE ;
            if( CHECK_REAR_RANGE(this->range, backUpspace/CLOSE_RANGE)){
                //Can't unblock, not enough space to move back
                ROS_INFO("Not enough room ... space needed %f, RANGE %f", backUpspace, REAR_RANGE);
                break;
            }
            //Go back enough to unblock the vehicle
            s.request.val =2 * ( scale + 0.1 ) * CLOSE_RANGE;
            mvBack.call(s);
            ros::Duration(0.6).sleep();
            brakeC.call(s);
            ROS_INFO("backup space %f, turn angle %f, requ speed %f",backUpspace, turnAngle,s.request.val );

        }
        else if ( CHECK_RIGHT_RANGE(this->range,scale) ){
            //Handle Right blocked case
            ROS_INFO("RIGHT OBSTACLE");
            brakeC.call(s);  
            if ( CHECK_FRONT_RANGE(this->range,scale) ) {          
                continue;
            }
            double backUpspace = scale * CLOSE_RANGE + sin(turnAngle) * roboRadius ;
            if( CHECK_LEFT_RANGE(this->range,backUpspace/CLOSE_RANGE) ){
                //Not enough room to make a turn
                ROS_INFO("Not enough room ... space needed %f, RANGE %f",backUpspace,LEFT_RANGE );
                break;
            }
            s.request.val =turnAngle;
            turnLC.call(s);
            s.request.val =2 * ( scale + 0.1 ) * CLOSE_RANGE;            
            mvFrwdC.call(s);
            ros::Duration(0.6).sleep();            
            brakeC.call(s);
            s.request.val =turnAngle;
            turnRC.call(s);
            ROS_INFO("backup space %f, turn angle %f, requ speed %f",backUpspace, turnAngle,CLOSE_RANGE);
        }   
        else if ( CHECK_LEFT_RANGE(this->range,1) ){
            //Handle Left blocked case
            ROS_INFO("LEFT OBSTACLE");
            brakeC.call(s);    
            if ( CHECK_FRONT_RANGE(this->range,scale) ) {        
                        continue;
            }
            double backUpspace = 1.5 * CLOSE_RANGE + sin(turnAngle) * roboRadius ;
            if( CHECK_RIGHT_RANGE(this->range,backUpspace/CLOSE_RANGE) ){
                //Not enough room to make a turn
                ROS_INFO("Not enough room ... space needed %f, RANGE %f",backUpspace, RIGHT_RANGE);
                break;
            }
            s.request.val =turnAngle;
            turnRC.call(s);
            s.request.val =2 * ( scale + 0.1 ) * CLOSE_RANGE;            
            mvFrwdC.call(s);
            ros::Duration(0.6).sleep();  
            brakeC.call(s);
            s.request.val =turnAngle;
            turnLC.call(s);
            ROS_INFO("backup space %f, turn angle %f, requ speed %f",backUpspace, turnAngle,CLOSE_RANGE);
        }
        else break;
    }
   
    return true;
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
void nav::odomQueueCb(){
    static const double timeout = 0.01;
    while (this->rosNode->ok())
    {
        this->odom_queue.callAvailable(ros::WallDuration(timeout));
    }
}
void nav::shrtSensorQueueCb(int i){
    static const double timeout = 0.01;
    while (this->rosNode->ok())
    {
        this->shrtSensorqueue[i].callAvailable(ros::WallDuration(timeout));
    }
}
