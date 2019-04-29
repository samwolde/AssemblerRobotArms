#include <functional>
#include <iostream>
#include <regex>
#include <gazebo/gazebo.hh>
#include <gazebo/physics/physics.hh>
#include <gazebo/common/common.hh>
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

namespace gazebo{

    //represents a graph node.
    struct Node
    {
        double coordiates[2];
        int id;
    };
    //An order array of Nodes.
    typedef  struct Node* Tour_t;
    class tsp : public ModelPlugin
    {
        public:
            void Load(physics::ModelPtr parent, sdf::ElementPtr sdf_ptr);
            Tour_t GetMinTour();
            double GetError(double prev_yaw, double goal_yaw, bool right);
            bool HasStoped();
            double GetGoalRad(double rad, double yaw, bool right);
            void controlSpeed(Eigen::Vector3d v);
            void adjustOrientation(Eigen::Vector3d dest_vect);
            void odometryMsg(nav_msgs::OdometryConstPtr odom);
            void start_tour(std_msgs::Float32ConstPtr);
            void linkState(gazebo_msgs::LinkStatesConstPtr);
            void cmdQueueCb();
            void odomQueueCb();
            void linkQueueCb();
            void OnUpdate(){};

        private:

            ros::Publisher mvFrwdPub,mvBackPub,turnRPub,turnLPub,brakePub;
            ros::CallbackQueue cmd_queue, odom_queue,linkQueue;
            ros::Subscriber sub1,sub2,link_state_sub ;
            std::thread odomQueueThread, cmdQueueThread, linkThread;
            nav_msgs::Odometry odometry;
            geometry_msgs::Point pose;
            physics::ModelPtr model;
            event::ConnectionPtr con;
            std::unique_ptr<ros::NodeHandle> rosNode;
            sdf::ElementPtr sdf_ptr, world,boxes;
            double roll, pitch, yaw,x,y,z;
            Eigen::Vector3d robo_axis_init;
            gazebo_msgs::LinkStates linkStates;
            double turnAccuracy=0.001,distanceAccuracy=0.25,kp=0.1;
            
    };
    void tsp::Load(physics::ModelPtr parent, sdf::ElementPtr sdf_p){
        this->model = parent;
        sdf_ptr = sdf_p;
        world = sdf_ptr->GetParent()->GetParent();
        robo_axis_init << 0,-1,0;
		ROS_INFO("TSP Plugin Loaded");

		if(!ros::isInitialized()){
			ROS_INFO("Ros not initialiazed");
			int argc = 0;
			char **argv = NULL;
			ros::init(argc, argv, "gazebo_client",ros::init_options::NoSigintHandler);
		}
        rosNode.reset(new ros::NodeHandle("tsp_ctr"));
        odomQueueThread = std::thread(std::bind(&tsp::odomQueueCb, this));
        cmdQueueThread = std::thread(std::bind(&tsp::cmdQueueCb, this));
        linkThread = std::thread(std::bind(&tsp::linkQueueCb,this));

        ros::SubscribeOptions so = ros::SubscribeOptions::create<std_msgs::Float32>(
            "/wheely/nav/start",1,
            boost::bind(&tsp::start_tour, this, _1),
            ros::VoidPtr(), &cmd_queue
        );
        ros::SubscribeOptions so_l = ros::SubscribeOptions::create<gazebo_msgs::LinkStates>(
            "/gazebo/link_states",1,
            boost::bind(&tsp::linkState, this, _1),
            ros::VoidPtr(), &linkQueue
        );
        std::string  odom_topic = std::string("/wheely/steering/odom");
        if ( sdf_p->HasElement("odom") ) {
            odom_topic =  sdf_p->GetElement("odom")->GetValue()->GetAsString();
        }
        
        if( sdf_p->HasElement("turnAccuracy") ){
            turnAccuracy  = atof(sdf_p->GetElement("turnAccuracy")->GetValue()->GetAsString().c_str());
        }

        if( sdf_p->HasElement("distanceAccuracy") ){
            distanceAccuracy  = atof(sdf_p->GetElement("distanceAccuracy")->GetValue()->GetAsString().c_str());
        }
        if( sdf_p->HasElement("kp") ){
            kp  = atof(sdf_p->GetElement("kp")->GetValue()->GetAsString().c_str());
        }
        ROS_INFO("Using %s for odometry", odom_topic.c_str());
        ros::SubscribeOptions so_od = ros::SubscribeOptions::create<nav_msgs::Odometry>(
            odom_topic,1,
            boost::bind(&tsp::odometryMsg, this, _1),
            ros::VoidPtr(), &odom_queue
        );
        sub1 = rosNode->subscribe(so);
        sub2 =  rosNode->subscribe(so_od);
        link_state_sub = rosNode->subscribe(so_l);

        mvFrwdPub = rosNode->advertise<std_msgs::Float32>("/wheely/steering/cmd_moveForward",10);
        mvBackPub = rosNode->advertise<std_msgs::Float32>("/wheely/steering/cmd_moveBack",10);
        turnRPub = rosNode->advertise<std_msgs::Float32>("/wheely/steering/cmd_turnRight",10);
        turnLPub = rosNode->advertise<std_msgs::Float32>("/wheely/steering/cmd_turnLeft",10);
        brakePub = rosNode->advertise<std_msgs::Float32>("/wheely/steering/cmd_brake",10);
		this->con = event::Events::ConnectWorldUpdateBegin(std::bind(&tsp::OnUpdate, this));
    }


    void swap(Node &a, Node &b){
        Node t = a;
        a = b;
        b = t;
    }
    double getCost(Tour_t tour,int size){
        double cost = sqrt(pow(tour[0].coordiates[0], 2) + pow(tour[0].coordiates[1],2));
        for(size_t i = 1; i < size; i++)   
        {
            auto diffx = tour[i].coordiates[0] - tour[i-1].coordiates[0];
            auto diffy = tour[i].coordiates[1] - tour[i-1].coordiates[1];
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
                memcpy(shortest, tour, n * sizeof(Node));
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
    Tour_t tsp::GetMinTour(){
        static std::regex r("boxes::box[0-9]");
        static std::smatch sm;
        Tour_t tour = (Tour_t)malloc(sizeof(struct Node) * 5);
        Tour_t shortest = (Tour_t)malloc(sizeof(struct Node) * 5);
        int j=0;
        for(size_t i = 0; i < linkStates.name.size(); i++)
        {
            std::regex_match( linkStates.name.at(i) , sm, r);
            if ( !sm.empty() && j <5 ){
                shortest[j].id= tour[j].id = j;
                shortest[j].coordiates[0]= tour[j].coordiates[0] = linkStates.pose.at(i).position.x;
                shortest[j].coordiates[1] =tour[j].coordiates[1] = linkStates.pose.at(i).position.y;
                j++;
            }   
        }
        heapPermutation(tour, 5,5,shortest);
        for(size_t i = 0; i < 5; i++)
        {
            std::cout << "[*] Node " << i << shortest[i].coordiates[0] << shortest[i].coordiates[1] << std::endl;
        }
        return shortest;
    }
    double tsp::GetGoalRad(double rad, double yaw, bool right){
		double goal = yaw + rad;
		if ( right && goal < - M_PI){
			//Normalize goal, -pi <= goal <= pi
			goal += 2  * M_PI;
		}
		if ( !right && goal > M_PI){
			//Normalize,  -pi <= goal <= pi
			goal -= 2* M_PI;
		}
		return goal;
	};
    bool tsp::HasStoped(){
            geometry_msgs::Twist current_vel = this->odometry.twist.twist;
            current_vel.linear.x=trunc(current_vel.linear.x*1000);
            current_vel.linear.y=trunc(current_vel.linear.y*1000);
            current_vel.linear.z=trunc(current_vel.linear.z*1000);
            current_vel.angular.z=trunc(current_vel.angular.z*1000);			
            bool stoped = 
            (current_vel.linear.x == 0 && current_vel.linear.y == 0 && current_vel.linear.z == 0 && current_vel.angular.z == 0);
            return stoped;
    }
    double tsp::GetError(double prev_yaw, double goal_yaw, bool right){
		//Distance
		double err = std::fabs(goal_yaw - prev_yaw);
		if( right && prev_yaw < goal_yaw ){
			err = 2 * M_PI -err;
		}
		if ( !right && goal_yaw < prev_yaw){
			err  = 2 * M_PI -err;
		}
		return err;
	};
    void tsp::adjustOrientation(Eigen::Vector3d dest_vect){
        dest_vect -= Eigen::Vector3d(x,y,0.0);
        brakePub.publish(std_msgs::Float32());
        Eigen::Affine3d rot_mat(Eigen::AngleAxisd(this->yaw, Eigen::Vector3d::UnitZ()));
        double goal_rad;
        bool right = false;
        //Calculate the new robo axis
        //Its axis is rotated by this->yaw from the initial robot axis
        auto robo_axis = rot_mat * robo_axis_init;
        float dot = robo_axis.dot(dest_vect);
        float det  = robo_axis.cross(dest_vect).dot(Eigen::Vector3d::UnitZ());
        std_msgs::Float32 theta;
        theta.data =  std::atan2(det,dot) * 180/M_PI;
        
        ROS_INFO("Theta is %f", theta.data);
        if (theta.data < 0){
            theta.data *= -1;
            right = true;
            goal_rad = GetGoalRad(theta.data, this->yaw, true);
            this->turnRPub.publish(theta);
        }
        else{
            goal_rad = GetGoalRad(theta.data, this->yaw, false);
            this->turnLPub.publish(theta);
        }
        ros::Rate r(400);
        //Need to make sure orientation has been adjusted.
        while( GetError(this->yaw, goal_rad,right) > turnAccuracy && !HasStoped() ){
            //Wait till its done turning!
            // r.sleep();
        }
    }
    /*Given the destination point (vector with initials (0,0,0))
     *It Controls the speed using a proportionla controller
    */
    void tsp::controlSpeed(Eigen::Vector3d dest){
        std_msgs::Float32 vel;
        // ros::Rate r(120);
        auto distance = (Eigen::Vector3d(x,y,0.0) - dest).norm(); //distance
        while(true){
            auto temp_distance = (Eigen::Vector3d(x,y,0.0) - dest).norm();
            vel.data = this->kp * temp_distance;
            if( temp_distance <= distanceAccuracy){
                ROS_INFO("Destination Reached!");
                brakePub.publish(std_msgs::Float32());        
                ros::Rate r(30);
                while(!HasStoped()){r.sleep();};
                ROS_INFO("Robot state yaw = %f, (%f,%f,%f)", yaw, x,y,z);
                return;
            }
            if ( temp_distance < 0.2  * distance ){
                distance = (Eigen::Vector3d(x,y,0.0) - dest).norm();
                adjustOrientation(dest);
            }
            // if ( temp_distance < 0.4)
            //     ROS_INFO("Distance %f, " , temp_distance);
            mvFrwdPub.publish(vel);
            // r.sleep();
        }
    }
    void tsp::start_tour(std_msgs::Float32ConstPtr fl_){
        ROS_INFO("Starting tour...");   
        Tour_t tour = GetMinTour();
        for(size_t i = 0; i < 5; i++)
        {
            Eigen::Vector3d destVect;
            destVect << tour[i].coordiates[0],
                 tour[i].coordiates[1], 
                 0.0;
            ROS_INFO("Going to node %ld => Dest is (%f, %f)\nvector is(%f,%f,%f)...", i, 
            tour[i].coordiates[0],
            tour[i].coordiates[1], 
            destVect.x(),destVect.y(),destVect.z());
            adjustOrientation(destVect);
            controlSpeed(destVect);
        }
    }
    //Continously update the odometry
    void tsp::odometryMsg(nav_msgs::OdometryConstPtr odom){
        this->odometry = *odom;
        this->pose = odom->pose.pose.position;
        geometry_msgs::Quaternion q = odom->pose.pose.orientation;
		tf::Quaternion tf_qut(q.x, q.y, q.z, q.w); 
		tf::Matrix3x3(tf_qut).getRPY(this->roll, this->pitch, this->yaw);
        x = odom->pose.pose.position.x;
        y = odom->pose.pose.position.y;
        z = odom->pose.pose.position.z;   
    }

    void tsp::linkState(gazebo_msgs::LinkStatesConstPtr ls){
        static auto set = false;
        if ( set ){return;}
        this->linkStates = *ls;
        set = true;
    }
    void tsp::cmdQueueCb(){
        static const double timeout = 0.01;
		while (this->rosNode->ok())
		{
			this->cmd_queue.callAvailable(ros::WallDuration(timeout));
		}
    }
    void tsp::odomQueueCb(){
        static const double timeout = 0.01;
		while (this->rosNode->ok())
		{
			this->odom_queue.callAvailable(ros::WallDuration(timeout));
		}
    }
     void tsp::linkQueueCb(){
        static const double timeout = 0.01;
		while (this->rosNode->ok())
		{
			this->linkQueue.callAvailable(ros::WallDuration(timeout));
		}
    }
    
GZ_REGISTER_MODEL_PLUGIN(tsp)
}