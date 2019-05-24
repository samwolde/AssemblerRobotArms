#include <robot_lib/nav.h>


int main(int argc, char ** argv){
    nav t(argc, argv);
    ros::ServiceServer gotoServ = t.rosNode->advertiseService("/wheely/nav/goto_srv", &nav::goTo, &t); 
    ros::ServiceServer minTourServ = t.rosNode->advertiseService("/wheely/nav/mintour_srv", &nav::GetMinTour, &t); 
    ROS_INFO("Using service name : %s",gotoServ.getService().c_str());
    ROS_INFO("Using service name : %s",minTourServ.getService().c_str());
    ros::spin();
}