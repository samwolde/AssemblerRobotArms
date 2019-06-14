#include <robot_lib/nav.h>


int main(int argc, char ** argv){
    nav t(argc, argv);
    ros::ServiceServer gotoServ = t.rosNode->advertiseService("/wheely/nav/goto_srv", &nav::goTo, &t); 
    ROS_INFO("Using service name : %s",gotoServ.getService().c_str());
    ros::spin();
}