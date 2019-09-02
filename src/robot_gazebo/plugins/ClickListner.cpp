#include <functional>
#include <gazebo/gui/GuiIface.hh>
#include <gazebo/rendering/rendering.hh>
#include <gazebo/gazebo.hh>
#include <gazebo/gui/MouseEventHandler.hh>
#include <gazebo/gui/KeyEventHandler.hh>
#include <gazebo/common/MouseEvent.hh>
#include <gazebo/common/KeyEvent.hh>
#include <ros/ros.h>
#include <geometry_msgs/Point.h>
#include <robot_lib/GoTo.h>

namespace gazebo
{
class ClickListner : public SystemPlugin
{
    public: virtual ~ClickListner()
    {
        this->connections.clear();
        if (this->userCam)
        this->userCam->EnableSaveFrame(false);
        this->userCam.reset();
    }
    /////////////////////////////////////////////
    /// \brief Called after the plugin has been constructed.
public: 
    void Load(int /*_argc*/, char ** /*_argv*/)
    {
        printf("\n\n\n\nCLICK LISTNER LOADED!!\n\n\n\n");
        fflush(NULL);
        this->connections.push_back(
                event::Events::ConnectPreRender(
                        boost::bind(&ClickListner::Update, this)));

        gui::MouseEventHandler::Instance()->AddPressFilter("glwidget", boost::bind(&ClickListner::OnMousePress, this, _1));
        gui::KeyEventHandler::Instance()->AddPressFilter("glwidgetd", boost::bind(&ClickListner::OnKey, this, _1));
        if (!ros::isInitialized())
        {
            int argc = 0;
            char **argv = NULL;
            ros::init(argc, argv, "gazebo_client", ros::init_options::NoSigintHandler);
        }
        rosNode.reset(new ros::NodeHandle("clickty"));
        pub = rosNode->advertise<geometry_msgs::Point>("/wheely/clicked_pos",10);
        pubScan = rosNode->advertise<geometry_msgs::Point>("/wheely/slam/BuildMap",10);
    }

    bool OnKey(const common::KeyEvent& _event)
    {
        if ( _event.key ==  16777220){
            geometry_msgs::Point p;
            p.x = 20.26;//20.260273,3.967819
            p.y = 3.967;
            p.z = 0;
            pubScan.publish(p);
        }
        return true;
    }
    bool OnMousePress(const common::MouseEvent& _event)
    {
        mouseClicked = _event.Pos();
        if (!this->userCam)
        {
            // Get a pointer to the active user camera
            this->userCam = gui::get_active_camera();
        }
        // Get scene pointer
        rendering::ScenePtr scene = rendering::get_scene();
        // Wait until the scene is initialized.
        if (!scene || !scene->Initialized())
            return true;

        if(this->userCam){
            ignition::math::Vector3d position_clicked;
            scene->FirstContact(this->userCam, mouseClicked, position_clicked);
            printf("[%f,%f],",position_clicked.X(),position_clicked.Y());
            fflush(NULL);
            geometry_msgs::Point pt ;
            pt.x  = position_clicked.X();
            pt.y = position_clicked.Y();
            pub.publish(pt);
        }
        return true;
    }
private: void Update(){}
private: void Init(){}
private: 
    rendering::UserCameraPtr userCam;
    std::vector<event::ConnectionPtr> connections;
    ros::NodeHandlePtr rosNode;
    ros::Publisher pub,pubScan;
    ros::ServiceClient goto_cl;
    ignition::math::Vector2i mouseClicked;
};
 GZ_REGISTER_SYSTEM_PLUGIN(ClickListner)
}