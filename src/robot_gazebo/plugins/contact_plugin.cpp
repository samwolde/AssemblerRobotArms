#include <gazebo/gazebo.hh>
#include <gazebo/physics/physics.hh>
#include <string>
#include <gazebo/gazebo.hh>
#include <gazebo/sensors/sensors.hh>
#include "std_msgs/String.h"
#include "ros/ros.h"
#include <gazebo/common/Plugin.hh>

namespace gazebo
{
    class ContactPlugin : public SensorPlugin
    {
       
        private: sensors::ContactSensorPtr parentSensor;
        private: event::ConnectionPtr updateConnection;
        private: ros::Publisher data_pub;
        private: std::unique_ptr<ros::NodeHandle> rosNode;

        void Load(sensors::SensorPtr _sensor, sdf::ElementPtr /*_sdf*/)
        {
            if (!ros::isInitialized())
            {
                int argc = 0;
                char **argv = NULL;
                ros::init(argc, argv, "gazebo_client", ros::init_options::NoSigintHandler);
                ROS_FATAL_STREAM("Ros is not initialized."
                               << "Load the .. in gazebo_ros");
            }
            else
            {
                ROS_INFO("+++++++++++++++++ContactPlugin++++++++++++++");
            }

            this->rosNode.reset(new ros::NodeHandle("gazebo_client"));
            this->data_pub = this->rosNode->advertise<std_msgs::String>("/robot/gripper/collision", 1000);

            this->parentSensor = std::dynamic_pointer_cast<sensors::ContactSensor>(_sensor);

            if (!this->parentSensor)
            {
                gzerr << "ContactPlugin requires a ContactSensor.\n";
                return;
            }
            this->updateConnection = this->parentSensor->ConnectUpdated( std::bind(&ContactPlugin::OnUpdate, this));
            this->parentSensor->SetActive(true);
        }

        void OnUpdate()
        {
            msgs::Contacts contacts;
            contacts = this->parentSensor->Contacts();

            std_msgs::String msg;
            std::stringstream ss;

            for (unsigned int i = 0; i < contacts.contact_size(); ++i)
            {
                // ss << contacts.contact(i).collision1() << "-" << contacts.contact(i).collision2() << "=";
                ss << contacts.contact(i).collision1() << " ";
                msg.data = ss.str();
                this->data_pub.publish(msg);
            }
        }
    };

    GZ_REGISTER_SENSOR_PLUGIN(ContactPlugin)
}
