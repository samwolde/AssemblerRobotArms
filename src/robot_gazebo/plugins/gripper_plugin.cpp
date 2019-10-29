#include <boost/bind.hpp>
#include <gazebo/gazebo.hh>
#include <gazebo/physics/physics.hh>
#include <gazebo/physics/physics.hh>
#include <gazebo/common/common.hh>
#include <gazebo/common/Plugin.hh>
#include "ros/ros.h"
#include "ros/callback_queue.h"
#include "ros/subscribe_options.h"
#include <ignition/math/Vector3.hh>
#include <ignition/math/Pose3.hh>
#include <stdio.h>
#include <gazebo/transport/transport.hh>
#include <gazebo/msgs/msgs.hh>
#include "std_msgs/Float32.h"
#include <robot_lib/GripperAngles.h>
#include "std_msgs/String.h"
#include <thread>
#include <cstdlib>
#include <math.h>

namespace gazebo
{
    class GripperPlugin : public ModelPlugin
    {
        private: physics::ModelPtr model;
        private: common::PID pid;
        private: physics::JointControllerPtr jointController;
        private: event::ConnectionPtr updateConnection;
        private: std::unique_ptr<ros::NodeHandle> rosNode;
                
        private: ros::Subscriber    rosSub;
        private: ros::CallbackQueue rosQueue;
        private: std::thread        rosQueueThread;
        
        private:std::thread rosDataPublishThread;
        private:ros::Publisher data_pub;

        public: void Load(physics::ModelPtr _parent, sdf::ElementPtr _sdf)
        {
            this->model = _parent;

            if (!ros::isInitialized())
            {
                int argc = 0;
                char **argv = NULL;
                ros::init(argc, argv, "gazebo_client", ros::init_options::NoSigintHandler);
            }
            else
            {
                ROS_INFO("Starting GripperPlugin");
            }
            std::cout << "Gripper plugin started" << std::endl;
            this->pid = common::PID(1, 0.1, 0.01);
            
            std::string move_fingers   = "/" + this->model->GetName() + "/gripper/move_fingers";

            this->rosNode.reset(new ros::NodeHandle("gazebo_client"));

            this->jointController = this->model->GetJointController();
            this->jointController->Reset();
            this->jointController->AddJoint(model->GetJoint("finger_one_joint"));
            this->jointController->AddJoint(model->GetJoint("finger_two_joint"));

            // default angles
            this->SetAngle("finger_one_joint",      30);
            this->SetAngle("finger_two_joint",      -30);

            ros::SubscribeOptions so =
                ros::SubscribeOptions::create<robot_lib::GripperAngles>
                (
                    move_fingers,
                    1,
                    boost::bind( &GripperPlugin::set_angle, this, _1), 
                    ros::VoidPtr(), 
                    &this->rosQueue
                );

            this->rosSub = this->rosNode->subscribe(so);
            this->rosQueueThread = std::thread(std::bind(&GripperPlugin::QueueThread, this));
            
            ROS_WARN("Loaded Plugin with parent...%s", this->model->GetName().c_str());
        }

        private: void QueueThread()
        {
            static const double timeout = 0.01;
            while (this->rosNode->ok())
            {
                this->rosQueue.callAvailable(ros::WallDuration(timeout));
            }
        }

        public: void set_angle(const robot_lib::GripperAngles::ConstPtr &_msg)
        {
            this->SetAngle("finger_one_joint",      _msg->palm_finger1);
            this->SetAngle("finger_two_joint",      _msg->palm_finger2);
        }

        private: void SetAngle(std::string joint_name, float degree)
        {
            float rad = M_PI * degree / 180;
            std::string name = this->model->GetJoint(joint_name)->GetScopedName();
            this->jointController->SetPositionPID(name, pid);
            this->jointController->SetPositionTarget(name, rad);
            this->jointController->Update();
        }
    };

    GZ_REGISTER_MODEL_PLUGIN(GripperPlugin);
}