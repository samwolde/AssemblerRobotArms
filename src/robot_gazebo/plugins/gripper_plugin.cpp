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
        
        private:ros::Publisher data_pub;
        private:int updateNum = 0;

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

            this->pid = common::PID(0, 0, 0);
            
            std::string move_fingers   = "/" + this->model->GetName() + "/gripper/move_fingers";
            
            this->rosNode.reset(new ros::NodeHandle("gazebo_client"));

            this->jointController = this->model->GetJointController();
            this->jointController->Reset();
            this->jointController->AddJoint(model->GetJoint("finger_one_joint"));
            this->jointController->AddJoint(model->GetJoint("finger_two_joint"));
            this->jointController->AddJoint(model->GetJoint("finger_three_joint"));
            this->jointController->AddJoint(model->GetJoint("finger_four_joint"));
            this->jointController->AddJoint(model->GetJoint("finger_one_tip_joint"));
            this->jointController->AddJoint(model->GetJoint("finger_two_tip_joint"));
            this->jointController->AddJoint(model->GetJoint("finger_three_tip_joint"));
            this->jointController->AddJoint(model->GetJoint("finger_four_tip_joint"));

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
            this->SetAngle("finger_three_joint",    _msg->palm_finger3);
            this->SetAngle("finger_four_joint",     _msg->palm_finger4);
            this->SetAngle("finger_one_tip_joint",  _msg->finger1_tip);
            this->SetAngle("finger_two_tip_joint",  _msg->finger2_tip);
            this->SetAngle("finger_three_tip_joint",_msg->finger3_tip);
            this->SetAngle("finger_four_tip_joint", _msg->finger4_tip);
        }

        private: void SetAngle(std::string joint_name, float degree)
        {
            if (degree < -45 && degree > 45)
            {
                ROS_INFO("GripperPlugin: angle must be less than 45 degree");
                return;
            }

            float rad = M_PI * degree / 180;
            std::string name = this->model->GetJoint(joint_name)->GetScopedName();
            this->jointController->SetPositionPID(name, pid);
            this->jointController->SetPositionTarget(name, rad);
            this->jointController->Update();

            std::cout << joint_name + "  |  " << rad << std::endl;   

        }

        public: void OnUpdate()
        {
            if(updateNum < 4000)
            {
              this->jointController->Update();
            } 
            else
            {
                this->model->GetJoint("finger_one_joint")->SetParam("fmax", 0, 0);
                this->model->GetJoint("finger_two_joint")->SetParam("fmax", 0, 0);
                this->model->GetJoint("finger_three_joint")->SetParam("fmax", 0, 0);
                this->model->GetJoint("finger_four_joint")->SetParam("fmax", 0, 0);
                this->model->GetJoint("finger_one_tip_joint")->SetParam("fmax", 0, 0);
                this->model->GetJoint("finger_two_tip_joint")->SetParam("fmax", 0, 0);
                this->model->GetJoint("finger_three_tip_joint")->SetParam("fmax", 0, 0);
                this->model->GetJoint("finger_four_tip_joint")->SetParam("fmax", 0, 0);
            }

            updateNum++;
          }
    };


    GZ_REGISTER_MODEL_PLUGIN(GripperPlugin);
}