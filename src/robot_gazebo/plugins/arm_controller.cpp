#include <functional>
#include <gazebo/gazebo.hh>
#include <gazebo/physics/physics.hh>
#include <gazebo/common/common.hh>

#include <gazebo/transport/transport.hh>
#include <gazebo/msgs/msgs.hh>

#include <ignition/math/Vector3.hh>

#include <thread>
#include "ros/ros.h"
#include "ros/callback_queue.h"
#include "ros/subscribe_options.h"
#include "std_msgs/Float32.h"

#include "geometry_msgs/Pose2D.h"   
#include <robot_lib/ArmAngles.h>                       // to get desired position command
#include "turtlesim/Pose.h"

namespace gazebo
{
class ArmController : public ModelPlugin
{
public:
  void Load(physics::ModelPtr _model, sdf::ElementPtr _sdf)
  {
    std::cout << "Plugin started" << std::endl;

    // Safety check
    if (_model->GetJointCount() == 0)
    {
      std::cerr << "Model not loaded!\n";
      return;
    }

    // Store the model pointer for convenience.
    this->model = _model;

    // Setup a P-controller, with a gain of 0.1.
    this->pid = common::PID(5, 0, 0);
    this->jointController = this->model->GetJointController();

    // Apply the P-controller to the joint.
    // this->model->GetJointController()->SetVelocityPID(this->armBase_armBaseTop_J->GetScopedName(), 
    //                                                   this->pid);

    // Create the node
    this->node = transport::NodePtr(new transport::Node());
#if GAZEBO_MAJOR_VERSION < 8
    this->node->Init(this->model->GetWorld()->GetName());
#else
    this->node->Init(this->model->GetWorld()->Name());
#endif

    // Initialize ros, if it has not already bee initialized.
    if (!ros::isInitialized())
    {
      int argc = 0;
      char **argv = NULL;
      ros::init(argc, argv, "gazebo_client", ros::init_options::NoSigintHandler);
    }

    // Create our ROS node. This acts in a similar manner to
    // the Gazebo node
    this->rosNode.reset(new ros::NodeHandle("gazebo_client"));

    
    // Create a named topic, and subscribe to it.
    ros::SubscribeOptions so =
        ros::SubscribeOptions::create<robot_lib::ArmAngles>(
            // "/" + this->model->GetName() + "/joint/angles",
            "/my_car/joint/angles",
            1,
            boost::bind(&ArmController::OnRosJointCmd, this, _1),
            ros::VoidPtr(), &this->rosQueue);

    this->rosSub = this->rosNode->subscribe(so);
    
    // Spin up the queue helper thread.
    this->rosQueueThread = std::thread(std::bind(&ArmController::QueueThread, this));
    // ros::spin();
  }

  // Called by the world update start event
public:
  void OnUpdate()
  {
    if(updateNum < 4000){
      this->jointController->Update();
    } else{
      this->model->GetJoint("armBase_armBaseTop")->SetParam("fmax", 0, 0);
      this->model->GetJoint("armBaseTop_arm1")->SetParam("fmax", 0, 0);
      this->model->GetJoint("arm1_arm2")->SetParam("fmax", 0, 0);
    }

    updateNum++;
    // Apply a small linear velocity to the model.
    // this->model->SetLinearVel(ignition::math::Vector3d(.3, 0, 0));
  }

public:
  void OnRosJointCmd(const robot_lib::ArmAngles::ConstPtr &msg)
  {
    // std::cout << "ArmBase - ArmBaseTop Angle: " << msg->armBase_armBaseTop << std::endl;   
    this->SetAngle("armBase_armBaseTop", msg->armBase_armBaseTop, 0.9, 0, 1.25, false);
    this->SetAngle("armBaseTop_arm1", msg->armBaseTop_arm1, 18, 0, 3.5, false);
    this->SetAngle("arm1_arm2", msg->arm1_arm2, msg->P, msg->I, msg->D, false);
  }

private:
  void SetAngle(std::string joint_name, float degree, float P, float I, float D, bool smallDegree=true)
  {
    if (smallDegree == true && !(degree >= -90 && degree <= 90)){
      return;
    }

    // std::cout << joint_name << std::endl;
    float rad = 3.14 * degree / 180;
    std::string name = this->model->GetJoint(joint_name)->GetScopedName();

    // std::cout << name << std::endl;
    this->jointController->SetPositionPID(name, common::PID(P, I, D));
    this->jointController->SetPositionTarget(name, rad);
    this->jointController->Update();
  }

  /// \brief ROS helper function that processes messages
private:
  void QueueThread()
  {
    static const double timeout = 0.01;
    while (this->rosNode->ok())
    {
      this->rosQueue.callAvailable(ros::WallDuration(timeout));
    }
  }

private:
  int updateNum = 0;
  /// \brief A node used for transport
private:
  transport::NodePtr node;

  /// \brief A subscriber to a named topic.
private:
  transport::SubscriberPtr sub;

  /// \brief Pointer to the model.
private:
  physics::ModelPtr model;

  /// \brief A PID controller for the joint.
private:
  common::PID pid;

private:
  physics::JointControllerPtr jointController;

  // Pointer to the update event connection
private:
  event::ConnectionPtr updateConnection;

  /// \brief A node use for ROS transport
private:
  std::unique_ptr<ros::NodeHandle> rosNode;

  /// \brief A ROS subscriber
private:
  ros::Subscriber rosSub;

  /// \brief A ROS callbackqueue that helps process messages
private:
  ros::CallbackQueue rosQueue;

  /// \brief A thread the keeps running the rosQueue
private:
  std::thread rosQueueThread;
};

// Register this plugin with the simulator
GZ_REGISTER_MODEL_PLUGIN(ArmController)
} // namespace gazebo



























//   /// \brief Pointer to all the joints.
// private:
//   physics::JointPtr armBase_armBaseTop_J;

// private:
//   physics::JointPtr armBaseTop_arm1_J;

// private:
//   physics::JointPtr arm1_arm2_J;

// private:
//   physics::JointPtr arm2_gripper_J;


// Get the first joint. We are making an assumption about the model
    // having one joint that is the rotational joint.
    // this->armBase_armBaseTop_J = this->model->GetJoint("armBase_armBaseTop");
    // this->armBaseTop_arm1_J = this->model->GetJoint("armBaseTop_arm1");
    // this->arm1_arm2_J = this->model->GetJoint("arm1_arm2");
    // this->arm2_gripper_J = this->model->GetJoint("arm2_gripper");



    // ros::SubscribeOptions so1 =
    //     ros::SubscribeOptions::create<std_msgs::Float32>(
    //         topicName,
    //         1,
    //         boost::bind(&ArmController::ArmBase_ArmBaseTop_Cb, this, _1),
    //         ros::VoidPtr(), &this->rosQueue1);

    // ros::SubscribeOptions so2 =
    //     ros::SubscribeOptions::create<std_msgs::Float32>(
    //         topicName1,
    //         1,
    //         boost::bind(&ArmController::ArmBaseTop_Arm1_Cb, this, _1),
    //         ros::VoidPtr(), &this->rosQueue2);

    // ros::SubscribeOptions so3 =
    //     ros::SubscribeOptions::create<std_msgs::Float32>(
    //         "/" + this->model->GetName() + "/" + this->arm1_arm2_J->GetName() + "/force",
    //         1,
    //         boost::bind(&ArmController::Arm1_Arm2_Cb, this, _1),
    //         ros::VoidPtr(), &this->rosQueue3);

    // ros::SubscribeOptions so4 =
    //     ros::SubscribeOptions::create<std_msgs::Float32>(
    //         "/" + this->model->GetName() + "/" + this->arm2_gripper_J->GetName() + "/force",
    //         1,
    //         boost::bind(&ArmController::Arm2_Gripper_Cb, this, _1),
    //         ros::VoidPtr(), &this->rosQueue4);

    // this->rosSub1 = this->rosNode->subscribe(so1);
    // this->rosSub2 = this->rosNode->subscribe(so2);
    // this->rosSub3 = this->rosNode->subscribe(so3);
    // this->rosSub4 = this->rosNode->subscribe(so4);