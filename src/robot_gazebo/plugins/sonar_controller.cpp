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
class SonarController : public ModelPlugin
{
public:
  void Load(physics::ModelPtr _model, sdf::ElementPtr _sdf)
  {
    std::cout << "Sonar Plugin started" << std::endl;

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

    this->SetPid("sonarBase_sonarMount", 1, 0, 1);

    // default sonar position
    // this->SetAngle("body_sonarMount", 0);
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
        ros::SubscribeOptions::create<std_msgs::Float32>(
            // "/wheely/arm/angles_cmd",
            // "/" + this->model->GetName() + "/sonar/angle_cmd",
            "/robot/sonar/angle_cmd",
            1,
            boost::bind(&SonarController::OnSonarCmd, this, _1),
            ros::VoidPtr(), &this->rosQueue);

    this->rosSub = this->rosNode->subscribe(so);
    
    // Spin up the queue helper thread.
    this->rosQueueThread = std::thread(std::bind(&SonarController::QueueThread, this));
  }

  // Called by the world update start event
public:
  void OnUpdate()
  {
    if(updateNum < 4000){
      this->jointController->Update();
    } else{
      this->model->GetJoint("sonarBase_sonarMount")->SetParam("fmax", 0, 0);
    }

    updateNum++;
  }

public:
  void OnSonarCmd(const std_msgs::Float32::ConstPtr &msg)
  {
    this->SetAngle("sonarBase_sonarMount", msg->data);
  }

private:
  void SetAngle(std::string joint_name, float degree)
  {
    float rad = 3.14 * degree / 180;
    std::string name = this->model->GetJoint(joint_name)->GetScopedName();

    this->jointController->SetPositionTarget(name, rad);
    this->jointController->Update();
    
    updateNum = 0;
  }

private:
  void SetVelocity(std::string joint_name, float velocity)
  {
    this->model->GetJoint(joint_name)->SetVelocity(0, velocity);    
  }

private:
  void SetPid(std::string jointName, float P, float I, float D){
    std::string name = this->model->GetJoint(jointName)->GetScopedName();
    this->jointController->SetPositionPID(name, common::PID(P, I, D));
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
GZ_REGISTER_MODEL_PLUGIN(SonarController)
} // namespace gazebo
