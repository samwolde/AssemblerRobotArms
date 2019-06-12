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
#include <robot_lib/Sensor.h>
#include "turtlesim/Pose.h"

namespace gazebo
{
class IRSensorCtrl : public ModelPlugin
{
public:
  void Load(physics::ModelPtr _model, sdf::ElementPtr _sdf)
  {
    ROS_INFO("Sensor Turn Service started" );
    this->model = _model;
    this->jointController = this->model->GetJointController();

    this->SetPid("IR_body_joint", 5, 1, 0.005);

    // default sensor position
    this->SetAngle("IR_body_joint", 0);
    // Initialize ros, if it has not already bee initialized.
    if (!ros::isInitialized())
    {
      int argc = 0;
      char **argv = NULL;
      ros::init(argc, argv, "gazebo_client", ros::init_options::NoSigintHandler);
    }
    this->rosNode.reset(new ros::NodeHandle("gazebo_client"));
    turnServer = this->rosNode->advertiseService("/wheely/ir_sensor/cmd_turn",&IRSensorCtrl::turnCmd, this);
    turnPub = this->rosNode->advertise<std_msgs::Float32>("/wheely/ir_sensor/state",10);
    pubThread = std::thread(std::bind(&IRSensorCtrl::publishState,this));
  }
  void publishState(){
    std_msgs::Float32 f;
    auto j = this->model->GetJoint("IR_body_joint");
    ros::Rate r(500);
    while(true){
      f.data = j->Position();;
      turnPub.publish(f);
      r.sleep();
    }
  }
  bool turnCmd(robot_lib::Sensor::Request &req, robot_lib::Sensor::Response &res)
  {
    this->SetAngle("IR_body_joint", req.angle);
    return res.turned = true;
  }

  void SetAngle(std::string joint_name, double degree)
  {
    double rad = M_PI * degree / 180;
    std::string name = this->model->GetJoint(joint_name)->GetScopedName();
    this->jointController->SetPositionTarget(name, rad);
    this->jointController->Update();
    auto j = this->model->GetJoint(joint_name);
    while( fabs( j->Position() - rad) > 0.01){
        //Wait till actually has turned
    }
  }
  void SetPid(std::string jointName, float P, float I, float D){
    std::string name = this->model->GetJoint(jointName)->GetScopedName();
    this->jointController->SetPositionPID(name, common::PID(P, I, D));
  }
  
  private:
    physics::ModelPtr model;
    physics::JointControllerPtr jointController;
    event::ConnectionPtr updateConnection;
    ros::Subscriber rosSub;
    ros::CallbackQueue rosQueue;
    std::thread pubThread;
    std::thread rosQueueThread;
    std::unique_ptr<ros::NodeHandle> rosNode;
    ros::ServiceServer turnServer;
    ros::Publisher turnPub;
};
GZ_REGISTER_MODEL_PLUGIN(IRSensorCtrl)
}