#include <functional>
#include <gazebo/gazebo.hh>
#include <gazebo/physics/physics.hh>
#include <gazebo/common/common.hh>
#include <ignition/math/Vector3.hh>
#include "stdio.h"

using namespace std;

namespace gazebo
{
class WheelPlugin : public ModelPlugin
{
  public:
	physics::JointControllerPtr jointController;

	void Load(physics::ModelPtr parent, sdf::ElementPtr sdf_ptr)
	{
		std::cout << "Hello world from plugin" << endl;
		this->model = parent;
		printf("Plugin Loaded Model Name : %s\n", this->model->GetName().c_str());
		ignition::math::Vector3d pose = this->model->WorldPose().Pos();
		printf("The position is %f %f %f : \n", pose.X(), pose.Y(), pose.Z());

		// this->front_left_wheel = this->model->GetJoint("axel_wheel_4");
		// this->front_right_wheel = this->model->GetJoint("axel_wheel_1");
		// this->back_right_wheel = this->model->GetJoint("axel_wheel_2");
		// this->back_left_wheel = this->model->GetJoint("axel_wheel_3");

		// this->front_right_steer = this->model->GetJoint("susp1_body");
		// this->front_left_steer = this->model->GetJoint("susp4_body");
		this->back_right_steer = this->model->GetJoint("susp2_body");
		this->back_left_steer = this->model->GetJoint("susp3_body");

		/*Get Parameters from sdf file if they are set*/
		if (sdf_ptr->HasElement("proportional_pid") && sdf_ptr->HasElement("integral_pid") && sdf_ptr->HasElement("derivative_pid") && sdf_ptr->HasElement("velocity"))
		{
		    this->p_pid = atof(sdf_ptr->GetElement("proportional_pid")->GetValue()->GetAsString().c_str());
		    this->d_pid = atof(sdf_ptr->GetElement("derivative_pid")->GetValue()->GetAsString().c_str());
		    this->i_pid = atof(sdf_ptr->GetElement("integral_pid")->GetValue()->GetAsString().c_str());
		    this->velocity = atof(sdf_ptr->GetElement("velocity")->GetValue()->GetAsString().c_str());
		    printf("pid p %f i%f d%f velocity%f ", this->p_pid, this->i_pid, this->d_pid, this->velocity);
		    fflush(NULL);
		}

		this->pid = common::PID(this->p_pid, this->i_pid, this->d_pid);
		this->jointController = this->model->GetJointController();
		this->con = event::Events::ConnectWorldUpdateBegin(std::bind(&WheelPlugin::OnUpdate, this));
	};
	void OnUpdate(){
		this->TurnRight(this->velocity);
	};
	void MoveForward(double vel)
	{

		// std::string name_fr = this->model->GetJoint("axel_wheel_1")->GetScopedName();
		// std::string name_fl = this->model->GetJoint("axel_wheel_4")->GetScopedName();
		std::string name_br = this->model->GetJoint("axel_wheel_2")->GetScopedName();
		std::string name_bl = this->model->GetJoint("axel_wheel_3")->GetScopedName();
		// this->jointController->SetVelocityPID(name_fr, this->pid);
		// this->jointController->SetVelocityPID(name_fl, this->pid);
		this->jointController->SetPositionPID(name_br, this->pid);
		this->jointController->SetPositionPID(name_bl, this->pid);

		// this->jointController->SetVelocityTarget(name_fr, vel);
		// this->jointController->SetVelocityTarget(name_fl, vel);
		this->jointController->SetPositionTarget(name_br, vel);
		this->jointController->SetPositionTarget(name_bl, vel);

		this->jointController->Update();

		//This Works
		// this->front_left_wheel->SetVelocity(0,vel);
		// this->front_right_wheel->SetVelocity(0,vel);
		// this->back_left_wheel->SetVelocity(0,vel);
		// this->back_right_wheel->SetVelocity(0,vel);
	};
	void TurnRight(double angle)
	{
		// std::cout << "Turn" << endl;
		double rad = M_PI * angle / 180;

		std::string name_fr = this->model->GetJoint("susp2_body")->GetScopedName();
		std::string name_fl = this->model->GetJoint("susp3_body")->GetScopedName();

		this->jointController->SetPositionPID(name_fr, this->pid);
		this->jointController->SetPositionPID(name_fl, this->pid);

		this->jointController->SetPositionTarget(name_fr, rad);
		this->jointController->SetPositionTarget(name_fl, rad);
		this->jointController->Update();

		// this->model->GetJoint("susp1_body")->SetPosition(0, rad);
		// this->model->GetJoint("susp4_body")->SetPosition(0, rad);
	};

	void TurnLeft(double angle)
	{
		this->TurnRight(-angle);
	};

  private:
	physics::ModelPtr model;
	event::ConnectionPtr con;
	double p_pid = 2, i_pid = 0, d_pid = 0, velocity = 1;
	double width = 0.3;
	double radius = width / 4;
	common::PID pid;
	physics::JointPtr front_left_wheel;
	physics::JointPtr front_right_wheel;
	physics::JointPtr front_right_steer, front_left_steer;
	physics::JointPtr back_right_steer, back_left_steer;
	physics::JointPtr back_right_wheel;
	physics::JointPtr back_left_wheel;
};
GZ_REGISTER_MODEL_PLUGIN(WheelPlugin)
} // namespace gazebo