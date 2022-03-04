#include <gazebo/common/Plugin.hh>
#include <gazebo/gazebo.hh>
#include <gazebo/physics/physics.hh>
#include <gazebo/common/common.hh>
#include <ros/ros.h>
#include <std_msgs/Float64MultiArray.h>
#include <std_msgs/Header.h>
#include <std_msgs/Int16.h>
#include <iostream>
#include <string>
#include <time.h>
#include <Eigen/Dense>
#include <tf/tf.h>

#define PI 3.1416

namespace gazebo
{
class OmniCtrlPlugin : public ModelPlugin
{

public: ros::NodeHandle nh;
public: std::string namespc;
public: ros::Subscriber command_sub ;
public: Eigen::Vector3d velocities;
private: physics::ModelPtr model;
private: event::ConnectionPtr updateConnection;

public: void Load(physics::ModelPtr _parent, sdf::ElementPtr _sdf)
	{
    	this->model = _parent;
		if(_sdf->HasElement("nameSpace"))
			namespc  = _sdf->GetElement("nameSpace")->Get<std::string>();

		velocities << 0, 0, 0;	
				
		std::string topicName = namespc + "/joint_motor_controller" ;

		command_sub = nh.subscribe(topicName, 1, &OmniCtrlPlugin::comm_callback, this);
	
    	this->updateConnection = event::Events::ConnectWorldUpdateBegin(std::bind(&OmniCtrlPlugin::onUpdate, this));
		
	}  

public:	void onUpdate()
	{
		
		// Get simulation time and initialize sensor message header //
		physics::JointPtr back, left, right;
		back = this->model->GetJoint(namespc + "::" + "rim_back_joint");
		left = this->model->GetJoint(namespc + "::" + "rim_left_joint");
		right = this->model->GetJoint(namespc + "::" + "rim_right_joint");
				
		back->SetVelocity(0,velocities(2));
		left->SetVelocity(0,velocities(0));		
		right->SetVelocity(0,velocities(1));

	}

public: void comm_callback(const std_msgs::Float64MultiArray& msg)
	{
		velocities << msg.data[0], msg.data[1], msg.data[2];
		return;
	}



};
GZ_REGISTER_MODEL_PLUGIN(OmniCtrlPlugin)
}
