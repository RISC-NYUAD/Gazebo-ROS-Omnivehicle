#include <gazebo/common/Plugin.hh>
#include <gazebo/gazebo.hh>
#include <gazebo/physics/physics.hh>
#include <gazebo/common/common.hh>
#include <ros/ros.h>
#include <nav_msgs/Odometry.h>
#include <geometry_msgs/Point.h>
#include <geometry_msgs/Quaternion.h>
#include <geometry_msgs/Vector3.h>
#include <geometry_msgs/PoseWithCovariance.h>
#include <geometry_msgs/TwistWithCovariance.h>
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
class OmniVehiclePlugin : public ModelPlugin
{

public: ros::NodeHandle nh;
public: std::string namespc;
public: ros::Publisher  vehicle_state_pub ;
public: ros::Subscriber keysub ;
public: double time;
public: double prev_time, prev_error_x, prev_error_y ;
public: uint32_t sequence;
public: double xdes, ydes, thetades ;
private: physics::ModelPtr model;
private: event::ConnectionPtr updateConnection;

public: void Load(physics::ModelPtr _parent, sdf::ElementPtr _sdf)
	{
    	this->model = _parent;
		if(_sdf->HasElement("nameSpace"))
			namespc  = _sdf->GetElement("nameSpace")->Get<std::string>();
			
		sequence = 0;
		prev_time = 0;
		prev_error_x = 0;
		prev_error_y = 0;
		xdes = 0;
		ydes = 0;
		thetades = 0;
			
		std::string topicName = namespc + "/vehicle_pose" ;

		keysub = nh.subscribe("/Omni_wheel/teleoperator", 1, &OmniVehiclePlugin::keyper_callback, this);
		vehicle_state_pub = nh.advertise<nav_msgs::Odometry>(topicName, 1); 
	
    	this->updateConnection = event::Events::ConnectWorldUpdateBegin(std::bind(&OmniVehiclePlugin::onUpdate, this));
		
	}  

public:	void onUpdate()
	{
		
		// Get simulation time and initialize sensor message header //
		sequence++;
		ros::Time curr_time = ros::Time::now();
		
		std_msgs::Header header;	
		header.seq = sequence;
		header.frame_id = " " ;
		header.stamp = curr_time ;
		
		nav_msgs::Odometry msg;
		msg.header = header;
		msg.child_frame_id = " " ;
			
		
		// Get pointers to relevant joints and iterate //
		physics::LinkPtr vehicle;
		physics::JointPtr back, left, right;
		std::string vehicle_name;
		vehicle_name = namespc + "::" + "origin_link" ;

		vehicle = this->model->GetLink(vehicle_name);
		back = this->model->GetJoint(namespc + "::" + "rim_back_joint");
		left = this->model->GetJoint(namespc + "::" + "rim_left_joint");
		right = this->model->GetJoint(namespc + "::" + "rim_right_joint");
		
		gazebo::math::Pose pose = vehicle->GetWorldCoGPose() ;
		gazebo::math::Vector3 lin_vel = vehicle->GetWorldLinearVel() ;					
		gazebo::math::Vector3 position = pose.pos ;
		gazebo::math::Quaternion quatern = pose.rot ;		
		gazebo::math::Vector3 ang_vel = vehicle->GetRelativeAngularVel();
		
		geometry_msgs::Point point;
		point.x = position.x;	point.y = position.y;	point.z = position.z;		
		geometry_msgs::Quaternion quaternion;
		quaternion.x = quatern.x;	quaternion.y = quatern.y;	quaternion.z = quatern.z;	quaternion.w = quatern.w;	
		
		tf::Quaternion q(quaternion.x, quaternion.y, quaternion.z, quaternion.w);
		tf::Matrix3x3 R(q);
		double roll, pitch, yaw ;
		R.getRPY(roll, pitch, yaw) ; 

		
		geometry_msgs::Vector3 linear, angular;
		linear.x = lin_vel.x;	linear.y = lin_vel.y;	linear.z = lin_vel.z;
		angular.x = ang_vel.x;	angular.y = ang_vel.y;	angular.z = ang_vel.z;	
		
		msg.pose.pose.position = point ;
		msg.pose.pose.orientation = quaternion ;
		msg.twist.twist.linear = linear ;
		msg.twist.twist.angular = angular ;
		
		vehicle_state_pub.publish(msg);

		
		Eigen::Vector3d velocities;
		controller(point, yaw, &velocities);
		
		back->SetVelocity(0,velocities(1));
		left->SetVelocity(0,velocities(0));		
		right->SetVelocity(0,velocities(2));

	}

public: void keyper_callback(const std_msgs::Int16& keyper_msg)
	{
		int code = keyper_msg.data;
		switch(code)
		{		
			case 1:
				thetades = thetades + 0.1745;
				//puts("Turning left");
				break;
			case 2:
				thetades = thetades - 0.1745;				
				//puts("Turning right");
				break;
			case 5:
				xdes = xdes + 0.1;
				//puts("Going X forward");
				break;
			case 6:
				xdes = xdes - 0.1;
				//puts("Going X backward");
				break;
			case 7:
				ydes = ydes + 0.1;
				//puts("Going Y forward");
				break;
			case 8:
				ydes = ydes - 0.1;
				//puts("Going Y backward");
				break;
		}
	}


public: void controller(geometry_msgs::Point position, double yaw, Eigen::Vector3d* motor_velocities)
	{
	

		double t_time = ros::Time::now().toSec();
		if(prev_time==0){
			prev_time = t_time - 0.01;
		}
		double dt = t_time - prev_time ;
		//==================================================================================================================//
	
		double x = position.x;	
		double y = position.y;
	
		//  GAINS //
		double Kp_x = 170;   double Kp_y = 170;   double Kp_t = 120;
		double Kd_x = 0.0;   double Kd_y = 0.0;   double Kd_t = 0.0;
		double Ki_x = 70;	 double Ki_y = 70;	  double Ki_t = 80;

		double u_x = - Kp_x * ( x - xdes ) - Ki_x * integral(x-xdes, prev_error_x, dt);
		double u_y = - Kp_y * ( y - ydes ) - Ki_y * integral(y-ydes, prev_error_y, dt);
		double yaw_error = yaw - thetades;
		if(yaw_error > PI){
			yaw_error -= 2*PI;
		}else{
			if(yaw_error < -PI){
				yaw_error += 2*PI;
			}
		}
		double u_t = - Kp_t * yaw_error;

		double L = 0.051*3.0 ;
		double Vx_m = cos(yaw)*u_x + sin(yaw)*u_y ;
		double Vy_m = -sin(yaw)*u_x + cos(yaw)*u_y ;

		double v_left = - Vx_m/2.0 - sqrt(3.0)*Vy_m/2.0 + L * u_t ;
		double v_back = Vx_m + L * u_t ;
		double v_right = - Vx_m/2.0 + sqrt(3.0)*Vy_m/2.0 + L * u_t ;

		prev_error_x = x - xdes ;
		prev_error_y = y - ydes ;
		prev_time = t_time ;
		
		Eigen::Vector3d vels;
		vels << v_left, v_back, v_right;
		*motor_velocities = vels ;
		
	}  

public: double integral(double error, double prev_error, double dt)
	{
		double result = dt * (error + prev_error)/2.0 ;
		return result ;
	}



};
GZ_REGISTER_MODEL_PLUGIN(OmniVehiclePlugin)
}
