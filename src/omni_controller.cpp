#include <iostream>
#include <stdio.h>
#include <cstring>
#include <ros/ros.h>
#include <geometry_msgs/Pose.h>
#include <tf/tf.h>
#include <std_msgs/Float64MultiArray.h>
#include <gazebo/gazebo.hh>
#include <Eigen/Dense>
#include <std_msgs/Int16.h>
#include <std_msgs/Float64.h>
#include <string>

using namespace std;	

double xdes, ydes, thetades ;
ros::Publisher back_wheel, right_wheel, left_wheel, motor_wheels ;
double prev_time, prev_error_x, prev_error_y ;
Eigen::Vector3d curr_state;

//----------------------------------------------------//

void odometry_callback(const geometry_msgs::Pose& );

void keyper_callback(const std_msgs::Int16& );

double myabs(double );

double integral(double , double , double );
//----------------------------------------------------//

int main(int argc, char** argv)
{
	ros::init(argc, argv, "Omni_Control_node");		
	ros::NodeHandle n_h;
	ros::Rate loop_rate(100);

    curr_state << 0, 0, 0;	
	prev_time = 0;
	prev_error_x = 0;
	prev_error_y = 0;


	xdes = stod(argv[1]);
	ydes = stod(argv[2]);
	thetades = stod(argv[3]);
	

	ros::Subscriber keysub = n_h.subscribe("/Omni_wheel/teleoperator", 1, &keyper_callback);

	ros::Subscriber posesub = n_h.subscribe("/Omni_vehicle/vehicle_pose", 1, &odometry_callback);
	
	back_wheel = n_h.advertise<std_msgs::Float64>("/Omni_vehicle/back_controller/command", 1);
	left_wheel = n_h.advertise<std_msgs::Float64>("/Omni_vehicle/left_controller/command", 1);
	right_wheel = n_h.advertise<std_msgs::Float64>("/Omni_vehicle/right_controller/command", 1);

//	motor_wheels = n_h.advertise<std_msgs::Float64MultiArray>("/Omni_vehicle/joint_motor_controller", 1);
		
	while(ros::ok()){
		ros::spinOnce();
		loop_rate.sleep();
	}

	return 0;
}


void keyper_callback(const std_msgs::Int16& keyper_msg)
{
	int code = keyper_msg.data;
	switch(code)
	{		
		case 1:
			thetades = thetades + 0.1745;
			puts("Turning left");
			break;
		case 2:
			thetades = thetades - 0.1745;				
			puts("Turning right");

			break;
		case 5:
			xdes = xdes + 0.1;
			puts("Going X forward");
			break;
		case 6:
			xdes = xdes - 0.1;
			puts("Going X backward");
			break;
		case 7:
			ydes = ydes + 0.1;
			puts("Going Y forward");
			break;
		case 8:
			ydes = ydes - 0.1;
			puts("Going Y backward");
			break;
	}
}


double integral(double error, double prev_error, double dt)
{
	double result = dt * (error + prev_error)/2.0 ;
	return result ;
}


void odometry_callback(const geometry_msgs::Pose& odometry_msg)
{
	Eigen::Vector3d position, velocity ;
	position << odometry_msg.position.x, odometry_msg.position.y, odometry_msg.position.z ;
	
	tf::Quaternion q(odometry_msg.orientation.x, odometry_msg.orientation.y, odometry_msg.orientation.z, odometry_msg.orientation.w);
	tf::Matrix3x3 R(q);
	double roll, pitch, yaw ;
	R.getRPY(roll, pitch, yaw) ; 
	
	double t_time = ros::Time::now().toSec();
	if(prev_time==0){
		prev_time = t_time - 0.01;
	}
	double dt = t_time - prev_time ;
	double x = position(0);	
	double y = position(1);

	double vx = (x - curr_state(0))/dt ;
	double vy = (y - curr_state(1))/dt ;
	double w  = (yaw - curr_state(2))/dt ;
	curr_state << x, y, yaw ;
		
	//  GAINS //
	double Kp_x = 170;   double Kp_y = 170;   double Kp_t = 120;
	double Kd_x = 0.0;   double Kd_y = 0.0;   double Kd_t = 0.0;
	double Ki_x = 70;	 double Ki_y = 70;	  double Ki_t = 80;

	double yaw_error = yaw - thetades ;
	if(yaw_error > M_PI){
		yaw_error -= 2*M_PI;
	}else{
		if(yaw_error < -M_PI){
			yaw_error += 2*M_PI;
		}
	}


	double u_x = - Kp_x * ( x - xdes ) - Ki_x * integral(x-xdes, prev_error_x, dt);
	double u_y = - Kp_y * ( y - ydes ) - Ki_y * integral(y-ydes, prev_error_y, dt);
	double u_t = - Kp_t * yaw_error;
	
	double L = 0.051*3 ;
	double Vx_m = cos(yaw)*u_x + sin(yaw)*u_y ;
	double Vy_m = -sin(yaw)*u_x + cos(yaw)*u_y ;
	
	double v_left = - Vx_m/2.0 - sqrt(3.0)*Vy_m/2.0 + L * u_t ;
	double v_back = Vx_m + L * u_t ;
	double v_right = - Vx_m/2.0 + sqrt(3.0)*Vy_m/2.0 + L * u_t ;

/*	std::cout << "==================================" << std::endl;
	std::cout << v_left << " , " << v_back << " , " << v_right << std::endl;
	std::cout << "==================================" << std::endl;
*/
	
	prev_error_x = x - xdes ;
	prev_error_y = y - ydes ;
	prev_time = t_time ;
	
	std_msgs::Float64 msg;
	msg.data = v_left ;
	left_wheel.publish(msg);
	
	msg.data = v_back ;
	back_wheel.publish(msg);
	
	msg.data = v_right ;
	right_wheel.publish(msg);

/*	std_msgs::Float64MultiArray msg;
	msg.data.push_back(v_left);
	msg.data.push_back(v_right);
	msg.data.push_back(v_back);	
	motor_wheels.publish(msg);	
*/
}


double myabs(double x)
{
	if(x>0){
		return x;
	}else{
		return -1.0*x;
	}
}

