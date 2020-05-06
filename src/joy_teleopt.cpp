#include <ros/ros.h>
#include <geometry_msgs/Twist.h> 
#include <sensor_msgs/Joy.h>
#include <math.h>
#include <time.h>

#include "extensa/Command_ABL.h"
#include "extensa/Seg_ABL.h"
#include "extensa/States.h"

float alpha;
float beta;
float length  = 0.35;

float a_scale = 0.001;
float b_scale = 0.001;
float l_scale = 0.0001;

float l_max =  0.50;
float l_min =  0.25;
float a_max =  M_PI/2;
float a_min = -M_PI/2;
float b_max =  M_PI;
float b_min = -M_PI;

float da;
float db;
int dl_u;
int dl_d;
int enable;
int disable;
int last_enable;
int last_disable;
int status;

float State_ABL[3];

void joyCallback(const sensor_msgs::Joy::ConstPtr& joy)
{
	da = joy->axes[1];
	db = joy->axes[3];	
	dl_u = joy->axes[2];
	dl_d = joy->axes[5];
	enable = joy->buttons[7];
	disable = joy->buttons[6];
	
	if (enable == 1 && last_enable == 0)
 {
		status = 1;
 }
	else if (disable == 1 && last_disable == 0)
	{
		status = 0;
	}

	last_enable = enable; 
  last_disable = disable;
}

//subscribe current state
void stateCallback(const extensa::States::ConstPtr& state)
{
	

}


int main(int argc, char **argv)
{
	ros::init(argc, argv, "joy_teleopt");
	ros::NodeHandle nh;	
	ros::Rate r(100);     //Hz

	ros::Subscriber sub1 = nh.subscribe("joy", 1, joyCallback);
	ros::Subscriber sub2 = nh.subscribe("state", 1, stateCallback);	
	ros::Publisher  pub  = nh.advertise<extensa::Seg_ABL>("Cmd_ABL", 10);
	
	while (ros::ok())
	{
		//check whether joystick is available
		if (status == 1)
		{
			if (da > 0)
			{alpha = alpha + a_scale;}
			else if (da < 0)
			{alpha = alpha - a_scale;}
	
			if (db > 0)
			{
				if (db > 0.5)
				{beta = beta + 2*b_scale;}
				else
				{beta = beta + b_scale;}
			}
			else if (db < 0)
			{
				if (db < -0.5)
				{beta = beta - 2*b_scale;}
				else
				{beta = beta - b_scale;}
			}

			if (abs(dl_u-1) > 0.01)
			{
				if (abs(dl_u-1) > 1)
				{length = length + 2*l_scale;}
				else
				{length = length + l_scale;}
			}
			else if (abs(dl_d-1) > 0.01)
			{
				if (abs(dl_d-1) > 1)
				{length = length - 2*l_scale;}
				else
        {length = length - l_scale;}
			}

			if (alpha >= a_max)
			{alpha = a_max;}
			else if (alpha <= a_min)
			{alpha = a_min;}

			if (beta >= b_max)
			{beta = b_max;}
			else if (beta <= b_min)
			{beta = b_min;}

			if (length >= l_max)
			{length = l_max;}
			else if (length <= l_min)
  		{length = l_min;}
			
			ROS_INFO("status: %d", status);
			ROS_INFO("Alpha : %f", alpha);	
  		ROS_INFO("Beta  : %f", beta);
			ROS_INFO("Length: %f", length);	
			
		}						
		else
		{
			ROS_INFO("status: %d", status);
			ROS_INFO("Alpha : %f", alpha);	
  		ROS_INFO("Beta  : %f", beta);
			ROS_INFO("Length: %f", length);	
		}
		
		extensa::Seg_ABL Cmd_ABL;
		Cmd_ABL.A = alpha;
		Cmd_ABL.B = beta;
		Cmd_ABL.L = length;

		pub.publish(Cmd_ABL);

		ros::spinOnce();
		r.sleep();    //sleep for 1/r sec
		//usleep(10000); // N*us
	}
	return 0;
}


