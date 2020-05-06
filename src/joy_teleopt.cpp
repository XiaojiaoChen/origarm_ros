#include <ros/ros.h>
#include <geometry_msgs/Twist.h> 
#include <sensor_msgs/Joy.h>
#include <math.h>
#include <time.h>
#include <iostream>
#include <vector>

#include "extensa/Command_ABL.h"
#include "extensa/Seg_ABL.h"
#include "extensa/States.h"
#include "extensa/Seg.h"
#include "extensa/SegOpening.h"

using namespace std;

//Write ABL
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

//write Opening
float belloConfigurationR = 0.06;
float Rmax = 1.0;
float joyLx;
float joyLy;
float joyRx;
float joyRy;
float rawAngle;
float rawAmplitude;
float rawAmplitudeMax;
float angleCommand;
float amplitudeCommand;
float bellowProjection[6];
float bellowConfigurationPx[6];
float bellowConfigurationPy[6];
float openingBase;
float OpeningResult[6];

float State_ABL[3];

void joyCallback(const sensor_msgs::Joy::ConstPtr& joy)
{
	//ABL mapping
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

	//writeOpening mapping
	joyLx = joy->axes[0];
	joyLy = joy->axes[1];
	joyRx = joy->axes[3];
	joyRy = joy->axes[4];

	//WriteXYZ mapping
		
}

//Joystick->ABL
void WriteABL()
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
		
}

void	Init_parameter()
{
	for (int i = 0; i < 6; i++)
	{
		bellowConfigurationPx[i] = belloConfigurationR*cos(i*2*M_PI/6);
		bellowConfigurationPy[i] = belloConfigurationR*sin(i*2*M_PI/6);
	}
}

//Joystick->Opening
void WriteOpening()
{
	//openingBase for elongation
	if (joyLy > 0)
	{
		openingBase = 0.5;
	}
	else if (joyLy < 0)
  {
		openingBase = -0.8;
	}
	else
	{
		openingBase = 0;
	}
	
	//opening for rotation
	if (joyRx == 0 && joyRy == 0)
	{
		rawAngle = 0;
		rawAmplitude = 0;
		rawAmplitudeMax = Rmax;
	}
	else
	{
		rawAngle = atan2(joyRx, joyRy);
		rawAmplitude = sqrt(joyRx*joyRx+joyRy*joyRy);
	
		if (rawAmplitude > 0)
		{
			if (rawAngle < M_PI*0.25 || rawAngle > M_PI*0.75)
			{
				rawAmplitudeMax = Rmax/cos(rawAngle);
			}
			else
			{
				rawAmplitudeMax = Rmax/sin(rawAngle);
			}
		}
		else
		{
			if (rawAngle > -M_PI*0.25 || rawAngle < -M_PI*0.75)
			{
				rawAmplitudeMax = Rmax/cos(rawAngle+M_PI);
			}
			else
			{
				rawAmplitudeMax = Rmax/sin(rawAngle+M_PI);
			} //if (rawAngle > -M_PI*0.25 || rawAngle < -M_PI*0.75)else						
		} //if (rawAmplitude>0)else			
	} //if (joyRx == 0&&...)else
	
	angleCommand = rawAngle;
	amplitudeCommand = abs(rawAmplitude/rawAmplitudeMax);
	
	for (int i = 0; i < 6; i++)
	{
		bellowProjection[i] = cos(angleCommand)*bellowConfigurationPx[i] + sin(angleCommand)*bellowConfigurationPy[i];
		OpeningResult[i] = -(bellowProjection[i]/belloConfigurationR*amplitudeCommand*0.5)+openingBase;
	}

} //void 


int main(int argc, char **argv)
{
	ros::init(argc, argv, "joy_teleopt");
	ros::NodeHandle nh;	
	ros::Rate r(100);     //Hz

	ros::Subscriber sub1 = nh.subscribe("joy", 1, joyCallback);	
	ros::Publisher  pub1  = nh.advertise<extensa::Seg_ABL>("Cmd_ABL", 10);
	ros::Publisher  pub2  = nh.advertise<extensa::SegOpening>("Cmd_Opening", 100);
	
	Init_parameter();

	while (ros::ok())
	{
		//check whether joystick is available
		if (status == 1)
		{
			WriteABL();

			ROS_INFO("status: %d", status);
			ROS_INFO("Alpha : %f", alpha);	
  		ROS_INFO("Beta  : %f", beta);
			ROS_INFO("Length: %f", length);				
			
			//WriteOpening();
      //ROS_INFO("Lx: %f, Ly: %f, Rx: %f, Ry: %f",joyLx,joyLy,joyRx,joyRx);
      //ROS_INFO("OpeningResult[0]: %f,[1]: %f,[2]: %f,[3]: %f,[4]: %f,[5]: %f",OpeningResult[0],OpeningResult[1],OpeningResult[2],OpeningResult[3],OpeningResult[4],OpeningResult[5]);
		}						
		else
		{
			ROS_INFO("status: %d", status);
			ROS_INFO("Alpha : %f", alpha);	
  		ROS_INFO("Beta  : %f", beta);
			ROS_INFO("Length: %f", length);	
			//ROS_INFO("Lx: %f, Ly: %f, Rx: %f, Ry: %f",joyLx,joyLy,joyRx,joyRx);
      //ROS_INFO("OpeningResult[0]: %f,[1]: %f,[2]: %f,[3]: %f,[4]: %f,[5]: %f",OpeningResult[0],OpeningResult[1],OpeningResult[2],OpeningResult[3],OpeningResult[4],OpeningResult[5]);	
		}
		
		extensa::Seg_ABL Cmd_ABL;
		Cmd_ABL.A = alpha;
		Cmd_ABL.B = beta;
		Cmd_ABL.L = length;
				
		extensa::SegOpening Cmd_Opening;
		for (int i = 0; i < 6; i++)
		{
			Cmd_Opening.Op[i] = OpeningResult[i];
		}
		
		pub1.publish(Cmd_ABL);
		pub2.publish(Cmd_Opening);

		ros::spinOnce();
		r.sleep();    //sleep for 1/r sec
		//usleep(10000); // N*us
	}
	return 0;
}


