#include <ros/ros.h>
#include <geometry_msgs/Twist.h> 
#include <geometry_msgs/Pose.h> 
#include <sensor_msgs/Joy.h>
#include <math.h>
#include <time.h>
#include <iostream>
#include <vector>

#include "origarm_ros/Command_ABL.h"
#include "origarm_ros/Seg_ABL.h"
#include "origarm_ros/SegOpening.h"
#include "origarm_ros/Command_Position.h"

using namespace std;

//joystick mapping
float joyLx;
float joyLy;
float joyRx;
float joyRy;
float joyLT;
float joyRT;
int joyLB;
int joyRB;

int enable;  
int disable; 
int last_enable;
int last_disable;
int status;

//Write ABL
float alpha;
float beta;
float length  = 0.055;

float a_scale = 0.001;
float b_scale = 0.001;
float l_scale = 0.00001;

float l_max =  0.08;
float l_min =  0.03;
float a_max =  M_PI/2;
float a_min = -M_PI/2;
float b_max =  M_PI;
float b_min = -M_PI;

//write Opening
float belloConfigurationR = 0.06;
float Rmax = 1.0;

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

//Write XYZ unit:m
float x = 0;
float y = 0;
float z = 0.055;

float x_scale = 0.00001;
float y_scale = 0.00001;
float z_scale = 0.0001;

float x_max =  0.01;
float x_min = -0.01;
float y_max =  0.01;
float y_min = -0.01;
float z_max =  0.08;
float z_min =  0.03;

//joystick callback
void joyCallback(const sensor_msgs::Joy::ConstPtr& joy)
{
	//joystick mapping
	joyLx = joy->axes[0];
	joyLy = joy->axes[1];
	joyRx = joy->axes[3];
	joyRy = joy->axes[4];
	joyLT = joy->axes[2];
	joyRT = joy->axes[5];	
	joyLB = joy->buttons[4];
	joyRB = joy->buttons[5]; 
 //enable = joy->buttons[0];
	disable = joy->buttons[6];//back

//only when joyRT && joyLT pressed together, joystick starts to control
	if (joyLT == -1 && joyRT == -1)
	{
		enable = 1;
	}
	else
	{
		enable = 0;
	}
			
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

//Joystick->ABL
void WriteABL()
{
			if (joyLy > 0.05)
			{alpha = alpha + a_scale;}
			else if (joyLy < -0.05)
			{alpha = alpha - a_scale;}
	
			if (joyRx > 0.05)
			{
				if (joyRx > 0.5)
				{beta = beta + 2*b_scale;}
				else
				{beta = beta + b_scale;}
			}
			else if (joyRx < -0.05)
			{
				if (joyRx < -0.5)
				{beta = beta - 2*b_scale;}
				else
				{beta = beta - b_scale;}
			}

		if (joyLT != 1 && joyRT != 1)
		{
			
		}
		else
		{
			if (abs(joyLT-1) > 0.05)
			{
				if (abs(joyLT-1) > 1)
				{length = length + 2*l_scale;}
				else
				{length = length + l_scale;}
			}
			else if (abs(joyRT-1) > 0.05)
			{
				if (abs(joyRT-1) > 1)
				{length = length - 2*l_scale;}
				else
        {length = length - l_scale;}
			}
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

//Joystick->XYZ (joyRx->x, joyRy->y, joyLy->z)
void WriteXYZ()
{
	if (joyRx > 0.05)
	{
		x = x + x_scale;
	}
	else if (joyRx < -0.05)
	{
		x = x - x_scale;
	}

	if (joyRy > 0.05)
	{
		y = y + y_scale;
	}			
	else if (joyRy < -0.05)
	{
		y = y - y_scale;
	}
		
	if (joyLy > 0.05)
	{
		z = z + z_scale;
	}			
	else if (joyLy < -0.05)
	{
		z = z - z_scale;
	}

	if (x >= x_max)
	{x = x_max;}
	else if (x <= x_min)
	{x = x_min;}

	if (y >= y_max)
	{y = y_max;}
	else if (y <= y_min)
	{y = y_min;}

	if (z >= z_max)
	{z = z_max;}
	else if (z <= z_min)
  {z = z_min;}		
}

int main(int argc, char **argv)
{
	ros::init(argc, argv, "joy_teleopt");
	ros::NodeHandle nh;	
	ros::Rate r(100);     //Hz

	ros::Subscriber sub1 = nh.subscribe("joy", 1, joyCallback);	
	ros::Publisher  pub1  = nh.advertise<origarm_ros::Command_ABL>("Cmd_ABL", 100);
	ros::Publisher  pub2  = nh.advertise<origarm_ros::SegOpening>("Cmd_Opening", 100);
	//ros::Publisher  pub3  = nh.advertise<geometry_msgs::Pose>("Cmd_XYZ", 100);
	ros::Publisher  pub3  = nh.advertise<origarm_ros::Command_Position>("Command_Position", 100);

	
	Init_parameter();

	while (ros::ok())
	{
		//check whether joystick is available
		if (status == 1)
		{
			WriteABL();		
			ROS_INFO("status: %d", status);
			/*ROS_INFO("Alpha : %f", alpha);	
  		ROS_INFO("Beta  : %f", beta);
			ROS_INFO("Length: %f", length);*/			
			
			WriteOpening();
      //ROS_INFO("Lx: %f, Ly: %f, Rx: %f, Ry: %f",joyLx,joyLy,joyRx,joyRx);
      //ROS_INFO("OpeningResult[0]: %f,[1]: %f,[2]: %f,[3]: %f,[4]: %f,[5]: %f",OpeningResult[0],OpeningResult[1],OpeningResult[2],OpeningResult[3],OpeningResult[4],OpeningResult[5]);

			WriteXYZ();
			/*ROS_INFO("x : %f", x);	
  		ROS_INFO("y : %f", y);
			ROS_INFO("z : %f", z);*/				
		}						
		else
		{
			ROS_INFO("status: %d", status);
			/*ROS_INFO("Alpha : %f", alpha);	
  		ROS_INFO("Beta  : %f", beta);
			ROS_INFO("Length: %f", length);*/

			//ROS_INFO("Lx: %f, Ly: %f, Rx: %f, Ry: %f",joyLx,joyLy,joyRx,joyRx);
      //ROS_INFO("OpeningResult[0]: %f,[1]: %f,[2]: %f,[3]: %f,[4]: %f,[5]: %f",OpeningResult[0],OpeningResult[1],OpeningResult[2],OpeningResult[3],OpeningResult[4],OpeningResult[5]);

			/*ROS_INFO("x : %f", x);	
  		ROS_INFO("y : %f", y);
			ROS_INFO("z : %f", z);*/
	
		}
		
		origarm_ros::Command_ABL Cmd_ABL;
		Cmd_ABL.segment[0].A = alpha;
		Cmd_ABL.segment[0].B = beta;
		Cmd_ABL.segment[0].L = length;
				
		origarm_ros::SegOpening Cmd_Opening;
		for (int i = 0; i < 6; i++)
		{
			Cmd_Opening.Op[i] = OpeningResult[i];
		}
		
		/*geometry_msgs::Pose Cmd_XYZ;
		Cmd_XYZ.position.x = x;
		Cmd_XYZ.position.y = y;
		Cmd_XYZ.position.z = z;*/

		origarm_ros::Command_Position Command_Position;
		Command_Position.pose.position.x = x;
		Command_Position.pose.position.y = y;
		Command_Position.pose.position.z = z;
		Command_Position.pose.orientation.x = 1;
		Command_Position.pose.orientation.w = 1;

		pub1.publish(Cmd_ABL);
		pub2.publish(Cmd_Opening);
		pub3.publish(Command_Position);

		ros::spinOnce();
		r.sleep();    //sleep for 1/r sec
		//usleep(10000); // N*us
	}
	return 0;
}


