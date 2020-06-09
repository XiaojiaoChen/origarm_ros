#include <ros/ros.h>
#include <geometry_msgs/Pose.h> 
#include <math.h>
#include <time.h>
#include <fstream>
#include <iostream>

#include "origarm_ros/Command_Position.h"
#include "origarm_ros/Command_ABL.h"
#include "origarm_ros/modenumber.h"
#include "origarm_ros/segnumber.h"
#include "myData.h"

using namespace std;

int ts = 10000;  //time sleep at each step
int repeat = 5;

int mode_in;
float x_in, y_in, z_in;
float a_in[6], b_in[6], l_in[6];

ifstream inFile;

int main(int argc, char **argv)
{
	ros::init(argc, argv, "demo_picknplace");
	ros::NodeHandle nh;	
	ros::Rate r(100);     //Hz
	
	inFile.open("/home/lijing/catkin_ws/src/origarm_ros/savedata.txt", ios::in);

	if (!inFile)
	{
		printf("%s\n", "unable to open the file.");
		exit(1);
	}
	
	if (nh.getParam("ts", ts))
	{
		ROS_INFO("ts is set to %d\r\n", ts);
	}
	else
	{
		ts = 10000;
	}

	if (nh.getParam("repeat", repeat))
	{
		ROS_INFO("repeat is set to %d\r\n", repeat);
	}
	else
	{
		repeat = 10;
	}

	ros::Publisher  pub1 = nh.advertise<origarm_ros::Command_ABL>("Cmd_ABL_joy", 100);
	ros::Publisher  pub2 = nh.advertise<origarm_ros::modenumber>("modenumber", 100);	

	while (ros::ok())
	{
		origarm_ros::modenumber moden;
		origarm_ros::Command_ABL Command_ABL_demo;

		if (!inFile.eof() && repeat > 0)
		{				
			inFile>>mode_in>>x_in>>y_in>>z_in>>a_in[0]>>b_in[0]>>l_in[0]>>a_in[1]>>b_in[1]>>l_in[1]>>a_in[2]>>b_in[2]>>l_in[2]>>a_in[3]>>b_in[3]>>l_in[3]>>a_in[4]>>b_in[4]>>l_in[4]>>a_in[5]>>b_in[5]>>l_in[5];

			moden.modeNumber = 2;

			for (int i = 0; i < 6; i++)
			{
				Command_ABL_demo.segment[i].A = a_in[i];
				Command_ABL_demo.segment[i].B = b_in[i];
				Command_ABL_demo.segment[i].L = l_in[i];
			}

			for (int i = 6; i < 9; i++)
			{
				Command_ABL_demo.segment[i].A = 0;
				Command_ABL_demo.segment[i].B = 0;
				Command_ABL_demo.segment[i].L = 0.055;
			}
			
			pub1.publish(Command_ABL_demo);
			pub2.publish(moden);
			
			printf("Receiving: %d, %f, %f, %f, %f, %f, %f\n", mode_in, x_in, y_in, z_in, a_in[0], b_in[0], l_in[0]);
			usleep(ts);								
		}
		else if (repeat > 0)
		{
			inFile.clear();
			printf("eof:%d\n", inFile.eof());
			inFile.seekg(0, ios::beg);
			repeat = repeat - 1;
		}
		else
		{
			moden.modeNumber = 2;

			for (int i = 0; i < 9; i++)
			{
				Command_ABL_demo.segment[i].A = 0;
				Command_ABL_demo.segment[i].B = 0;
				Command_ABL_demo.segment[i].L = 0.055;
			}
			
			pub1.publish(Command_ABL_demo);
			pub2.publish(moden);
		}

		r.sleep();
	}

	inFile.close();	
	return 0;
}
