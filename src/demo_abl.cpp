#include <ros/ros.h>
#include <geometry_msgs/Pose.h> 
#include <math.h>
#include <time.h>

#include "origarm_ros/Command_ABL.h"
#include "origarm_ros/modenumber.h"
#include "origarm_ros/segnumber.h"
#include "myData.h"

int tp = 1000;  //timestep
int ts = 10000; //time sleep at each step
int flag = 1;

int mode_ = 0;
int segment_ = 0;

float demo_a;
float demo_b;
float demo_l;

int main(int argc, char **argv)
{
	ros::init(argc, argv, "demo_abl");
	ros::NodeHandle nh;	
	ros::Rate r(100);     //Hz

	ros::Publisher  pub1 = nh.advertise<origarm_ros::Command_ABL>("Cmd_ABL_joy", 100);
	ros::Publisher  pub2 = nh.advertise<origarm_ros::modenumber>("modenumber", 100);
	ros::Publisher  pub3 = nh.advertise<origarm_ros::segnumber>("segnumber", 100);		

	while (ros::ok())
	{
		origarm_ros::Command_ABL Command_ABL_demo;
		origarm_ros::modenumber moden;
		origarm_ros::segnumber segn;

		moden.modeNumber = mode_;
		segn.segmentNumber = segment_;

		pub2.publish(moden);
		pub3.publish(segn);

		if (flag == 1)
		{	
			for (int i = 0; i < tp; i++)
 			{
 				demo_a = i*M_PI*0.5/(tp-1);
 				demo_b = 0;
 				demo_l = length0*9;

 				for (int i = 0; i < seg; i++)
 				{
 					Command_ABL_demo.segment[i].A = demo_a/9;
 					Command_ABL_demo.segment[i].B = demo_b;
 					Command_ABL_demo.segment[i].L = demo_l/9;
 				}
 			
 				pub1.publish(Command_ABL_demo);
 				printf("A: %f, B: %f, L: %f\r\n", demo_a, demo_b, demo_l);
 			
 				usleep(ts);
 			}

 			for (int i = 0; i < tp; i++)
 			{
 				demo_a = M_PI*0.5;
 				demo_b = i*2*M_PI/(tp-1);
 				demo_l = length0*9;

 				for (int i = 0; i < seg; i++)
 				{
 					Command_ABL_demo.segment[i].A = demo_a/9;
 					Command_ABL_demo.segment[i].B = demo_b;
 					Command_ABL_demo.segment[i].L = demo_l/9;
 				}

 				pub1.publish(Command_ABL_demo);
 				printf("A: %f, B: %f, L: %f\r\n", demo_a, demo_b, demo_l);
 			
 				usleep(ts);
 			}

 			for (int i = 0; i < tp; i++)
 			{
 				demo_a = M_PI*0.5;
 				demo_b = 0;
 				demo_l = length0*9 + i*(0.08-length0)*9/(tp-1);

 				for (int i = 0; i < seg; i++)
 				{
 					Command_ABL_demo.segment[i].A = demo_a/9;
 					Command_ABL_demo.segment[i].B = demo_b;
 					Command_ABL_demo.segment[i].L = demo_l/9;
 				}

 				pub1.publish(Command_ABL_demo);
 				printf("A: %f, B: %f, L: %f\r\n", demo_a, demo_b, demo_l);
 			
 				usleep(ts); 			
 			}

 			for (int i = 0; i < tp; i++)
 			{
 				demo_a = M_PI*0.5;
 				demo_b = 0;
 				demo_l = 0.08*9 - i*(0.08-length0)*9/(tp-1);

 				for (int i = 0; i < seg; i++)
 				{
 					Command_ABL_demo.segment[i].A = demo_a/9;
 					Command_ABL_demo.segment[i].B = demo_b;
 					Command_ABL_demo.segment[i].L = demo_l/9;
 				}

 				pub1.publish(Command_ABL_demo);
 				printf("A: %f, B: %f, L: %f\r\n", demo_a, demo_b, demo_l);
 			
 				usleep(ts); 			
 			}

 			for (int i = 0; i < tp; i++)
 			{
 				demo_a = M_PI*0.5 - i*M_PI*0.5/(tp-1);
 				demo_b = 0;
 				demo_l = length0*9;

 				for (int i = 0; i < seg; i++)
 				{
 					Command_ABL_demo.segment[i].A = demo_a/9;
 					Command_ABL_demo.segment[i].B = demo_b;
 					Command_ABL_demo.segment[i].L = demo_l/9;
 				}
 			
 				pub1.publish(Command_ABL_demo);
 				printf("A: %f, B: %f, L: %f\r\n", demo_a, demo_b, demo_l);
 			
 				usleep(ts);
 			}

 			flag = 0;
		}
		else
		{
			for (int i = 0; i < seg; i++)
			{
				Command_ABL_demo.segment[i].A = 0;
				Command_ABL_demo.segment[i].B = 0;
				Command_ABL_demo.segment[i].L = length0;
			}

			pub1.publish(Command_ABL_demo);
		}	
				
		ros::spinOnce();
		r.sleep();
	}

	return 0;
}
