#include <ros/ros.h>
#include <geometry_msgs/Pose.h> 
#include <math.h>
#include <time.h>

#include "origarm_ros/Command_Position.h"
#include "origarm_ros/Command_ABL.h"
#include "origarm_ros/modenumber.h"
#include "origarm_ros/segnumber.h"
#include "myData.h"

int tp = 1000;   //timestep
int ts = 10000;  //time sleep at each step
int repeat = 2; //cycles A->B, B->A
int flag = 1;

int mode_ = 3;
int segment_ = 0;

float demo_x;
float demo_y;
float demo_z = length0;

float xorigin = 0.0;
float yorigin = 0.0;
float zorigin = length0;

float xa = 0.06;
float ya = 0.02;
float za = 0.07;

float xb = -0.04;
float yb = 0.04;
float zb = 0.05;

int main(int argc, char **argv)
{
	ros::init(argc, argv, "demo_picknplace");
	ros::NodeHandle nh;	
	ros::Rate r(100);     //Hz

	if (nh.getParam("tp", tp))
	{
		ROS_INFO("tp is set to %d\r\n", tp);
	}
	else
	{
		tp = 1000;
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
		repeat = 2;
	}

	ros::Publisher  pub1 = nh.advertise<origarm_ros::Command_Position>("Cmd_Position", 100);
	ros::Publisher  pub2 = nh.advertise<origarm_ros::modenumber>("modenumber", 100);
	ros::Publisher  pub3 = nh.advertise<origarm_ros::segnumber>("segnumber", 100);		

	while (ros::ok())
	{
		origarm_ros::Command_Position Command_XYZ_demo;
		origarm_ros::modenumber moden;
		origarm_ros::segnumber segn;
		
		if (flag == 1)
		{
			// original point -> pose A
			for (int i = 0; i < tp; i++)
			{
				moden.modeNumber = mode_;
				pub2.publish(moden);
				segn.segmentNumber = segment_;
				pub3.publish(segn);

				demo_x = i*xa/(tp-1);
				demo_y = i*ya/(tp-1);
				demo_z = length0 + i*(za-length0)/(tp-1);

				Command_XYZ_demo.pose.position.x = demo_x;
				Command_XYZ_demo.pose.position.y = demo_y;
				Command_XYZ_demo.pose.position.z = demo_z;
				Command_XYZ_demo.pose.orientation.x = 1;
				Command_XYZ_demo.pose.orientation.w = 1;

				pub1.publish(Command_XYZ_demo);
				
				printf("O->A, X: %f, Y: %f, Z: %f\r\n", demo_x, demo_y, demo_z);
 				usleep(ts);
			}
						
			flag = 2;
		}
		else if (flag == 2)
		{
			for (int j = 0; j < repeat; j++)
			{
				// pose A -> pose B
				for (int i = 0; i < tp; i++)
				{
					moden.modeNumber = mode_;
					pub2.publish(moden);
					segn.segmentNumber = segment_;
					pub3.publish(segn);

					demo_x = xa + i*(xb - xa)/(tp-1);
					demo_y = ya + i*(yb - ya)/(tp-1);
					demo_z = za + i*(zb - za)/(tp-1);

					Command_XYZ_demo.pose.position.x = demo_x;
					Command_XYZ_demo.pose.position.y = demo_y;
					Command_XYZ_demo.pose.position.z = demo_z;
					Command_XYZ_demo.pose.orientation.x = 1;
					Command_XYZ_demo.pose.orientation.w = 1;

					pub1.publish(Command_XYZ_demo);
					
					printf("A->B[%d], X: %f, Y: %f, Z: %f\r\n", j, demo_x, demo_y, demo_z);
 					usleep(ts);
				}

				// pose B -> pose A 			
				for (int i = 0; i < tp; i++)
				{
					moden.modeNumber = mode_;
					pub2.publish(moden);
					segn.segmentNumber = segment_;
					pub3.publish(segn);

					demo_x = xb + i*(xa - xb)/(tp-1);
					demo_y = yb + i*(ya - yb)/(tp-1);
					demo_z = zb + i*(za - zb)/(tp-1);

					Command_XYZ_demo.pose.position.x = demo_x;
					Command_XYZ_demo.pose.position.y = demo_y;
					Command_XYZ_demo.pose.position.z = demo_z;
					Command_XYZ_demo.pose.orientation.x = 1;
					Command_XYZ_demo.pose.orientation.w = 1;

					pub1.publish(Command_XYZ_demo);

					printf("B->A[%d], X: %f, Y: %f, Z: %f\r\n", j, demo_x, demo_y, demo_z);
 					usleep(ts);
				}
			}
			
			flag = 3;			
		}
		else if (flag == 3)
		{
			// pose A -> original point
			for (int i = 0; i < tp; i++)
			{
				moden.modeNumber = mode_;
				pub2.publish(moden);
				segn.segmentNumber = segment_;
				pub3.publish(segn);

				demo_x = xa + i*(xorigin - xa)/(tp-1);
				demo_y = ya + i*(yorigin - ya)/(tp-1);
				demo_z = za + i*(zorigin - za)/(tp-1);

				Command_XYZ_demo.pose.position.x = demo_x;
				Command_XYZ_demo.pose.position.y = demo_y;
				Command_XYZ_demo.pose.position.z = demo_z;
				Command_XYZ_demo.pose.orientation.x = 1;
				Command_XYZ_demo.pose.orientation.w = 1;

				pub1.publish(Command_XYZ_demo);
				

				printf("A->O, X: %f, Y: %f, Z: %f\r\n", demo_x, demo_y, demo_z);
 				usleep(ts);
			}

			flag = 0;				
		}
		else
		{
			moden.modeNumber = mode_;
			pub2.publish(moden);
			segn.segmentNumber = segment_;
			pub3.publish(segn);

			demo_x = xorigin;
			demo_y = yorigin;
			demo_z = zorigin;

			Command_XYZ_demo.pose.position.x = demo_x;
			Command_XYZ_demo.pose.position.y = demo_y;
			Command_XYZ_demo.pose.position.z = demo_z;
			Command_XYZ_demo.pose.orientation.x = 1;
			Command_XYZ_demo.pose.orientation.w = 1;
			
			pub1.publish(Command_XYZ_demo);
			
			printf("stay O, X: %f, Y: %f, Z: %f\r\n", demo_x, demo_y, demo_z);
		}	
				
		ros::spinOnce();
		r.sleep();
	}

	return 0;
}
