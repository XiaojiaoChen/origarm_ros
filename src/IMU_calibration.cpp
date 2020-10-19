#include <ros/ros.h>
#include <geometry_msgs/Twist.h> 
#include <geometry_msgs/Pose.h> 
#include <sensor_msgs/Joy.h>
#include <math.h>
#include <time.h>
#include <iostream>
#include <fstream>
#include <vector>
#include <stdio.h>
#include <stdlib.h>

#include "origarm_ros/Command_ABL.h"
#include "origarm_ros/Seg_ABL.h"
#include "origarm_ros/SegOpening.h"
#include "origarm_ros/Command_Pre_Open.h"
#include "origarm_ros/Command_Position.h"
#include "origarm_ros/keynumber.h"
#include "origarm_ros/modenumber.h"
#include "origarm_ros/segnumber.h"
#include "origarm_ros/States.h"
#include "origarm_ros/Sensor.h"

#include "myData.h"
#include "myFunction.h"
#include <Eigen/Eigen>
#include <Eigen/Dense>
#include <Eigen/Geometry>
using Eigen::Quaternionf;

using namespace std;

int Calibration_flag = 0;
int key_no[11];
int16_t imur[seg][act][4];
float imu_init[seg][act][4];


void keyCallback(const origarm_ros::keynumber& key)
{
	for (int i = 0; i < 11; i++)
	{
		key_no[i] = key.KEY_CODE[i];
		if (key_no[10] > 0) // 'C' pressed
		{
			Calibration_flag = 1;
		}
		else
		{
			Calibration_flag = 0;
		}
	}
}

void SensorCallback(const origarm_ros::Sensor& sensordata)
{
	for (int i = 0; i < seg; i++)
	{
		for (int j = 0; j < act; j++)
		{
			imu_init[i][j][0] = sensordata.sensor_segment[i].sensor_actuator[j].pose.orientation.w; //double check
	        imu_init[i][j][1] = sensordata.sensor_segment[i].sensor_actuator[j].pose.orientation.x;
	        imu_init[i][j][2] = sensordata.sensor_segment[i].sensor_actuator[j].pose.orientation.y;
	        imu_init[i][j][3] = sensordata.sensor_segment[i].sensor_actuator[j].pose.orientation.z;
		}
	}
}


int main(int argc, char **argv)
{
	ros::init(argc, argv, "IMU_calibration");
	ros::NodeHandle nh;	
	ros::Rate r(100);     //Hz
	
	ros::Subscriber sub1 = nh.subscribe("Sensor", 1, SensorCallback);	
	ros::Subscriber sub2 = nh.subscribe("key_number", 1, keyCallback);

	ofstream data;
  data.open("/home/ubuntu/Desktop/imu_data.txt", ios::app);
  	//data.open("/home/lijing/catkin_ws/src/origarm_ros/imu_data.txt", ios::app);


	while (ros::ok())
	{
		if (Calibration_flag == 1)
		{
			//write imu data into yaml file/imu_data.txt
			for (int i = 0; i < seg; i++)
			{
				for (int j = 0; j < act; j++)
				{
					data<<imu_init[i][j][0]<<" "<<imu_init[i][j][1]<<" "<<imu_init[i][j][2]<<" "<<imu_init[i][j][3]<<endl;
				}
			}
			printf("data[0][0][0]:%f\n",imu_init[0][0][0]);
			printf("data[1][0][0]:%f\n",imu_init[1][0][0]);
			printf(" KEY_EQUAL pressed! Calibration starts!\n");
		}	
			
		ros::spinOnce();
		r.sleep();    //sleep for 1/r sec
	}

	data.close();
	return 0;
}
