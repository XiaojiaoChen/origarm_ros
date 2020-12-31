#include <ros/ros.h>
#include <geometry_msgs/Pose.h> 
#include <math.h>
#include <time.h>
#include <fstream>
#include <sys/time.h>
#include <ros/time.h>
#include "ros/package.h"
#include <linux/input-event-codes.h>
#include <vector>

#include "origarm_ros/Command_ABL.h"
#include "origarm_ros/keynumber.h"
#include "myData.h"

using namespace std;

const int mt = 1000; //1ms
int ts = 1000*mt;    //time sleep at each point

// int a1_step = 2; //0->amax
// int b1_step = 2; //0-bmax
// int l1_step = 2; //l0-lengthmax / l0->lengthmin
// int a2_step = 4; //amin->amax
// int b2_step = 2; //0-bmax
// int l2_step = 2; //l0-lengthmax / l0->lengthmin
int current_step = 0;

float alpha1 =  0;
float alpha2 =  0;
float beta1 = 0;
float beta2 = 0;
float length1 = length0*3;
float length2 = length0*3;

// float alphamax =  0.35;
// float alphamin = -0.35;
// float betamax = M_PI/3;
// float betamin = 0;
// float lengthmin = 0.03;
// float lengthmax = 0.08;

vector<float> p_start{-0.35*3,      0, 0.055*3,      0,      0, 0.055*3};
vector<float> p_end  { 0.35*3, M_PI/3,  0.08*3, 0.35*3, M_PI/3,  0.08*3};
vector<int> steps	 {      2,      2,       1,      1,      1,       1};
vector<vector<float>> trajectoryGroup;

static int flag_start = 0;

ros::Time ABLDataBeginTime;
std::string getTimeString()
{
	time_t rawtime;
	struct tm *timeinfo;
	char buffer[100];

	time(&rawtime);
	timeinfo = localtime(&rawtime);
	struct timeval time_now
	{
	};
	gettimeofday(&time_now, nullptr);
	time_t msecs_time = time_now.tv_usec;
	time_t msec = (msecs_time / 1000) % 1000;

	strftime(buffer, 100, "%G_%h_%d_%H_%M_%S", timeinfo);
	std::string ret1 = buffer;
	std::string ret = ret1 + "_" + std::to_string(msec);
	return ret;
}

std::string getTimeNsecString()
{
	ros::Time curtime = ros::Time::now();
	ros::Duration pasttime = curtime - ABLDataBeginTime;
	int64_t pasttimems = pasttime.toNSec() / 1000000;
	std::string ret = std::to_string(pasttimems);
	return ret;
}

void keyCallback(const origarm_ros::keynumber &key)
{
	if (key.keycodePressed == KEY_B) // break and reset all command pressure to 0
	{		
		// printf("KEY_B pressed!\r\n");
		flag_start = 0;
	}
	else if (key.keycodePressed == KEY_D) // same as saving button
	{
		// printf("KEY_D pressed!\r\n");
		flag_start = 1;
	}	
}

static vector<float> trajectoryGeneration(float ps, float pe, int tstep, int cycle, int totalstep)
{
	vector<float> trajectory;
	float point_temp;

	for (int k = 0; k < totalstep; k++)
	{
		for (int i = 0; i < tstep; i++)
		{
			if (tstep == 1)
			{
				point_temp = ps;
				trajectory.push_back(point_temp);
			}
			else
			{
				for (int j = 0; j < cycle; j++)
				{
					point_temp = ps + i*(pe-ps)/(tstep-1);
					trajectory.push_back(point_temp);					
				}
			}											
		}
	}
		
	return trajectory;
}

static vector<vector<float>> TrajGroupGeneration(vector<float> & ps, vector<float> & pe, vector<int> & tstep)
{	
	int size_row = tstep.size();
	int size_col = 1;
	int cycle, totalstep;
	vector<float> traj_temp;
	vector<vector<float>> traj_group;

	for (int i = 0; i < tstep.size(); i++)
	{
		size_col = size_col*tstep[i];
	}
	
	for (int i = 0; i < size_row; i++)
	{
		int cycle = 1;
		if (i == 0)
		{
			cycle = 1;	
			totalstep = size_col/(cycle*tstep[i]);			
		}
		else
		{
			cycle = cycle*tstep[i-1];
			totalstep = size_col/(cycle*tstep[i]);			
		}

		printf("totalstep: %d, size_col: %d, cycle: %d, tstep: %d\r\n", totalstep, size_col, cycle, tstep[i]);

		traj_temp = trajectoryGeneration(ps[i], pe[i], tstep[i], cycle, totalstep);
		traj_group.push_back(traj_temp);
	}

	return traj_group;
}

void writeABLCommand()
{
	if (flag_start)
	{						
		trajectoryGroup = TrajGroupGeneration(p_start, p_end, steps);

		// for (int i = 0; i < trajectoryGroup.size(); i++)
		// {
		// 	for (int j = 0; j < trajectoryGroup[i].size(); j++)
		// 	{
		// 		printf("trajectoryGroup[%d][%d]: %.4f\r\n", i, j, trajectoryGroup[i][j]);
		// 	}
		// }

		alpha2 = trajectoryGroup[0][current_step];
		beta2 = trajectoryGroup[1][current_step];
		length2 = trajectoryGroup[2][current_step];
		alpha1 = trajectoryGroup[3][current_step];
		beta1 = trajectoryGroup[4][current_step];
		length1 = trajectoryGroup[5][current_step];

		if (current_step < trajectoryGroup[0].size()-1)
		{
			current_step = current_step + 1; 
		}
		else
		{
			current_step = 0;
		}
							
		usleep(ts);
	}
	else
	{
		alpha1 = 0;
		alpha2 = 0;
		beta1  = 0;
		beta2  = 0;
		length1 = length0*3;
		length2 = length0*3;
		current_step = 0;
	}		
}

int main(int argc, char **argv)
{
	ros::init(argc, argv, "demo_abl_datacollection");
	ros::NodeHandle nh;	
	ros::Rate r(100);     //Hz

	ros::Subscriber key_sub_ = nh.subscribe("key_number", 1, keyCallback);
	ros::Publisher  pub1 = nh.advertise<origarm_ros::Command_ABL>("Cmd_ABL_joy", 100);	
	origarm_ros::Command_ABL Command_ABL_demo;

	while (ros::ok())
	{				
		writeABLCommand(); 
		// printf("flag_start: %d, current_step: %d\r\n", flag_start, current_step);
		// printf("alpha1: %.4f, beta1: %.4f, length1: %.4f, alpha2: %.4f, beta2: %.4f, length2: %.4f\r\n", 
		// 				alpha1, beta1, length1, alpha2, beta2, length2);

		for (int i = 0; i < 3; i++)
		{
			Command_ABL_demo.segment[i].A = alpha1/3;
			Command_ABL_demo.segment[i].B = beta1;
			Command_ABL_demo.segment[i].L = length1/3;
		}
		for (int i = 3; i < 6; i++)
		{
			Command_ABL_demo.segment[i].A = alpha2/3;
			Command_ABL_demo.segment[i].B = beta2;
			Command_ABL_demo.segment[i].L = length2/3;
		}
		for (int i = 6; i < SEGNUM; i++)
		{
			Command_ABL_demo.segment[i].A = 0;
			Command_ABL_demo.segment[i].B = 0;
			Command_ABL_demo.segment[i].L = length0;
		}

		pub1.publish(Command_ABL_demo);	

		ros::spinOnce(); //necessary for subscribe											
		r.sleep();		
	}
		
	return 0;
}
