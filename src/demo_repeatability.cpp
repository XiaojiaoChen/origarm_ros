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

const int mt = 10; //1ms
int ts[2] = {2600*mt, 4500*mt};    //time sleep at each point

int current_step = 0;
int count_step = 0;

vector<vector<float>> trajectory_Alpha;
vector<vector<float>> trajectory_Beta;
vector<vector<float>> trajectory_Length;

static int flag_start = 0;
int trajgroup_no = 0;


vector<float> alpha {0, 0, 0, 0, 0, 0};
vector<float> beta  {0, 0, 0, 0, 0, 0};
vector<float> length{length0, length0, length0, length0, length0, length0};

vector<int> steps {2, 2, 2, 2};

vector<vector<float>> state_Alpha{
    {   0,    0,    0,    0,    0,    0},
    {   0,    0,    0,    0,    0,    0},
    {-0.3, -0.3, -0.3, -0.3, -0.3, -0.3},
    {   0,    0,    0,    0,    0,    0}
};

vector<vector<float>> state_Beta{
    {   0,    0,    0,    0,    0,    0},
    {   0,    0,    0,    0,    0,    0},
    {   0,    0,    0,    0,    0,    0},
    {   0,    0,    0,    0,    0,    0}
};

vector<vector<float>> state_Length{
    {0.055,    0.055,    0.055,    0.055,    0.055,    0.055},
    {0.055,    0.055,    0.055,    0.055,    0.055,    0.055},
    {0.055,    0.055,    0.055,    0.055,    0.055,    0.055},
    {0.055,    0.055,    0.055,    0.055,    0.055,    0.055}
};

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
		printf("KEY_B pressed!\r\n");
		flag_start = 0;
	}
	else if (key.keycodePressed == KEY_J) // same as saving button
	{
		printf("KEY_J pressed!\r\n");
		flag_start = 1;
	}	
}

static vector<float> trajectoryGeneration(float ps, float pe, int tstep)
{
	vector<float> trajectory;
	float point_temp;
	
    for (int i = 0; i < tstep; i++)
    {
        if (tstep == 1 || ps == pe)
        {            
            point_temp = ps;
            trajectory.push_back(point_temp);				
        }
        else
        {
            point_temp = ps + i*(pe-ps)/(tstep-1);
            trajectory.push_back(point_temp);					
        }											
    }
		
	return trajectory;
}

// generate trajectory via linear differences between start&end pose, ps{a1,...,a6}
static vector<vector<float>> TrajGroupGeneration(vector<float> & ps, vector<float> & pe, int tstep)
{	
	vector<float> traj_temp;
	vector<vector<float>> traj_group;
		
	for (int i = 0; i < ps.size(); i++)
	{		
		traj_temp = trajectoryGeneration(ps[i], pe[i], tstep);
		traj_group.push_back(traj_temp);
	}

	return traj_group;
}

void writeABLCommand()
{
	if (flag_start)
	{		
        if (current_step >= trajectory_Alpha.size())
        {
            current_step = 0;
            flag_start = 0;				
        }		

        for (int i = 0; i < trajectory_Alpha.size(); i++)
        {
            for (int j = 0; j < 6; j++)
            {
                alpha[i] = trajectory_Alpha[i][j];
                beta[i] = trajectory_Beta[i][j];
                length[i] = trajectory_Length[i][j];
            }
        }			

		current_step ++;
        usleep(ts[1]);																										
	}
	else
	{
        for (int i = 0; i < 6; i++)
        {
            alpha[i]  = 0;
            beta[i]   = 0;
            length[i] = length0;
        }
		current_step = 0;
	}		
}

int main(int argc, char **argv)
{
	ros::init(argc, argv, "demo_repeatability");
	ros::NodeHandle nh;	
	ros::Rate r(100);     //Hz

	ros::Subscriber key_sub_ = nh.subscribe("key_number", 1, keyCallback);
	ros::Publisher  pub1 = nh.advertise<origarm_ros::Command_ABL>("Cmd_ABL_joy", 100);	
	origarm_ros::Command_ABL Command_ABL_demo;

	//load parameters
	for (int i = 0; i < state_Alpha.size()-1; i++)
	{
		trajectory_Alpha  = TrajGroupGeneration(state_Alpha[i], state_Alpha[i+1], steps[i]);
        trajectory_Beta   = TrajGroupGeneration(state_Beta[i], state_Beta[i+1], steps[i]);
        trajectory_Length = TrajGroupGeneration(state_Length[i], state_Length[i+1], steps[i]);
	}

	for (int i = 0; i < trajectory_Alpha.size(); i++)
	{
		for (int j = 0; j < trajectory_Alpha[i].size(); j++)
		{			
            printf("Alpha [%d][%d]: %.4f\r\n", i, j, trajectory_Alpha[i][j]);
            printf("Beta  [%d][%d]: %.4f\r\n", i, j, trajectory_Beta[i][j]);
            printf("Length[%d][%d]: %.4f\r\n", i, j, trajectory_Length[i][j]);			
		}
	}

	while (ros::ok())
	{				
		writeABLCommand(); 
		// printf("flag_start: %d, current_step: %d\r\n", flag_start, current_step);
		// printf("alpha1: %.4f, beta1: %.4f, length1: %.4f, alpha2: %.4f, beta2: %.4f, length2: %.4f\r\n", 
		// 		alpha1, beta1, length1, alpha2, beta2, length2);

		for (int i = 0; i < 6; i++)
		{
			Command_ABL_demo.segment[i].A = alpha[i];
			Command_ABL_demo.segment[i].B = beta[i];
			Command_ABL_demo.segment[i].L = length[i];
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
