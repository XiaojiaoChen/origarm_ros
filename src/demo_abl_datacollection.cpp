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
int ts = 10*mt;    //time sleep at each point

int current_step = 0;
int count_step = 0;

float alpha1 =  0;
float alpha2 =  0;
float beta1 = 0;
float beta2 = 0;
float length1 = length0*3;
float length2 = length0*3;

// order: a2, b2, l2, a1, b1, l1
vector<vector<int>> steps {
	{     150,      1,       1,      1,      1,       1},
	{     200,      1,       1,      1,      1,       1},
	{     500,      1,       1,      1,      1,       1},
	{     250,      1,       1,      1,      1,       1},

	{     150,      1,       1,      1,      1,       1},

	{     250,      1,       1,      1,      1,       1},
	{     500,      1,       1,      1,      1,       1},
	{     250,      1,       1,      1,      1,       1},

	{     150,      1,       1,      1,      1,       1},

	{     250,      1,       1,      1,      1,       1},
	{     500,      1,       1,      1,      1,       1},
	{     250,      1,       1,      1,      1,       1}
};

// group 1_1, beta: 0->pi/6, length: l0->lmax
// vector<vector<float>> p_start_group{
// 	{-0.30*3,      0, 0.055*3,      0,      0, 0.055*3},  //initial
// 	{-0.30*3,      0, 0.055*3,      0,      0, 0.055*3}   
// };

// vector<vector<float>> p_end_group{
// 	{-0.30*3,      0, 0.055*3,      0,      0, 0.055*3},
// 	{ 0.30*3, M_PI/6, 0.065*3, 0.30*3, M_PI/6, 0.065*3}
// };

// // group 1_2, beta: 0->pi/6, length: l0->lmid
// vector<vector<float>> p_start_group{
// 	{-0.30*3,      0, 0.055*3,      0,      0, 0.055*3},  //initial
// 	{-0.30*3,      0, 0.055*3,      0,      0, 0.055*3}   
// };

// vector<vector<float>> p_end_group{
// 	{-0.30*3,      0, 0.055*3,      0,      0, 0.055*3},
// 	{ 0.30*3, M_PI/6, 0.040*3, 0.30*3, M_PI/6, 0.040*3}
// };

// group 1_3, beta: 0->pi/6, length: lmid->lmin
// vector<vector<float>> p_start_group{
// 	{-0.30*3,      0, 0.040*3,      0,      0, 0.040*3},  //initial
// 	{-0.30*3,      0, 0.040*3,      0,      0, 0.040*3}   
// };

// vector<vector<float>> p_end_group{
// 	{-0.30*3,      0, 0.040*3,      0,      0, 0.040*3},
// 	{ 0.30*3, M_PI/6, 0.030*3, 0.30*3, M_PI/6, 0.030*3}
// };

// // group 2_1, beta: pi/6->pi/3, length: l0->lmax
// vector<vector<float>> p_start_group{
// 	{-0.30*3, M_PI/6, 0.055*3,      0, M_PI/6, 0.055*3},  //initial
// 	{-0.30*3, M_PI/6, 0.055*3,      0, M_PI/6, 0.055*3}   
// };

// vector<vector<float>> p_end_group{
// 	{-0.30*3, M_PI/6, 0.055*3,      0, M_PI/6, 0.055*3},
// 	{ 0.30*3, M_PI/3, 0.065*3, 0.30*3, M_PI/3, 0.065*3}
// };

// // group 2_2, beta: pi/6->pi/3, length: l0->lmid
// vector<vector<float>> p_start_group{
// 	{-0.30*3, M_PI/6, 0.055*3,      0, M_PI/6, 0.055*3},  //initial
// 	{-0.30*3, M_PI/6, 0.055*3,      0, M_PI/6, 0.055*3}   
// };

// vector<vector<float>> p_end_group{
// 	{-0.30*3, M_PI/6, 0.055*3,      0, M_PI/6, 0.055*3},
// 	{ 0.30*3, M_PI/3, 0.040*3, 0.30*3, M_PI/3, 0.040*3}
// };

// // group 2_3, beta: pi/6->pi/3, length: lmid->lmin
// vector<vector<float>> p_start_group{
// 	{-0.30*3, M_PI/6, 0.040*3,      0, M_PI/6, 0.040*3},  //initial
// 	{-0.30*3, M_PI/6, 0.040*3,      0, M_PI/6, 0.040*3}   
// };

// vector<vector<float>> p_end_group{
// 	{-0.30*3, M_PI/6, 0.040*3,      0, M_PI/6, 0.040*3},
// 	{ 0.30*3, M_PI/3, 0.030*3, 0.30*3, M_PI/3, 0.030*3}
// };

// // group 3_1, beta: pi/3->pi/2, length: l0->lmax
// vector<vector<float>> p_start_group{
// 	{-0.30*3, M_PI/3, 0.055*3,      0, M_PI/3, 0.055*3},  //initial
// 	{-0.30*3, M_PI/3, 0.055*3,      0, M_PI/3, 0.055*3}   
// };

// vector<vector<float>> p_end_group{
// 	{-0.30*3, M_PI/3, 0.055*3,      0, M_PI/3, 0.055*3},
// 	{ 0.30*3, M_PI/2, 0.065*3, 0.30*3, M_PI/2, 0.065*3}
// };

// group 3_2, beta: pi/3->pi/2, length: l0->lmid
// vector<vector<float>> p_start_group{
// 	{-0.30*3, M_PI/3, 0.055*3,      0, M_PI/3, 0.055*3},  //initial
// 	{-0.30*3, M_PI/3, 0.055*3,      0, M_PI/3, 0.055*3}   
// };

// vector<vector<float>> p_end_group{
// 	{-0.30*3, M_PI/3, 0.055*3,      0, M_PI/3, 0.055*3},
// 	{ 0.30*3, M_PI/2, 0.040*3, 0.30*3, M_PI/2, 0.040*3}
// };

// // group 3_3, beta: pi/3->pi/2, length: lmid->lmin
// vector<vector<float>> p_start_group{
// 	{-0.30*3, M_PI/3, 0.040*3,      0, M_PI/3, 0.040*3},  //initial
// 	{-0.30*3, M_PI/3, 0.040*3,      0, M_PI/3, 0.040*3}   
// };

// vector<vector<float>> p_end_group{
// 	{-0.30*3, M_PI/3, 0.040*3,      0, M_PI/3, 0.040*3},
// 	{ 0.30*3, M_PI/2, 0.030*3, 0.30*3, M_PI/2, 0.030*3}
// };

vector<vector<float>> p_start_group{
	{	   0, M_PI*3/4, 0.055*3,       0, M_PI*3/4, 0.055*3}, //1
	{	   0, M_PI*3/4, 0.055*3,       0, M_PI*3/4, 0.055*3}, //2
	{-0.30*3, M_PI*3/4, 0.055*3, -0.30*3, M_PI*3/4, 0.055*3}, //3
	{ 0.30*3, M_PI*3/4, 0.055*3,  0.30*3, M_PI*3/4, 0.055*3}, //4
	
	{	   0, M_PI*3/4, 0.055*3,       0, M_PI*3/4, 0.055*3}, //5
	
	{	   0, M_PI*3/4, 0.055*3,       0, M_PI*3/4, 0.055*3}, //6
	{-0.30*3, M_PI*3/4, 0.055*3, -0.30*3, M_PI*3/4, 0.055*3}, //7
	{ 0.30*3, M_PI*3/4, 0.055*3,  0.30*3, M_PI*3/4, 0.055*3}, //8

	{	   0, M_PI*3/4, 0.055*3,       0, M_PI*3/4, 0.055*3}, //9

	{	   0, M_PI*3/4, 0.055*3,       0, M_PI*3/4, 0.055*3}, //10
	{-0.30*3, M_PI*3/4, 0.055*3, -0.30*3, M_PI*3/4, 0.055*3}, //11
	{ 0.30*3, M_PI*3/4, 0.055*3,  0.30*3, M_PI*3/4, 0.055*3}  //12          
};

vector<vector<float>> p_end_group{
	{	   0, M_PI*3/4, 0.055*3,      0, M_PI*3/4, 0.055*3}, //1
	{-0.30*3, M_PI*3/4, 0.055*3,      0, M_PI*3/4, 0.055*3}, //2
	{ 0.30*3, M_PI*3/4, 0.055*3, 0.30*3, M_PI*3/4, 0.055*3}, //3
	{	   0, M_PI*3/4, 0.055*3,      0, M_PI*3/4, 0.055*3}, //4
	
	{	   0, M_PI*3/4, 0.055*3,      0, M_PI*3/4, 0.055*3}, //5
	
	{-0.30*3, M_PI*3/4, 0.055*3,      0, M_PI*3/4, 0.055*3}, //6
	{ 0.30*3, M_PI*3/4, 0.055*3, 0.30*3, M_PI*3/4, 0.055*3}, //7
	{	   0, M_PI*3/4, 0.055*3,      0, M_PI*3/4, 0.055*3}, //8

	{	   0, M_PI*3/4, 0.055*3,      0, M_PI*3/4, 0.055*3}, //9

	{-0.30*3, M_PI*3/4, 0.055*3,      0, M_PI*3/4, 0.055*3}, //10
	{ 0.30*3, M_PI*3/4, 0.055*3, 0.30*3, M_PI*3/4, 0.055*3}, //11
	{	   0, M_PI*3/4, 0.055*3,      0, M_PI*3/4, 0.055*3}  //12
};

// vector<vector<int>> steps {
// 	{      2,      1,       1,      1,      1,       1},
// 	{      3,      3,       3,      3,      3,       3},
// 	{      4,      1,       1,      1,      1,       1},
// 	{      3,      3,       3,      3,      3,       3}};	

// vector<vector<float>> p_start_group{
// 	{-0.30*3,      0, 0.055*3,      0,      0, 0.055*3},  //initial
// 	{-0.30*3,      0, 0.055*3,      0,      0, 0.055*3},  //l0->lmax
// 	{-0.30*3,      0, 0.055*3,      0,      0, 0.055*3},  //initial
// 	{-0.30*3,      0, 0.055*3,      0,      0, 0.055*3}}; //l0->lmin

// vector<vector<float>> p_end_group{
// 	{-0.30*3,     0, 0.055*3,      0,      0, 0.055*3},
// 	{0.30*3, M_PI/3,  0.065*3, 0.30*3, M_PI/3,  0.065*3},
// 	{-0.30*3,     0, 0.055*3,      0,      0, 0.055*3},
// 	{0.30*3, M_PI/3,  0.03*3, 0.30*3, M_PI/3,  0.03*3}
// 	};

vector<vector<float>> trajectoryGroup;
vector<vector<vector<float>>> trajectory_group;

static int flag_start = 0;
int trajgroup_no = 0;

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
				for (int j = 0; j < cycle; j++)
				{
					point_temp = ps;
					trajectory.push_back(point_temp);
				}				
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
	int totalstep;
	vector<float> traj_temp;
	vector<vector<float>> traj_group;
	int cycle = 1;	
	
	for (int i = 0; i < tstep.size(); i++)
	{
		size_col = size_col*tstep[i];
	}
	
	for (int i = 0; i < size_row; i++)
	{		
		if (i == 0)
		{				
			totalstep = size_col/(cycle*tstep[i]);			
		}
		else
		{
			cycle = cycle*tstep[i-1];			
			totalstep = size_col/(cycle*tstep[i]);			
		}

		// printf("totalstep: %d, size_col: %d, cycle: %d, tstep: %d\r\n", totalstep, size_col, cycle, tstep[i]);

		traj_temp = trajectoryGeneration(ps[i], pe[i], tstep[i], cycle, totalstep);
		traj_group.push_back(traj_temp);
	}

	return traj_group;
}

void writeABLCommand()
{
	if (flag_start)
	{
		if (trajgroup_no < trajectory_group.size() - 1)
		{
			if (current_step >= trajectory_group[trajgroup_no][0].size())
			{
				current_step = 0;
				trajgroup_no ++; 
			}
		}
		else if (trajgroup_no == trajectory_group.size() - 1)
		{
			if (current_step >= trajectory_group[trajgroup_no][0].size())
			{									
				flag_start = 0;
			}
		}
		else
		{
			flag_start = 0;
		}
					
		alpha2 = trajectory_group[trajgroup_no][0][current_step];
		beta2 = trajectory_group[trajgroup_no][1][current_step];
		length2 = trajectory_group[trajgroup_no][2][current_step];
		alpha1 = trajectory_group[trajgroup_no][3][current_step];
		beta1 = trajectory_group[trajgroup_no][4][current_step];
		length1 = trajectory_group[trajgroup_no][5][current_step];

		current_step ++;		
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

	//load parameters
	for (int i = 0; i < p_start_group.size(); i++)
	{
		trajectoryGroup = TrajGroupGeneration(p_start_group[i], p_end_group[i], steps[i]);
		trajectory_group.push_back(trajectoryGroup);
	}

	// for (int i = 0; i < trajectory_group.size(); i++)
	// {
	// 	for (int j = 0; j < trajectory_group[i].size(); j++)
	// 	{
	// 		for (int k = 0; k < trajectory_group[i][j].size(); k++)
	// 		{
	// 			printf("trajectory_group[%d][%d][%d]: %.4f\r\n", i, j, k, trajectory_group[i][j][k]);
	// 		}			
	// 	}
	// }

	while (ros::ok())
	{				
		writeABLCommand(); 
		printf("flag_start: %d, current_step: %d\r\n", flag_start, current_step);
		printf("alpha1: %.4f, beta1: %.4f, length1: %.4f, alpha2: %.4f, beta2: %.4f, length2: %.4f\r\n", 
				alpha1, beta1, length1, alpha2, beta2, length2);

		for (int i = 0; i < 3; i++)
		{
			Command_ABL_demo.segment[i].A = alpha2/3;
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
