#include <ros/ros.h>
#include <geometry_msgs/Pose.h> 
#include <math.h>
#include <time.h>
#include <ros/ros.h>
#include <geometry_msgs/Pose.h> 
#include <math.h>
#include <time.h>
#include <fstream>
#include <iostream>
#include <vector>
#include <Eigen/Eigen>
#include <linux/input-event-codes.h>

#include "origarm_ros/Command_Position.h"
#include "origarm_ros/Command_ABL.h"
#include "origarm_ros/modenumber.h"
#include "origarm_ros/segnumber.h"
#include "origarm_ros/keynumber.h"
#include "ros/package.h"
#include "myData.h"

#define ARRAY_SIZE(a) (sizeof(a) / sizeof((a)[0]))

using namespace std;

ifstream inFile;
string GivenDataFilePath = ros::package::getPath("origarm_ros") + "/predefined_param/";
string GivenDataFileName = "repeat_1_0.txt";

const int ms = 1000;  //1ms
int ts[] = {5*ms, 10*ms, 60*ms, 10*ms};

int mode_in;
float x_in, y_in, z_in;
float a_in[6], b_in[6], l_in[6];

int flag_start = 0;
int period = 175;
int repeat = 12;  // whole repeat times
int num = 0;     // smallest step number
int loop_no = 0; // to which step exactly

vector<vector<float>> s_A;
vector<vector<float>> s_B;
vector<vector<float>> s_L;

vector<vector<float>> state_A;
vector<vector<float>> state_B;
vector<vector<float>> state_L;

vector<float> alpha {0, 0, 0, 0, 0, 0};
vector<float> beta  {0, 0, 0, 0, 0, 0};
vector<float> length{length0, length0, length0, length0, length0, length0};

void keyCallback(const origarm_ros::keynumber &key)
{
	if (key.keycodePressed == KEY_B) // break and reset all command pressure to 0
	{		
		printf("KEY_B pressed!\r\n");
		flag_start = 0;
		repeat = 12;
	}
	else if (key.keycodePressed == KEY_J) // same as saving button
	{
		printf("KEY_J pressed!\r\n");
		flag_start = 1;
	}	
}

//generate single point by defining starting and ending point
static float lineardiffgenetraj(float ps, float pe, int step, int tstep)
{
	float pm = ps + step*(pe-ps)/(tstep-1); 
	return pm;
}

//read state information from File
static void readFromFile()
{
	vector<float> s_a;
	vector<float> s_b;
	vector<float> s_l;
	vector<float> s_a1;
	vector<float> s_b1;
	vector<float> s_l1;

	inFile.open(GivenDataFilePath + GivenDataFileName, ios::in);

	if (!inFile)
	{
		printf("%s\n", "unable to open the file.");
		exit(1);
	}
	else
	{
		while (true)
		{
			inFile>>mode_in>>x_in>>y_in>>z_in>>a_in[0]>>b_in[0]>>l_in[0]>>a_in[1]>>b_in[1]>>l_in[1]>>a_in[2]>>b_in[2]>>l_in[2]>>a_in[3]>>b_in[3]>>l_in[3]>>a_in[4]>>b_in[4]>>l_in[4]>>a_in[5]>>b_in[5]>>l_in[5];
		
			if ( inFile.eof() )	
			{
				break;
			}	

			for (int i = 0; i < 6; i++)
			{
				s_a.push_back(a_in[i]);
				s_b.push_back(b_in[i]);
				s_l.push_back(l_in[i]);
			}			

			if (s_a.size() < 6)
			{
				printf("%s\n", ".............");
				exit(1);
			}
			else
			{
				s_a1.assign(s_a.end()-6, s_a.end());
				s_b1.assign(s_b.end()-6, s_b.end());
				s_l1.assign(s_l.end()-6, s_l.end());

				s_A.push_back(s_a1);
				s_B.push_back(s_b1);
				s_L.push_back(s_l1);
			}	
		}		
	}

	inFile.close();	
}

static vector<vector<float>> TrajGeneration(vector<vector<float>> & vect1, int tstep)
{
	int vectsize = vect1.size(); // return row size of 2d vector, size of timestamp read from file
	
	vector< vector<float> > state;
	vector<float> state_1;
	vector<float> state1;
		
	int t = 0;

	for (int i = 0; i < 6; i++)
	{
		for (int j = 0; j < vectsize-1; j++)
		{
			float ps = vect1[j][i];
			float pe = vect1[j+1][i];
			
			// printf("ps: %f, pe: %f\n", ps, pe);
			
			for (int k = 0; k < tstep; k++)// include both starting & ending point
			{
				float pm = lineardiffgenetraj(ps, pe, k, tstep);
				state1.push_back(pm);				
				// printf("state1[%d]: %f\n", t, state1[t]);
				t ++;
			}			
		}			

		int newvectorsize = tstep*(vectsize-1);
	
		if (state1.size() < newvectorsize)
		{
			printf("%s\n", "!.............");
			exit(1);
		}
		else
		{
			state_1.assign(state1.end() - newvectorsize, state1.end());	
			state.push_back(state_1);
		}										
	}

	return state;					
}

static void writeABLCommand()
{
	if (flag_start && repeat > 0)
	{
		if (num < state_A[0].size())
		{
			for (int i = 0; i < 6; i++)
			{
				alpha[i]  = state_A[i][num];
				beta[i]   = state_B[i][num];
				length[i] = state_L[i][num];
			}
			
			usleep(ts[loop_no]);			
			num ++;
			if (num % period == 0)
			{
				loop_no ++;
			}									
		}
		else
		{
			num = 0;
			loop_no = 0;
			repeat = repeat - 1;
		}					
	}		
	else
	{
		num = 0;
		loop_no = 0;

		for (int i = 0; i < 6; i++)
		{
			alpha[i]  = 0;
			beta[i]   = 0;
			length[i] = length0;
		}
	}	
}

int main(int argc, char **argv)
{
	ros::init(argc, argv, "demo_repeatability");
	ros::NodeHandle nh;	
	ros::Rate r(100);     //Hz

	ros::Publisher  pub1 = nh.advertise<origarm_ros::Command_ABL>("Cmd_ABL_joy", 100);
	ros::Subscriber key_sub_ = nh.subscribe("key_number", 1, keyCallback);
	origarm_ros::Command_ABL Command_ABL_demo;	

	readFromFile();

	// for (int i = 0; i < s_A.size(); i++)
	// {
	// 	for (int j = 0; j < s_A[0].size(); j++)
	// 	{
	// 		printf("s_A[%d][%d]: %f\n", i, j, s_A[i][j]);
	// 	}		
	// }

	state_A = TrajGeneration(s_A, period);
	state_B = TrajGeneration(s_B, period);
	state_L = TrajGeneration(s_L, period);

	// for (int i = 0; i < state_A.size(); i++)
	// {
	// 	for (int j = 0; j < state_A[0].size(); j++)
	// 	{
	// 		printf("state_A[%d][%d]: %f\n", i, j, state_A[i][j]);
	// 	}		
	// }

	while (ros::ok())
	{				
		writeABLCommand(); 
		printf("flag_start: %d, num: %d, loop_no: %d, repeat: %d\r\n", flag_start, num, loop_no, repeat);
		for (int i = 0; i < 6; i++)
		{
			printf("alpha[%d]: %.4f, beta[%d]: %.4f, length[%d]: %.4f\r\n", i, alpha[i], i, beta[i], i, length[i]);
		}
		
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

	inFile.close();	
	return 0;
}

