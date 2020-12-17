#include <ros/ros.h>
#include <geometry_msgs/Pose.h> 
#include <math.h>
#include <time.h>
#include <fstream>
#include <iostream>
#include <vector>
#include <Eigen/Eigen>

#include "origarm_ros/Command_Position.h"
#include "origarm_ros/Command_ABL.h"
#include "origarm_ros/modenumber.h"
#include "origarm_ros/segnumber.h"
#include "myData.h"

#define ARRAY_SIZE(a) (sizeof(a) / sizeof((a)[0]))

using namespace std;

ifstream inFile;
const int ms = 1000;  //1ms
int ts = 1000*ms;

int mode_in;
float x_in, y_in, z_in;
float a_in[6], b_in[6], l_in[6];

int flag = 1;

vector<vector<float>> s_A;
vector<vector<float>> s_B;
vector<vector<float>> s_L;

vector<vector<float>> state_A;
vector<vector<float>> state_B;
vector<vector<float>> state_L;

//generate single point by defining starting and ending point
static float genetraj(float ps, float pe, int step, int tstep)
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

	inFile.open("/home/ubuntu/catkin_ws/src/origarm_ros/predefined_param/trajp_2seg.txt", ios::in);

	if (!inFile)
	{
		printf("%s\n", "unable to open the file.");
		exit(1);
	}
	else
	{
		while (!inFile.eof())
		{
			inFile>>mode_in>>x_in>>y_in>>z_in>>a_in[0]>>b_in[0]>>l_in[0]>>a_in[1]>>b_in[1]>>l_in[1]>>a_in[2]>>b_in[2]>>l_in[2]>>a_in[3]>>b_in[3]>>l_in[3]>>a_in[4]>>b_in[4]>>l_in[4]>>a_in[5]>>b_in[5]>>l_in[5];
		
			for (int i = 0; i < 6; i++)
			{
				s_a.push_back(a_in[i]);
				s_b.push_back(b_in[i]);
				s_l.push_back(l_in[i]);
			}			

			if (s_a.size() < 6)
			{
				printf("%s\n", ".............");
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

static vector<vector<float>> LinearDiffTrajGeneration(vector<vector<float>> & vect1, int tstep)
{
	int vectsize = vect1.size(); // return row size of 2d vector, size of timestamp read from file
	
	vector< vector<float> > state;
	vector<float> state1;
	vector<float> state_1;

	int t = 0;

	for (int i = 0; i < 6; i++)
	{
		for (int j = 0; j < vectsize-1; j++)
		{
			float ps = vect1[j][i];
			float pe = vect1[j+1][i];
			// printf("ps: %f, pe: %f\n", ps, pe);

			for (int k = 0; k < tstep-1; k++)
			{
				float pm = genetraj(ps, pe, k, tstep);
				state1.push_back(pm);				
				// printf("state1[%d]: %f\n", t, state1[t]);
				t ++;
			}					
		}
	
		int newvectorsize = (tstep-1)*(vectsize-1);
		
		if (state1.size() < newvectorsize)
		{
			printf("%s\n", "!.............");
		}
		else
		{
			state_1.assign(state1.end() - newvectorsize, state1.end());	
			state.push_back(state_1);
		}		
						
	}

	return state;					
}

int main(int argc, char **argv)
{
	ros::init(argc, argv, "demo_trajectory_regenerate");
	ros::NodeHandle nh;	
	ros::Rate r(100);     //Hz

	ros::Publisher  pub1 = nh.advertise<origarm_ros::Command_ABL>("Cmd_ABL_joy", 100);	

	readFromFile();

	state_A = LinearDiffTrajGeneration(s_A, 10);
	state_B = LinearDiffTrajGeneration(s_B, 10);
	state_L = LinearDiffTrajGeneration(s_L, 10);

	// for (int i = 0; i < state_A.size(); i++)
	// {
	// 	for (int j = 0; j < state_A[0].size(); j++)
	// 	{
	// 		printf("state_A[%d][%d]: %f\n", i, j, state_A[i][j]);
	// 	}		
	// }

	while (ros::ok())
	{
		origarm_ros::Command_ABL Command_ABL_demo;
		
		if (flag)
		{
			for (int j = 0; j < state_A[0].size(); j++)
			{
				for (int i = 0; i < 6; i++)
				{
					Command_ABL_demo.segment[i].A = state_A[i][j];
					Command_ABL_demo.segment[i].B = state_B[i][j];
					Command_ABL_demo.segment[i].L = state_L[i][j];
				}
				for (int i = 6; i < 9; i++)
				{
					Command_ABL_demo.segment[i].A = 0;
					Command_ABL_demo.segment[i].B = 0;
					Command_ABL_demo.segment[i].L = length0;
				}
				
				pub1.publish(Command_ABL_demo);
				usleep(ts);
			}

			flag = 0;		
		}
		else
		{
			for (int i = 0; i < 9; i++)
			{
				Command_ABL_demo.segment[i].A = 0;
				Command_ABL_demo.segment[i].B = 0;
				Command_ABL_demo.segment[i].L = length0;
			}
			
			pub1.publish(Command_ABL_demo);
			usleep(ts);
		}
		
		r.sleep();
	}

	inFile.close();	
	return 0;
}

