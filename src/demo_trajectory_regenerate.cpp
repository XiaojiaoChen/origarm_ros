#include <ros/ros.h>
#include <geometry_msgs/Pose.h> 
#include <math.h>
#include <time.h>
#include <fstream>
#include <iostream>
#include <vector>

#include "origarm_ros/Command_Position.h"
#include "origarm_ros/Command_ABL.h"
#include "origarm_ros/modenumber.h"
#include "origarm_ros/segnumber.h"
#include "myData.h"

#define ARRAY_SIZE(a) (sizeof(a) / sizeof((a)[0]))

using namespace std;

ifstream inFile;
const int ms = 1000;  //1ms
int ts = 10*ms;

int mode_in;
float x_in, y_in, z_in;
float a_in[6], b_in[6], l_in[6];

vector<float> s_A1;
vector<float> s_B1;
vector<float> s_L1;
vector<float> s_A2;
vector<float> s_B2;
vector<float> s_L2;

//generate single point by defining starting and ending point
static float genetraj(float ps, float pe, int step, int tstep)
{
	float pm = ps + step*(pe-ps)/(tstep-1); 
	return pm;
}

// read state information from File
static void readFromFile()
{

	inFile.open("/home/lijing/catkin_ws/src/origarm_ros/predefined_param/trajp_2seg.txt", ios::in);

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

			s_A1.push_back(a_in[0]);
			s_B1.push_back(b_in[0]);
			s_L1.push_back(l_in[0]);
			s_A2.push_back(a_in[3]);
			s_B2.push_back(b_in[3]);
			s_L2.push_back(l_in[3]);	
		}
	}

	// printf("read from file: %f\n", s_A1[0]);
	// printf("read from file: %f\n", s_A1[1]);
	// printf("read from file: %f\n", s_A1[2]);

	inFile.close();	
}

static vector<float> LinearDiffTrajGeneration(vector<float> & vect1, int tstep)
{
	int vectsize = sizeof(vect1);
	
	// float[size] state;
	vector<float> state;

	for (int j = 0; j < vectsize-1; j++)
	{
		float ps = vect1[j];
		float pe = vect1[j+1];

		// printf("traj generate: %f\n", ps);
		// printf("traj generate: %f\n", pe);

		for (int i = 0; i < tstep-1; i++)
		{
			float mp = genetraj(ps, pe, i, tstep);
			state.push_back(mp);
			// printf("traj generate: %f\n", mp);

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

	vector<float> state_A1;
	vector<float> state_B1;
	vector<float> state_L1;
	vector<float> state_A2;
	vector<float> state_B2;
	vector<float> state_L2;

	readFromFile();

	printf("read from file: %f\n", s_A1[0]);
	printf("read from file: %f\n", s_A1[1]);
	printf("read from file: %f\n", s_A1[2]);
	printf("read from file: %f\n", s_A1[3]);
	printf("read from file: %f\n", s_A1[4]);
	
	state_A1 = LinearDiffTrajGeneration(s_A1, 10);

	printf("read from file: %f\n", state_A1[0]);
	printf("read from file: %f\n", state_A1[9]);
	printf("read from file: %f\n", state_A1[19]);


	state_B1 = LinearDiffTrajGeneration(s_B1, 10);
	state_L1 = LinearDiffTrajGeneration(s_L1, 10);

	printf("read from file: %f\n", state_L1[0]);
	printf("read from file: %f\n", state_L1[10]);
	printf("read from file: %f\n", state_L1[20]);

	state_A2 = LinearDiffTrajGeneration(s_A2, 10);
	state_B2 = LinearDiffTrajGeneration(s_B2, 10);
	state_L2 = LinearDiffTrajGeneration(s_L2, 10);

	while (ros::ok())
	{
		origarm_ros::Command_ABL Command_ABL_demo;
		
		int sizeoftraj = state_A1.size();
		printf("generate trajectory: %d\n", sizeoftraj);

		for (int j = 0; j < sizeoftraj; j++)
		{
			for (int i = 0; i < 3; i++)
			{
				Command_ABL_demo.segment[i].A = state_A1[j];
				Command_ABL_demo.segment[i].B = state_B1[j];
				Command_ABL_demo.segment[i].L = state_L1[j];
			}
			for (int i = 3; i < 6; i++)
			{
				Command_ABL_demo.segment[i].A = state_A2[j];
				Command_ABL_demo.segment[i].B = state_B2[j];
				Command_ABL_demo.segment[i].L = state_L2[j];
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

		r.sleep();
	}

	inFile.close();	
	return 0;
}

