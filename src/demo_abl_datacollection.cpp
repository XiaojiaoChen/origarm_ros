#include <ros/ros.h>
#include <geometry_msgs/Pose.h> 
#include <math.h>
#include <time.h>
#include <fstream>
#include <sys/time.h>
#include <ros/time.h>
#include "ros/package.h"
#include <linux/input-event-codes.h>

#include "origarm_ros/Command_ABL.h"
#include "origarm_ros/keynumber.h"
#include "myData.h"

using namespace std;

// n points, each points inclde {a1,b1,l1,a2,b2,l2}
float genetraj(float ps, float pe, int step, int tstep)
{
	float pm = ps + step*(pe-ps)/(tstep-1); 
	return pm;
}

const int mt = 1000; //1ms
int ts = 2500*mt;      //time sleep at each point
int a1_step = 3; //0->amax
int b1_step = 2; //0-bmax
int l1_step = 3; //l0-lengthmax / l0->lengthmin
int a2_step = 6; //amin->amax
int b2_step = 2; //0-bmax
int l2_step = 3; //l0-lengthmax / l0->lengthmin
int flag = 1;

float alpha1 =  0;
float alpha2 =  0;
float beta1 = 0;
float beta2 = 0;
float length1 = length0;
float length2 = length0;

float alphamax = 0.4;
float alphamin = -0.4;
float betamax = M_PI/3;
float betamin = 0;
float lengthmin = 0.03;
float lengthmax = 0.08;

int flag_saveABL = 0;
string ABLDataFileName = "";
string ABLDataPath = "";
ofstream ABLDataStream;
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

static void saveABLDataToFile()
{
	std::string curtime = getTimeNsecString();
	ABLDataStream << curtime << alpha1 << " " << beta1 << " " << length1 << " " << alpha2 << " " << beta2 << " " << length2 << endl;
}

float* generateABL(float range[12] , int step[6], int internal[6])
{
	float abl_command[6] = {0, 0, length0, 0, 0, length0};

	return abl_command;
}

void keyCallback(const origarm_ros::keynumber &key)
{
	if (key.keycodePressed == KEY_D) // 'D' pressed, KEY_D
	{
		printf(" KEY_D pressed!\r\n");

		flag_saveABL = 1;
		ABLDataBeginTime = ros::Time::now();
		ABLDataFileName = "ABLData_" + getTimeString() + ".txt";
		ABLDataStream.open(ABLDataPath + ABLDataFileName, ios::app); // ios::app
		
		cout << "Saving ABL data to" + ABLDataPath + ABLDataFileName << endl;
	}
	else if (key.keycodePressed == KEY_G) // 'G' pressed, KEY_G
	{
		flag_saveABL = 0;
		ABLDataStream.close();
		cout << "ABL data Saved" << endl;
	}	
}



int main(int argc, char **argv)
{
	ros::init(argc, argv, "demo_abl_datacollection");
	ros::NodeHandle nh;	
	ros::Rate r(100);     //Hz

	ABLDataPath = ros::package::getPath("origarm_ros") + "/data/ABLData/";

	ros::Subscriber key_sub_ = nh.subscribe("key_number", 1, keyCallback);
	ros::Publisher  pub1 = nh.advertise<origarm_ros::Command_ABL>("Cmd_ABL_joy", 100);	
	origarm_ros::Command_ABL Command_ABL_demo;

	while (ros::ok())
	{							
		if (flag == 1) // start from initial position
		{
			alpha1 = 0;
			alpha2 = alphamin;
			beta1 = 0;
			beta2 = 0;
			length1 = length0;
			length2 = length0;

			for (int i = 0; i < 5; i++)
			{
				for (int i = 0; i < 3; i++)
				{
					Command_ABL_demo.segment[i].A = alpha1;
					Command_ABL_demo.segment[i].B = beta1;
					Command_ABL_demo.segment[i].L = length1;
				}
				for (int i = 3; i < 6; i++)
				{
					Command_ABL_demo.segment[i].A = alpha2;
					Command_ABL_demo.segment[i].B = beta2;
					Command_ABL_demo.segment[i].L = length2;
				}
				for (int i = 6; i < SEGNUM; i++)
				{
					Command_ABL_demo.segment[i].A = 0;
					Command_ABL_demo.segment[i].B = 0;
					Command_ABL_demo.segment[i].L = length0;
				}

				pub1.publish(Command_ABL_demo);					
				usleep(ts);		
			}

			flag = 2;		
		}
		else if (flag == 2)
		{
			for (int step_l1 = 0; step_l1 < l1_step; step_l1 ++)
			{
				for (int step_b1 = 0; step_b1 < b1_step; step_b1 ++)
				{
					for (int step_a1 = 0; step_a1 < a1_step; step_a1 ++)
					{
						for (int step_l2 = 0; step_l2 < l2_step; step_l2 ++)
						{
							for (int step_b2 = 0; step_b2 < b2_step; step_b2 ++)
							{
								for (int step_a2 = 0; step_a2 < a2_step; step_a2 ++)
								{
									length2 = genetraj(length0, lengthmax, step_l2, l2_step);
									beta2 = genetraj(betamin, betamax, step_b2, b2_step);
									alpha2 = genetraj(alphamin,alphamax, step_a2, a2_step);
									length1 = genetraj(length0, lengthmax, step_l1, l1_step);
									beta1 = genetraj(betamin, betamax, step_b1, b1_step);
									alpha1 = genetraj(0,alphamax, step_a1, a1_step);

									for (int i = 0; i < 3; i++)
									{
										Command_ABL_demo.segment[i].A = alpha1;
										Command_ABL_demo.segment[i].B = beta1;
										Command_ABL_demo.segment[i].L = length1;
									}
									for (int i = 3; i < 6; i++)
									{
										Command_ABL_demo.segment[i].A = alpha2;
										Command_ABL_demo.segment[i].B = beta2;
										Command_ABL_demo.segment[i].L = length2;
									}
									for (int i = 6; i < SEGNUM; i++)
									{
										Command_ABL_demo.segment[i].A = 0;
										Command_ABL_demo.segment[i].B = 0;
										Command_ABL_demo.segment[i].L = length0;
									}

									pub1.publish(Command_ABL_demo);					
									usleep(ts);
								}								
							}	
						}
					}
				}
			}

			flag = 3;			
		}
		else if (flag == 3)
		{
			alpha1 = 0;
			alpha2 = alphamin;
			beta1 = 0;
			beta2 = 0;
			length1 = length0;
			length2 = length0;

			for (int i = 0; i < 5; i++)
			{
				for (int i = 0; i < 3; i++)
				{
					Command_ABL_demo.segment[i].A = 0;
					Command_ABL_demo.segment[i].B = 0;
					Command_ABL_demo.segment[i].L = length0;
				}
				for (int i = 3; i < 6; i++)
				{
					Command_ABL_demo.segment[i].A = alphamin;
					Command_ABL_demo.segment[i].B = 0;
					Command_ABL_demo.segment[i].L = length0;
				}
				for (int i = 6; i < SEGNUM; i++)
				{
					Command_ABL_demo.segment[i].A = 0;
					Command_ABL_demo.segment[i].B = 0;
					Command_ABL_demo.segment[i].L = length0;
				}

				pub1.publish(Command_ABL_demo);					
				usleep(ts);				
			}
				
			flag = 4;
		}
		else if (flag == 4)
		{
			for (int step_l1 = 0; step_l1 < l1_step; step_l1 ++)
			{
				for (int step_b1 = 0; step_b1 < b1_step; step_b1 ++)
				{
					for (int step_a1 = 0; step_a1 < a1_step; step_a1 ++)
					{
						for (int step_l2 = 0; step_l2 < l2_step; step_l2 ++)
						{
							for (int step_b2 = 0; step_b2 < b2_step; step_b2 ++)
							{
								for (int step_a2 = 0; step_a2 < a2_step; step_a2 ++)
								{
									length2 = genetraj(length0, lengthmin, step_l2, l2_step);
									beta2 = genetraj(betamin, betamax, step_b2, b2_step);
									alpha2 = genetraj(alphamin,alphamax, step_a2, a2_step);
									length1 = genetraj(length0, lengthmin, step_l1, l1_step);
									beta1 = genetraj(betamin, betamax, step_b1, b1_step);
									alpha1 = genetraj(0,alphamax, step_a1, a1_step);

									for (int i = 0; i < 3; i++)
									{
										Command_ABL_demo.segment[i].A = alpha1;
										Command_ABL_demo.segment[i].B = beta1;
										Command_ABL_demo.segment[i].L = length1;
									}
									for (int i = 3; i < 6; i++)
									{
										Command_ABL_demo.segment[i].A = alpha2;
										Command_ABL_demo.segment[i].B = beta2;
										Command_ABL_demo.segment[i].L = length2;
									}
									for (int i = 6; i < SEGNUM; i++)
									{
										Command_ABL_demo.segment[i].A = 0;
										Command_ABL_demo.segment[i].B = 0;
										Command_ABL_demo.segment[i].L = length0;
									}

									pub1.publish(Command_ABL_demo);					
									usleep(ts);
								}								
							}	
						}
					}
				}
			}

			flag = 0;			
		}
		else
		{
			alpha1 = 0;
			alpha2 = 0;
			beta1 = 0;
			beta2 = 0;
			length1 = length0;
			length2 = length0;

			for (int i = 0; i < 9; i++)
			{
				Command_ABL_demo.segment[i].A = 0;
				Command_ABL_demo.segment[i].B = 0;
				Command_ABL_demo.segment[i].L = length0;
			}

			pub1.publish(Command_ABL_demo);				
		}

		// for (int i = 0; i < 3; i++)
		// {
		// 	Command_ABL_demo.segment[i].A = alpha1;
		// 	Command_ABL_demo.segment[i].B = beta1;
		// 	Command_ABL_demo.segment[i].L = length1;
		// }
		// for (int i = 3; i < 6; i++)
		// {
		// 	Command_ABL_demo.segment[i].A = alpha2;
		// 	Command_ABL_demo.segment[i].B = beta2;
		// 	Command_ABL_demo.segment[i].L = length2;
		// }
		// for (int i = 6; i < SEGNUM; i++)
		// {
		// 	Command_ABL_demo.segment[i].A = 0;
		// 	Command_ABL_demo.segment[i].B = 0;
		// 	Command_ABL_demo.segment[i].L = length0;
		// }

		// pub1.publish(Command_ABL_demo);													
		// r.sleep();
	}
		
	return 0;
}
