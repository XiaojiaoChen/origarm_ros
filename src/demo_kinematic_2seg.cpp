#include <ros/ros.h>
#include <geometry_msgs/Pose.h> 
#include <math.h>
#include <time.h>

#include "origarm_ros/Command_ABL.h"
#include "myData.h"

// n points, each points inclde {a1,b1,l1,a2,b2,l2}
float genetraj(float ps, float pe, int step, int tstep)
{
	float pm = ps + step*(pe-ps)/(tstep-1); 
	return pm;
}

const int mt = 1000; //1ms
int ts = 10*mt;      //time sleep at each point
int tsleep = 5000*mt;
int tstep[]={1200, 500, 200};
int flag = 1;

// right point with larger radius
// float alpha1 =  0.433002;
// float alpha2 = -0.321001;
// float length1 = 0.0459096;
// float length2 = 0.0508099;

// float alpha1 =  0.414002;
// float alpha2 = -0.342001;
// float length1 = 0.055;
// float length2 = 0.056;

// point with smaller radius
// float alpha1 =  0.198;
// float alpha2 = -0.139;
// float length1 = 0.0403279;
// float length2 = 0.0475594;

float alpha1 =  0.201;
float alpha2 = -0.164;
float length1 = 0.0458593;
float length2 = 0.0527699;

float beta;

int main(int argc, char **argv)
{
	ros::init(argc, argv, "demo_kinematic_2seg");
	ros::NodeHandle nh;	
	ros::Rate r(100);     //Hz

	ros::Publisher  pub1 = nh.advertise<origarm_ros::Command_ABL>("Cmd_ABL_joy", 100);	

	while (ros::ok())
	{				
		origarm_ros::Command_ABL Command_ABL_demo;
		
		//change beta 0->2*pi
		if (flag == 1)
		{
			beta = 0;
			for (int i = 0; i < 500; i++)
			{
				for (int i = 0; i < 3; i++)
				{
					Command_ABL_demo.segment[i].A = alpha1;
					Command_ABL_demo.segment[i].B = beta;
					Command_ABL_demo.segment[i].L = length1;
				}
				for (int i = 3; i < 6; i++)
				{
					Command_ABL_demo.segment[i].A = alpha2;
					Command_ABL_demo.segment[i].B = beta;
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
			for (int i = 0; i < tstep[0]; i++)
			{
				beta = genetraj(0, 2*M_PI, i, tstep[0]);
				for (int i = 0; i < 3; i++)
				{
					Command_ABL_demo.segment[i].A = alpha1;
					Command_ABL_demo.segment[i].B = beta;
					Command_ABL_demo.segment[i].L = length1;
				}
				for (int i = 3; i < 6; i++)
				{
					Command_ABL_demo.segment[i].A = alpha2;
					Command_ABL_demo.segment[i].B = beta;
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

				flag = 0; 			
			}			
		}
		else if (flag == 3)
		{
			beta = 0;
			for (int i = 0; i < 200; i++)
			{
				for (int i = 0; i < 3; i++)
				{
					Command_ABL_demo.segment[i].A = alpha1;
					Command_ABL_demo.segment[i].B = beta;
					Command_ABL_demo.segment[i].L = length1;
				}
				for (int i = 3; i < 6; i++)
				{
					Command_ABL_demo.segment[i].A = alpha2;
					Command_ABL_demo.segment[i].B = beta;
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
				flag = 0;
			}
		}
		else
		{
			for (int i = 0; i < 9; i++)
			{
				Command_ABL_demo.segment[i].A = 0;
				Command_ABL_demo.segment[i].B = 0;
				Command_ABL_demo.segment[i].L = length0;
			}
		}
						
		pub1.publish(Command_ABL_demo);				

		r.sleep();
	}
	
	return 0;

}
