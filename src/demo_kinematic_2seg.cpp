#include <ros/ros.h>
#include <geometry_msgs/Pose.h> 
#include <math.h>
#include <time.h>

#include "origarm_ros/Command_ABL.h"
#include "myData.h"

// six points, each points inclde {a1,b1,l1,a2,b2,l2}
float p[7][6] = {
					 {0,   M_PI/3,   0.165,    0, M_PI/3,   0.165},
					 {0.9, M_PI/3,   0.165, -0.9, M_PI/3,   0.165},
					 {0.9, M_PI*2/3, 0.165, -0.9, M_PI*2/3, 0.165},
					 {0.9, M_PI,     0.165, -0.9, M_PI,     0.165},
					 {0.9, M_PI*4/3, 0.165, -0.9, M_PI*4/3, 0.165},
					 {0.9, M_PI*5/3, 0.165, -0.9, M_PI*5/3, 0.165},
					 {0.9, M_PI*2,   0.165, -0.9, M_PI*2,   0.165}
				};

int k = 0;
const int mt = 1000; //1ms
int ts = 2000*mt;     //time sleep at each point
int tstep[]={8000*mt, 8000*mt, 8000*mt, 8000*mt, 8000*mt, 8000*mt};

int main(int argc, char **argv)
{
	ros::init(argc, argv, "demo_kinematic_2seg");
	ros::NodeHandle nh;	
	ros::Rate r(100);     //Hz

	ros::Publisher  pub1 = nh.advertise<origarm_ros::Command_ABL>("Cmd_ABL_joy", 100);	

	while (ros::ok())
	{				
		origarm_ros::Command_ABL Command_ABL_demo;
		
		if (k < 7)
		{
			for (int i = 0; i < 3; i++)
			{
				Command_ABL_demo.segment[i].A = p[k][0]/3;
				Command_ABL_demo.segment[i].B = p[k][1];
				Command_ABL_demo.segment[i].L = p[k][2]/3;
			}

			for (int i = 3; i < 6; i++)
			{
				Command_ABL_demo.segment[i].A = p[k][3]/3;
				Command_ABL_demo.segment[i].B = p[k][4];
				Command_ABL_demo.segment[i].L = p[k][5]/3;
			}

			for (int i = 6; i < 9; i++)
			{
				Command_ABL_demo.segment[i].A = 0;
				Command_ABL_demo.segment[i].B = 0;
				Command_ABL_demo.segment[i].L = 0.055;
			}
			
				pub1.publish(Command_ABL_demo);	
				// printf("Target point: %d, %f, %f, %f, %f, %f, %f\r\n", k, p[k][0], p[k][1], p[k][2]/3, p[k][3], p[k][4], p[k][5]/3);
				//usleep(tstep[k]);
				usleep(ts);	
				k = k + 1;						
		}
		else
		{			
			for (int i = 0; i < 9; i++)
			{
				Command_ABL_demo.segment[i].A = 0;
				Command_ABL_demo.segment[i].B = 0;
				Command_ABL_demo.segment[i].L = 0.055;
			}
			
			pub1.publish(Command_ABL_demo);				
		}

		r.sleep();
	}
	
	return 0;

}
