#include <ros/ros.h>
#include <geometry_msgs/Pose.h> 
#include <math.h>
#include <time.h>

#include "origarm_ros/Command_ABL.h"
#include "myData.h"

// six points, each points inclde {a1,b1,l1,a2,b2,l2,a3,b3,l3}
float p[7][9] = {
					 {0,   M_PI/3,   0.11, 0, M_PI/3,   0.11,    0, M_PI/3,   0.11},	
					 {0.6, M_PI/3,   0.11, 0, M_PI/3,   0.11, -0.6, M_PI/3,   0.11},
					 {0.6, M_PI*2/3, 0.11, 0, M_PI*2/3, 0.11, -0.6, M_PI*2/3, 0.11},
					 {0.6, M_PI*3/3, 0.11, 0, M_PI*3/3, 0.11, -0.6, M_PI*3/3, 0.11},
					 {0.6, M_PI*4/3, 0.11, 0, M_PI*4/3, 0.11, -0.6, M_PI*4/3, 0.11},
					 {0.6, M_PI*5/3, 0.11, 0, M_PI*5/3, 0.11, -0.6, M_PI*5/3, 0.11},
					 {0.6, M_PI*6/3, 0.11, 0, M_PI*6/3, 0.11, -0.6, M_PI*6/3, 0.11}
				};

int k = 0;
const int mt = 1000; //1ms
int ts = 2000*mt;     //time sleep at each point
int tstep[]={1000*mt, 8000*mt, 8000*mt, 8000*mt, 8000*mt, 8000*mt, 8000*mt};

int main(int argc, char **argv)
{
	ros::init(argc, argv, "demo_kinematic_3seg");
	ros::NodeHandle nh;	
	ros::Rate r(100);     //Hz

	ros::Publisher  pub1 = nh.advertise<origarm_ros::Command_ABL>("Cmd_ABL_joy", 100);

	while (ros::ok())
	{		
		origarm_ros::Command_ABL Command_ABL_demo;

		if ( k < 7 )
		{
			for (int i = 0; i < 2; i++)
			{
				Command_ABL_demo.segment[i].A = p[k][0]/2;
				Command_ABL_demo.segment[i].B = p[k][1];
				Command_ABL_demo.segment[i].L = p[k][2]/2;
			}

			for (int i = 2; i < 4; i++)
			{
				Command_ABL_demo.segment[i].A = p[k][3]/2;
				Command_ABL_demo.segment[i].B = p[k][4];
				Command_ABL_demo.segment[i].L = p[k][5]/2;
			}

			for (int i = 4; i < 6; i++)
			{
				Command_ABL_demo.segment[i].A = p[k][6]/2;
				Command_ABL_demo.segment[i].B = p[k][7];
				Command_ABL_demo.segment[i].L = p[k][8]/2;
			}

			for (int i = 6; i < 9; i++)
			{
				Command_ABL_demo.segment[i].A = 0;
				Command_ABL_demo.segment[i].B = 0;
				Command_ABL_demo.segment[i].L = 0.055;
			}
			
				pub1.publish(Command_ABL_demo);	
				// printf("Target point: %d, %f, %f, %f, %f, %f, %f, %f, %f, %f\r\n", k, p[k][0], p[k][1], p[k][2], p[k][3], p[k][4], p[k][5], p[k][6], p[k][7], p[k][8]);
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
