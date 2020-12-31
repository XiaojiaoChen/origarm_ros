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
int ts = 10*mt;     //time sleep at each point
int tstep[]={500, 400, 500, 300, 1200};
int flag = 1;

// float alpha1 =  0.414002;
// float alpha2 =  0.0;
// float alpha3 = -0.342001;
// float length1 = 0.055;
// float length2 = 0.065;
// float length3 = 0.056;

// float alpha1 =  0.201;
// float alpha2 =  0.0;
// float alpha3 = -0.164;
// float length1 = 0.0458593;
// float length2 = 0.035;
// float length3 = 0.0527699;

float alpha1[2] = {0.414002, 0.414002};
float alpha2[2] = {0, 0};
float alpha3[2] = {-0.342001, -0.342001};

float length1[2] = {0.055, 0.0458593};
float length2[2] = {0.075, 0.035};
float length3[2] = {0.056, 0.0527699};

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
		if (flag == 1) //initial pose hold for 5 sec
		{
			for (int i = 0; i < tstep[2]; i++)
			{
				beta = 0;
				for (int i = 0; i < 2; i++)
				{
					Command_ABL_demo.segment[i].A = alpha1[0];
					Command_ABL_demo.segment[i].B = beta;
					Command_ABL_demo.segment[i].L = length1[0];
				}
				for (int i = 2; i < 4; i++)
				{
					Command_ABL_demo.segment[i].A = alpha2[0];
					Command_ABL_demo.segment[i].B = beta;
					Command_ABL_demo.segment[i].L = length2[0];
				}
				for (int i = 4; i < 6; i++)
				{
					Command_ABL_demo.segment[i].A = alpha3[0];
					Command_ABL_demo.segment[i].B = beta;
					Command_ABL_demo.segment[i].L = length3[0];
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
		else if (flag == 2) // maximum circle 
		{
			for (int i = 0; i < tstep[4]; i++)
			{
				beta = genetraj(0, M_PI, i, tstep[4]);
				for (int i = 0; i < 2; i++)
				{
					Command_ABL_demo.segment[i].A = alpha1[0];
					Command_ABL_demo.segment[i].B = beta;
					Command_ABL_demo.segment[i].L = length1[0];
				}
				for (int i = 2; i < 4; i++)
				{
					Command_ABL_demo.segment[i].A = alpha2[0];
					Command_ABL_demo.segment[i].B = beta;
					Command_ABL_demo.segment[i].L = length2[0];
				}
				for (int i = 4; i < 6; i++)
				{
					Command_ABL_demo.segment[i].A = alpha3[0];
					Command_ABL_demo.segment[i].B = beta;
					Command_ABL_demo.segment[i].L = length3[0];
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
			
			flag = 3; 			
		}
		else if (flag == 3) // maximum ending position hold for 3 sec
		{
			for (int i = 0; i < tstep[3]; i++)
			{
				beta = M_PI;
				for (int i = 0; i < 2; i++)
				{
					Command_ABL_demo.segment[i].A = alpha1[0];
					Command_ABL_demo.segment[i].B = beta;
					Command_ABL_demo.segment[i].L = length1[0];
				}
				for (int i = 2; i < 4; i++)
				{
					Command_ABL_demo.segment[i].A = alpha2[0];
					Command_ABL_demo.segment[i].B = beta;
					Command_ABL_demo.segment[i].L = length2[0];
				}
				for (int i = 4; i < 6; i++)
				{
					Command_ABL_demo.segment[i].A = alpha3[0];
					Command_ABL_demo.segment[i].B = beta;
					Command_ABL_demo.segment[i].L = length3[0];
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
		else if (flag == 4) // maximum position to minimum position
		{
			for (int i = 0; i < tstep[1]; i++)
			{
				float alpha_1 = genetraj(alpha1[0], alpha1[1], i, tstep[1]);
				float alpha_2 = 0;
				float alpha_3 = genetraj(alpha3[0], alpha3[1], i, tstep[1]);
				float beta = M_PI;
				float length_1 = genetraj(length1[0], length1[1], i, tstep[1]);
				float length_2 = genetraj(length2[0], length2[1], i, tstep[1]);
				float length_3 = genetraj(length3[0], length3[1], i, tstep[1]);

				for (int i = 0; i < 2; i++)
				{
					Command_ABL_demo.segment[i].A = alpha_1;
					Command_ABL_demo.segment[i].B = beta;
					Command_ABL_demo.segment[i].L = length_1;
				}
				for (int i = 2; i < 4; i++)
				{
					Command_ABL_demo.segment[i].A = alpha_2;
					Command_ABL_demo.segment[i].B = beta;
					Command_ABL_demo.segment[i].L = length_2;
				}
				for (int i = 4; i < 6; i++)
				{
					Command_ABL_demo.segment[i].A = alpha_3;
					Command_ABL_demo.segment[i].B = beta;
					Command_ABL_demo.segment[i].L = length_3;
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

			flag = 5;
		}
		else if (flag == 5) // minimum starting pose hold for 3 sec
		{
			for (int i = 0; i < tstep[3]; i++)
			{
				beta = M_PI;
				for (int i = 0; i < 2; i++)
				{
					Command_ABL_demo.segment[i].A = alpha1[1];
					Command_ABL_demo.segment[i].B = beta;
					Command_ABL_demo.segment[i].L = length1[1];
				}
				for (int i = 2; i < 4; i++)
				{
					Command_ABL_demo.segment[i].A = alpha2[1];
					Command_ABL_demo.segment[i].B = beta;
					Command_ABL_demo.segment[i].L = length2[1];
				}
				for (int i = 4; i < 6; i++)
				{
					Command_ABL_demo.segment[i].A = alpha3[1];
					Command_ABL_demo.segment[i].B = beta;
					Command_ABL_demo.segment[i].L = length3[1];
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

			flag = 6; 	
		}
		else if (flag == 6) // minimum circle
		{
			for (int i = 0; i < tstep[0]; i++)
			{
				beta = genetraj(M_PI, 2*M_PI, i, tstep[0]);
				for (int i = 0; i < 2; i++)
				{
					Command_ABL_demo.segment[i].A = alpha1[1];
					Command_ABL_demo.segment[i].B = beta;
					Command_ABL_demo.segment[i].L = length1[1];
				}
				for (int i = 2; i < 4; i++)
				{
					Command_ABL_demo.segment[i].A = alpha2[1];
					Command_ABL_demo.segment[i].B = beta;
					Command_ABL_demo.segment[i].L = length2[1];
				}
				for (int i = 4; i < 6; i++)
				{
					Command_ABL_demo.segment[i].A = alpha3[1];
					Command_ABL_demo.segment[i].B = beta;
					Command_ABL_demo.segment[i].L = length3[1];
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

			flag = 7;
		}
		else if (flag == 7) // minimum position hold for 3 sec
		{
			for (int i = 0; i < tstep[3]; i++)
			{
				beta = 2*M_PI;
				for (int i = 0; i < 2; i++)
				{
					Command_ABL_demo.segment[i].A = alpha1[1];
					Command_ABL_demo.segment[i].B = beta;
					Command_ABL_demo.segment[i].L = length1[1];
				}
				for (int i = 2; i < 4; i++)
				{
					Command_ABL_demo.segment[i].A = alpha2[1];
					Command_ABL_demo.segment[i].B = beta;
					Command_ABL_demo.segment[i].L = length2[1];
				}
				for (int i = 4; i < 6; i++)
				{
					Command_ABL_demo.segment[i].A = alpha3[1];
					Command_ABL_demo.segment[i].B = beta;
					Command_ABL_demo.segment[i].L = length3[1];
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
		}
						
		pub1.publish(Command_ABL_demo);						
		r.sleep();
	}
	
	return 0;

}
