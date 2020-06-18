#include <ros/ros.h>
#include <geometry_msgs/Pose.h> 
#include <math.h>
#include <time.h>

#include "origarm_ros/Command_ABL.h"
#include "origarm_ros/modenumber.h"
#include "origarm_ros/segnumber.h"
#include "myData.h"

int tp = 1000;  //timestep

/*[0]arm: length: l0->lmax->l0->lmin->l0
	[1]arm: alpha:  0->pi/2->0->-pi/2
	[2]arm: beta:   0->4pi(0->pi(-pi)->0)->0
*/

int t_step[] = {1000, 1000, 4000};

int ts = 10000; //time sleep at each step
const int ms = 1000; //1ms
int t_sleep[] = {10*ms};

int flag = 1;

int mode_ = 0;
int segment_ = 0;

float lengthmax = 0.08;
float lengthmin = 0.03;

float demo_a;
float demo_b;
float demo_l;

float a_seg2[2];
float b_seg2[2];
float l_seg2[2];

float a_seg3[3];
float b_seg3[3];
float l_seg3[3];

float a_seg6[6];
float b_seg6[6];
float l_seg6[6];

float genetraj(float ps, float pe, int step, int tstep)
{
	float pm = ps + step*(pe-ps)/(tstep-1); 
	return pm;
}

float constrainb(float s)
{
	float s_new;
	if (s > M_PI && s < 3*M_PI)
	{
		s_new = s - 2*M_PI;
	}
	else if (s > 3*M_PI)
	{
		s_new = s - 4*M_PI;
	}
	else if (s < -M_PI && s> -3*M_PI)
	{
		s_new = s + 2*M_PI;
	}
	else if (s < -3*M_PI)
	{
		s_new = s + 4*M_PI;
	}
	else
	{
		s_new = s;
	}

	return s_new;
}

int main(int argc, char **argv)
{
	ros::init(argc, argv, "demo_abl");
	ros::NodeHandle nh;	
	ros::Rate r(100);     //Hz

	/*if (nh.getParam("tp", tp))
	{
		ROS_INFO("tp is set to %d\r\n", tp);
	}
	else
	{
		tp = 1000;
	}
	

	if (nh.getParam("ts", ts))
	{
		ROS_INFO("ts is set to %d\r\n", ts);
	}
	else
	{
		ts = 10000;
	}*/

	ros::Publisher  pub1 = nh.advertise<origarm_ros::Command_ABL>("Cmd_ABL_joy", 100);
	ros::Publisher  pub2 = nh.advertise<origarm_ros::modenumber>("modenumber", 100);
	ros::Publisher  pub3 = nh.advertise<origarm_ros::segnumber>("segnumber", 100);		

	while (ros::ok())
	{
		origarm_ros::Command_ABL Command_ABL_demo;
		origarm_ros::modenumber moden;
		origarm_ros::segnumber segn;

		moden.modeNumber = mode_;
		segn.segmentNumber = segment_;
		
		pub2.publish(moden);
		pub3.publish(segn);

		if (flag == 1)//writeArm
		{
			//writeArm, length: l0->lmax
			for (int i = 0; i < tp; i++)
 			{
 				demo_a = 0;
 				demo_b = 0;
 				demo_l = genetraj(length0*6, lengthmax*6, i, tp);
 				//demo_l = length0*6 + i*(lengthmax-length0)*6/(tp-1);

 				for (int i = 0; i < 6; i++)
 				{
 					Command_ABL_demo.segment[i].A = demo_a/6;
 					Command_ABL_demo.segment[i].B = demo_b;
 					Command_ABL_demo.segment[i].L = demo_l/6;
 				}

 				for (int i = 6; i < seg; i++)
 				{
 					Command_ABL_demo.segment[i].A = 0;
 					Command_ABL_demo.segment[i].B = 0;
 					Command_ABL_demo.segment[i].L = length0;
 				}

 				pub1.publish(Command_ABL_demo);
 				printf("A: %f, B: %f, L: %f\r\n", demo_a, demo_b, demo_l);
 			
 				usleep(ts); 			
 			}
 			//writeArm, length: lmax->l0
 			for (int i = 0; i < tp; i++)
 			{
 				demo_a = 0;
 				demo_b = 0;
 				demo_l = genetraj(lengthmax*6, length0*6, i, tp);

 				for (int i = 0; i < 6; i++)
 				{
 					Command_ABL_demo.segment[i].A = demo_a/6;
 					Command_ABL_demo.segment[i].B = demo_b;
 					Command_ABL_demo.segment[i].L = demo_l/6;
 				}

 				for (int i = 6; i < seg; i++)
 				{
 					Command_ABL_demo.segment[i].A = 0;
 					Command_ABL_demo.segment[i].B = 0;
 					Command_ABL_demo.segment[i].L = length0;
 				}

 				pub1.publish(Command_ABL_demo);
 				printf("A: %f, B: %f, L: %f\r\n", demo_a, demo_b, demo_l);
 			
 				usleep(ts); 			
 			}
 			//writeArm, length: l0->lmin
 			for (int i = 0; i < tp; i++)
 			{
 				demo_a = 0;
 				demo_b = 0;
 				demo_l = genetraj(length0*6, lengthmin*6, i, tp);
 
 				for (int i = 0; i < 6; i++)
 				{
 					Command_ABL_demo.segment[i].A = demo_a/6;
 					Command_ABL_demo.segment[i].B = demo_b;
 					Command_ABL_demo.segment[i].L = demo_l/6;
 				}

 				for (int i = 6; i < seg; i++)
 				{
 					Command_ABL_demo.segment[i].A = 0;
 					Command_ABL_demo.segment[i].B = 0;
 					Command_ABL_demo.segment[i].L = length0;
 				}

 				pub1.publish(Command_ABL_demo);
 				printf("A: %f, B: %f, L: %f\r\n", demo_a, demo_b, demo_l);
 			
 				usleep(ts); 			
 			}
 			//writeArm, length: lmin->l0
 			for (int i = 0; i < tp; i++)
 			{
 				demo_a = 0;
 				demo_b = 0;
 				demo_l = genetraj(lengthmin*6, length0*6, i, tp);

 				for (int i = 0; i < 6; i++)
 				{
 					Command_ABL_demo.segment[i].A = demo_a/6;
 					Command_ABL_demo.segment[i].B = demo_b;
 					Command_ABL_demo.segment[i].L = demo_l/6;
 				}

 				for (int i = 6; i < seg; i++)
 				{
 					Command_ABL_demo.segment[i].A = 0;
 					Command_ABL_demo.segment[i].B = 0;
 					Command_ABL_demo.segment[i].L = length0;
 				}

 				pub1.publish(Command_ABL_demo);
 				printf("A: %f, B: %f, L: %f\r\n", demo_a, demo_b, demo_l);
 			
 				usleep(ts); 			
 			}

 			//writeArm, alpha: 0->pi/2
			for (int i = 0; i < tp; i++)
 			{
 				demo_a = genetraj(0, M_PI*0.5, i, tp);
 				demo_b = 0;
 				demo_l = length0*6;

 				for (int i = 0; i < 6; i++)
 				{
 					Command_ABL_demo.segment[i].A = demo_a/6;
 					Command_ABL_demo.segment[i].B = demo_b;
 					Command_ABL_demo.segment[i].L = demo_l/6;
 				}

 				for (int i = 6; i < seg; i++)
 				{
 					Command_ABL_demo.segment[i].A = 0;
 					Command_ABL_demo.segment[i].B = 0;
 					Command_ABL_demo.segment[i].L = length0;
 				}
 			
 				pub1.publish(Command_ABL_demo);
 				printf("A: %f, B: %f, L: %f\r\n", demo_a, demo_b, demo_l);
 			
 				usleep(ts);
 			}
 			//writeArm, alpha: pi/2->0
 			for (int i = 0; i < tp; i++)
 			{
 				demo_a = genetraj(M_PI*0.5, 0, i, tp);
 				demo_b = 0;
 				demo_l = length0*6;

 				for (int i = 0; i < 6; i++)
 				{
 					Command_ABL_demo.segment[i].A = demo_a/6;
 					Command_ABL_demo.segment[i].B = demo_b;
 					Command_ABL_demo.segment[i].L = demo_l/6;
 				}

 				for (int i = 6; i < seg; i++)
 				{
 					Command_ABL_demo.segment[i].A = 0;
 					Command_ABL_demo.segment[i].B = 0;
 					Command_ABL_demo.segment[i].L = length0;
 				}
 			
 				pub1.publish(Command_ABL_demo);
 				printf("A: %f, B: %f, L: %f\r\n", demo_a, demo_b, demo_l);
 			
 				usleep(ts);
 			}
 			//writeArm, alpha: 0->-pi/2
 			for (int i = 0; i < tp; i++)
 			{
 				demo_a = genetraj(0, -M_PI*0.5, i, tp);
 				demo_b = 0;
 				demo_l = length0*6;

 				for (int i = 0; i < 6; i++)
 				{
 					Command_ABL_demo.segment[i].A = demo_a/6;
 					Command_ABL_demo.segment[i].B = demo_b;
 					Command_ABL_demo.segment[i].L = demo_l/6;
 				}

 				for (int i = 6; i < seg; i++)
 				{
 					Command_ABL_demo.segment[i].A = 0;
 					Command_ABL_demo.segment[i].B = 0;
 					Command_ABL_demo.segment[i].L = length0;
 				}
 			
 				pub1.publish(Command_ABL_demo);
 				printf("A: %f, B: %f, L: %f\r\n", demo_a, demo_b, demo_l);
 			
 				usleep(ts);
 			}

 			//writeArm, beta: 0->4*pi
 			for (int i = 0; i < tp; i++)
 			{
 				demo_a = -M_PI*0.5;
 				demo_b = genetraj(0, 4*M_PI, i, tp);
 				demo_b = constrainb(demo_b);
 				demo_l = length0*6;

 				for (int i = 0; i < 6; i++)
 				{
 					Command_ABL_demo.segment[i].A = demo_a/6;
 					Command_ABL_demo.segment[i].B = demo_b;
 					Command_ABL_demo.segment[i].L = demo_l/6;
 				}

 				for (int i = 6; i < seg; i++)
 				{
 					Command_ABL_demo.segment[i].A = 0;
 					Command_ABL_demo.segment[i].B = 0;
 					Command_ABL_demo.segment[i].L = length0;
 				}

 				pub1.publish(Command_ABL_demo);
 				printf("A: %f, B: %f, L: %f\r\n", demo_a, demo_b, demo_l);
 			
 				usleep(ts);
 			}
 			//writeArm, beta: 4*pi->0
 			for (int i = 0; i < tp; i++)
 			{
 				demo_a = -M_PI*0.5;
 				demo_b = genetraj(4*M_PI, 0, i, tp);
 				demo_b = constrainb(demo_b);
 				demo_l = length0*6;

 				for (int i = 0; i < 6; i++)
 				{
 					Command_ABL_demo.segment[i].A = demo_a/6;
 					Command_ABL_demo.segment[i].B = demo_b;
 					Command_ABL_demo.segment[i].L = demo_l/6;
 				}

 				for (int i = 6; i < seg; i++)
 				{
 					Command_ABL_demo.segment[i].A = 0;
 					Command_ABL_demo.segment[i].B = 0;
 					Command_ABL_demo.segment[i].L = length0;
 				}

 				pub1.publish(Command_ABL_demo);
 				printf("A: %f, B: %f, L: %f\r\n", demo_a, demo_b, demo_l);
 			
 				usleep(ts);
 			}

 			//writeArm, alpha: -pi/2->0
 			for (int i = 0; i < tp; i++)
 			{
 				demo_a = genetraj(-M_PI*0.5, 0, i, tp);
 				demo_b = 0;
 				demo_l = length0*6;

 				for (int i = 0; i < 6; i++)
 				{
 					Command_ABL_demo.segment[i].A = demo_a/6;
 					Command_ABL_demo.segment[i].B = demo_b;
 					Command_ABL_demo.segment[i].L = demo_l/6;
 				}

 				for (int i = 6; i < seg; i++)
 				{
 					Command_ABL_demo.segment[i].A = 0;
 					Command_ABL_demo.segment[i].B = 0;
 					Command_ABL_demo.segment[i].L = length0;
 				}
 			
 				pub1.publish(Command_ABL_demo);
 				printf("A: %f, B: %f, L: %f\r\n", demo_a, demo_b, demo_l);
 			
 				usleep(ts);
 			}

 			//pause for a while
 			usleep(5000000);

 			flag = 2;
		}
		else if (flag == 2)//writeTwoSegments
		{
 			//first segment, alpha: 0->pi/6
 			//second segment, alpha: 0->-pi/6
 			for (int i = 0; i < tp; i++)
 			{
 				a_seg2[0] = genetraj(0, M_PI/6, i, tp);
 				b_seg2[0] = 0;
 				l_seg2[0] = length0*3;

 				a_seg2[1] = genetraj(0, -M_PI/6, i, tp);
 				b_seg2[1] = 0;
 				l_seg2[1] = length0*3;

 				for (int i = 0; i < 3; i++)
 				{
 					Command_ABL_demo.segment[i].A = a_seg2[0]/3;
 					Command_ABL_demo.segment[i].B = b_seg2[0]/3;
 					Command_ABL_demo.segment[i].L = l_seg2[0]/3;
 				}
 				for (int i = 3; i < 6; i++)
 				{
 					Command_ABL_demo.segment[i].A = a_seg2[1]/3;
 					Command_ABL_demo.segment[i].B = b_seg2[1]/3;
 					Command_ABL_demo.segment[i].L = l_seg2[1]/3;
 				}
 				for (int i = 6; i < seg; i++)
 				{
 					Command_ABL_demo.segment[i].A = 0;
 					Command_ABL_demo.segment[i].B = 0;
 					Command_ABL_demo.segment[i].L = length0;
 				}
				
 				pub1.publish(Command_ABL_demo);
 				printf("A[0]: %f, B[0]: %f, L[0]: %f\r\nA[1]: %f, B[1]: %f, L[1]: %f\r\n", 
 						a_seg2[0], b_seg2[0], l_seg2[0],a_seg2[1], b_seg2[1], l_seg2[1]);
 			
 				usleep(ts); 			
 			}
 			
 			//first segment, beta: 0->pi
 			//second segment, beta: 0->-pi
 			for (int i = 0; i < tp; i++)
 			{
 				a_seg2[0] = M_PI/6;
 				b_seg2[0] = genetraj(0, M_PI, i, tp);
 				l_seg2[0] = length0*3;

 				a_seg2[1] = -M_PI/6;
 				b_seg2[1] = genetraj(0, -M_PI, i, tp);
 				l_seg2[1] = length0*3;

 				for (int i = 0; i < 3; i++)
 				{
 					Command_ABL_demo.segment[i].A = a_seg2[0]/3;
 					Command_ABL_demo.segment[i].B = b_seg2[0]/3;
 					Command_ABL_demo.segment[i].L = l_seg2[0]/3;
 				}
 				for (int i = 3; i < 6; i++)
 				{
 					Command_ABL_demo.segment[i].A = a_seg2[1]/3;
 					Command_ABL_demo.segment[i].B = b_seg2[1]/3;
 					Command_ABL_demo.segment[i].L = l_seg2[1]/3;
 				}
 				for (int i = 6; i < seg; i++)
 				{
 					Command_ABL_demo.segment[i].A = 0;
 					Command_ABL_demo.segment[i].B = 0;
 					Command_ABL_demo.segment[i].L = length0;
 				}
				
 				pub1.publish(Command_ABL_demo);
 				printf("A[0]: %f, B[0]: %f, L[0]: %f\r\nA[1]: %f, B[1]: %f, L[1]: %f\r\n", 
 						a_seg2[0], b_seg2[0], l_seg2[0],a_seg2[1], b_seg2[1], l_seg2[1]);
 						
 				usleep(ts); 			
 			}
 					
 			//reverse
 			//first segment, beta: pi->0
 			//second segment, beta: -pi->0			
 			for (int i = 0; i < tp; i++)
 			{
 				a_seg2[0] = M_PI/6;
 				b_seg2[0] = genetraj(M_PI, 0, i, tp);
 				l_seg2[0] = length0*3;

 				a_seg2[1] = -M_PI/6;
 				b_seg2[1] = genetraj(-M_PI, 0, i, tp);
 				l_seg2[1] = length0*3;

 				for (int i = 0; i < 3; i++)
 				{
 					Command_ABL_demo.segment[i].A = a_seg2[0]/3;
 					Command_ABL_demo.segment[i].B = b_seg2[0]/3;
 					Command_ABL_demo.segment[i].L = l_seg2[0]/3;
 				}
 				for (int i = 3; i < 6; i++)
 				{
 					Command_ABL_demo.segment[i].A = a_seg2[1]/3;
 					Command_ABL_demo.segment[i].B = b_seg2[1]/3;
 					Command_ABL_demo.segment[i].L = l_seg2[1]/3;
 				}
 				for (int i = 6; i < seg; i++)
 				{
 					Command_ABL_demo.segment[i].A = 0;
 					Command_ABL_demo.segment[i].B = 0;
 					Command_ABL_demo.segment[i].L = length0;
 				}
				
 				pub1.publish(Command_ABL_demo);
 				printf("A[0]: %f, B[0]: %f, L[0]: %f\r\nA[1]: %f, B[1]: %f, L[1]: %f\r\n", 
 						a_seg2[0], b_seg2[0], l_seg2[0],a_seg2[1], b_seg2[1], l_seg2[1]);
 			

 				usleep(ts); 			
 			}
 			
 			//first segment, alpha: pi/6->0
 			//second segment, alpha: -pi/6->0
 			for (int i = 0; i < tp; i++)
 			{
 				a_seg2[0] = genetraj(M_PI/6, 0, i, tp);
 				b_seg2[0] = 0;
 				l_seg2[0] = length0*3;

 				a_seg2[1] = genetraj(-M_PI/6, 0, i, tp);
 				b_seg2[1] = 0;
 				l_seg2[1] = length0*3;

 				for (int i = 0; i < 3; i++)
 				{
 					Command_ABL_demo.segment[i].A = a_seg2[0]/3;
 					Command_ABL_demo.segment[i].B = b_seg2[0]/3;
 					Command_ABL_demo.segment[i].L = l_seg2[0]/3;
 				}
 				for (int i = 3; i < 6; i++)
 				{
 					Command_ABL_demo.segment[i].A = a_seg2[1]/3;
 					Command_ABL_demo.segment[i].B = b_seg2[1]/3;
 					Command_ABL_demo.segment[i].L = l_seg2[1]/3;
 				}
 				for (int i = 6; i < seg; i++)
 				{
 					Command_ABL_demo.segment[i].A = 0;
 					Command_ABL_demo.segment[i].B = 0;
 					Command_ABL_demo.segment[i].L = length0;
 				}
				
 				pub1.publish(Command_ABL_demo);
 				printf("A[0]: %f, B[0]: %f, L[0]: %f\r\nA[1]: %f, B[1]: %f, L[1]: %f\r\n", 
 						a_seg2[0], b_seg2[0], l_seg2[0],a_seg2[1], b_seg2[1], l_seg2[1]);
 			
 				usleep(ts); 			
 			}

			//pause for a while
 			usleep(5000000);
 			
 			flag = 3; 			
		}
		else if (flag == 3)//writeThreeSegments
		{
			//first segment alpha: 0->pi/9
			//second segment length: l0->lmax
			//third segment alpha: 0->-pi/9
			for (int i = 0; i < tp; i++)
 			{
 				a_seg3[0] = genetraj(0, M_PI/9, i, tp);
 				b_seg3[0] = 0;
 				l_seg3[0] = length0*2;

 				a_seg3[1] = 0;
 				b_seg3[1] = 0;
 				l_seg3[1] = genetraj(length0*2, lengthmax*2, i, tp);

 				a_seg3[2] = genetraj(0, -M_PI/9, i, tp);
 				b_seg3[2] = 0;
 				l_seg3[2] = length0*2;

 				for (int i = 0; i < 2; i++)
 				{
 					Command_ABL_demo.segment[i].A = a_seg3[0]/2;
 					Command_ABL_demo.segment[i].B = b_seg3[0]/2;
 					Command_ABL_demo.segment[i].L = l_seg3[0]/2;
 				}
 				for (int i = 2; i < 4; i++)
 				{
 					Command_ABL_demo.segment[i].A = a_seg3[1]/2;
 					Command_ABL_demo.segment[i].B = b_seg3[1]/2;
 					Command_ABL_demo.segment[i].L = l_seg3[1]/2;
 				}
 				for (int i = 4; i < 6; i++)
 				{
 					Command_ABL_demo.segment[i].A = a_seg3[2]/2;
 					Command_ABL_demo.segment[i].B = b_seg3[2]/2;
 					Command_ABL_demo.segment[i].L = l_seg3[2]/2;
 				}
 				for (int i = 6; i < seg; i++)
 				{
 					Command_ABL_demo.segment[i].A = 0;
 					Command_ABL_demo.segment[i].B = 0;
 					Command_ABL_demo.segment[i].L = length0;
 				}
				
 				pub1.publish(Command_ABL_demo);
 				for(int i = 0; i < 3; i++)
 				{
 					printf("A[%d]: %f, B[%d]: %f, L[%d]: %f\r\n", i, a_seg3[i], i, b_seg3[i], i, l_seg3[i]);
 				}
 				
 				usleep(ts); 			
 			}

 			//first segment alpha: pi/9
			//second segment length: lmax->l0
			//third segment alpha: -pi/9
			for (int i = 0; i < tp; i++)
 			{
 				a_seg3[0] = M_PI/9;
 				b_seg3[0] = 0;
 				l_seg3[0] = length0*2;

 				a_seg3[1] = 0;
 				b_seg3[1] = 0;
 				l_seg3[1] = genetraj(lengthmax*2, length0*2, i, tp);

 				a_seg3[2] = -M_PI/9;
 				b_seg3[2] = 0;
 				l_seg3[2] = length0*2;

 				for (int i = 0; i < 2; i++)
 				{
 					Command_ABL_demo.segment[i].A = a_seg3[0]/2;
 					Command_ABL_demo.segment[i].B = b_seg3[0]/2;
 					Command_ABL_demo.segment[i].L = l_seg3[0]/2;
 				}
 				for (int i = 2; i < 4; i++)
 				{
 					Command_ABL_demo.segment[i].A = a_seg3[1]/2;
 					Command_ABL_demo.segment[i].B = b_seg3[1]/2;
 					Command_ABL_demo.segment[i].L = l_seg3[1]/2;
 				}
 				for (int i = 4; i < 6; i++)
 				{
 					Command_ABL_demo.segment[i].A = a_seg3[2]/2;
 					Command_ABL_demo.segment[i].B = b_seg3[2]/2;
 					Command_ABL_demo.segment[i].L = l_seg3[2]/2;
 				}
 				for (int i = 6; i < seg; i++)
 				{
 					Command_ABL_demo.segment[i].A = 0;
 					Command_ABL_demo.segment[i].B = 0;
 					Command_ABL_demo.segment[i].L = length0;
 				}
				
 				pub1.publish(Command_ABL_demo);
 				for(int i = 0; i < 3; i++)
 				{
 					printf("A[%d]: %f, B[%d]: %f, L[%d]: %f\r\n", i, a_seg3[i], i, b_seg3[i], i, l_seg3[i]);
 				}
 				
 				usleep(ts); 			
 			}

 			//first segment alpha: pi/9, beta: 0->pi
			//second segment length: l0
			//third segment alpha: -pi/9, beta: 0->-pi
			for (int i = 0; i < tp; i++)
 			{
 				a_seg3[0] = M_PI/9;
 				b_seg3[0] = genetraj(0, M_PI, i, tp);
 				l_seg3[0] = length0*2;

 				a_seg3[1] = 0;
 				b_seg3[1] = 0;
 				l_seg3[1] = length0*2;

 				a_seg3[2] = -M_PI/9;
 				b_seg3[2] = genetraj(0, -M_PI, i, tp);
 				l_seg3[2] = length0*2;

 				for (int i = 0; i < 2; i++)
 				{
 					Command_ABL_demo.segment[i].A = a_seg3[0]/2;
 					Command_ABL_demo.segment[i].B = b_seg3[0]/2;
 					Command_ABL_demo.segment[i].L = l_seg3[0]/2;
 				}
 				for (int i = 2; i < 4; i++)
 				{
 					Command_ABL_demo.segment[i].A = a_seg3[1]/2;
 					Command_ABL_demo.segment[i].B = b_seg3[1]/2;
 					Command_ABL_demo.segment[i].L = l_seg3[1]/2;
 				}
 				for (int i = 4; i < 6; i++)
 				{
 					Command_ABL_demo.segment[i].A = a_seg3[2]/2;
 					Command_ABL_demo.segment[i].B = b_seg3[2]/2;
 					Command_ABL_demo.segment[i].L = l_seg3[2]/2;
 				}
 				for (int i = 6; i < seg; i++)
 				{
 					Command_ABL_demo.segment[i].A = 0;
 					Command_ABL_demo.segment[i].B = 0;
 					Command_ABL_demo.segment[i].L = length0;
 				}
				
 				pub1.publish(Command_ABL_demo);
 				for(int i = 0; i < 3; i++)
 				{
 					printf("A[%d]: %f, B[%d]: %f, L[%d]: %f\r\n", i, a_seg3[i], i, b_seg3[i], i, l_seg3[i]);
 				}
 				usleep(ts); 			
 			}

 			//reverse
 			//first segment alpha: pi/9, beta: pi->0
			//second segment length: l0
			//third segment alpha: -pi/9, beta: -pi->0
			for (int i = 0; i < tp; i++)
 			{
 				a_seg3[0] = M_PI/9;
 				b_seg3[0] = genetraj(M_PI, 0, i, tp);
 				l_seg3[0] = length0*2;

 				a_seg3[1] = 0;
 				b_seg3[1] = 0;
 				l_seg3[1] = length0*2;

 				a_seg3[2] = -M_PI/9;
 				b_seg3[2] = genetraj(-M_PI, 0, i, tp);
 				l_seg3[2] = length0*2;

 				for (int i = 0; i < 2; i++)
 				{
 					Command_ABL_demo.segment[i].A = a_seg3[0]/2;
 					Command_ABL_demo.segment[i].B = b_seg3[0]/2;
 					Command_ABL_demo.segment[i].L = l_seg3[0]/2;
 				}
 				for (int i = 2; i < 4; i++)
 				{
 					Command_ABL_demo.segment[i].A = a_seg3[1]/2;
 					Command_ABL_demo.segment[i].B = b_seg3[1]/2;
 					Command_ABL_demo.segment[i].L = l_seg3[1]/2;
 				}
 				for (int i = 4; i < 6; i++)
 				{
 					Command_ABL_demo.segment[i].A = a_seg3[2]/2;
 					Command_ABL_demo.segment[i].B = b_seg3[2]/2;
 					Command_ABL_demo.segment[i].L = l_seg3[2]/2;
 				}
 				for (int i = 6; i < seg; i++)
 				{
 					Command_ABL_demo.segment[i].A = 0;
 					Command_ABL_demo.segment[i].B = 0;
 					Command_ABL_demo.segment[i].L = length0;
 				}
				
 				pub1.publish(Command_ABL_demo);
 				for(int i = 0; i < 3; i++)
 				{
 					printf("A[%d]: %f, B[%d]: %f, L[%d]: %f\r\n", i, a_seg3[i], i, b_seg3[i], i, l_seg3[i]);
 				}

 				usleep(ts); 			
 			}

 			//first segment alpha: pi/9->0
			//second segment length: l0
			//third segment alpha: -pi/9->0
			for (int i = 0; i < tp; i++)
 			{
 				a_seg3[0] = genetraj(M_PI/9, 0, i, tp);
 				b_seg3[0] = 0;
 				l_seg3[0] = length0*2;

 				a_seg3[1] = 0;
 				b_seg3[1] = 0;
 				l_seg3[1] = length0*2;

 				a_seg3[2] = genetraj(-M_PI/9, 0, i, tp);
 				b_seg3[2] = 0;
 				l_seg3[2] = length0*2;

 				for (int i = 0; i < 2; i++)
 				{
 					Command_ABL_demo.segment[i].A = a_seg3[0]/2;
 					Command_ABL_demo.segment[i].B = b_seg3[0]/2;
 					Command_ABL_demo.segment[i].L = l_seg3[0]/2;
 				}
 				for (int i = 2; i < 4; i++)
 				{
 					Command_ABL_demo.segment[i].A = a_seg3[1]/2;
 					Command_ABL_demo.segment[i].B = b_seg3[1]/2;
 					Command_ABL_demo.segment[i].L = l_seg3[1]/2;
 				}
 				for (int i = 4; i < 6; i++)
 				{
 					Command_ABL_demo.segment[i].A = a_seg3[2]/2;
 					Command_ABL_demo.segment[i].B = b_seg3[2]/2;
 					Command_ABL_demo.segment[i].L = l_seg3[2]/2;
 				}
 				for (int i = 6; i < seg; i++)
 				{
 					Command_ABL_demo.segment[i].A = 0;
 					Command_ABL_demo.segment[i].B = 0;
 					Command_ABL_demo.segment[i].L = length0;
 				}
				
 				pub1.publish(Command_ABL_demo);
 				for(int i = 0; i < 3; i++)
 				{
 					printf("A[%d]: %f, B[%d]: %f, L[%d]: %f\r\n", i, a_seg3[i], i, b_seg3[i], i, l_seg3[i]);
 				}
 			
 				usleep(ts); 			
 			}

			//pause for a while
 			usleep(5000000);

 			flag = 6;
		}
		else if (flag == 6)//writeSixSegments
		{
			//alpha
			for (int i = 0; i < tp; i++)
 			{
 				a_seg6[0] = genetraj(0, M_PI/18, i, tp);
 				b_seg6[0] = 0;
 				l_seg6[0] = length0;

 				a_seg6[1] = genetraj(0, -M_PI/18, i, tp);
 				b_seg6[1] = 0;
 				l_seg6[1] = length0;

 				a_seg6[2] = genetraj(0, M_PI/18, i, tp);
 				b_seg6[2] = 0;
 				l_seg6[2] = length0;

 				a_seg6[3] = genetraj(0, -M_PI/18, i, tp);
 				b_seg6[3] = 0;
 				l_seg6[3] = length0;

 				a_seg6[4] = genetraj(0, M_PI/18, i, tp);
 				b_seg6[4] = 0;
 				l_seg6[4] = length0;

 				a_seg6[5] = genetraj(0, -M_PI/18, i, tp);
 				b_seg6[5] = 0;
 				l_seg6[5] = length0;
 				 				
 				for (int i = 0; i < 6; i++)
 				{
 					Command_ABL_demo.segment[i].A = a_seg6[i];
 					Command_ABL_demo.segment[i].B = b_seg6[i];
 					Command_ABL_demo.segment[i].L = l_seg6[i];
 				}
 				for (int i = 6; i < seg; i++)
 				{
 					Command_ABL_demo.segment[i].A = 0;
 					Command_ABL_demo.segment[i].B = 0;
 					Command_ABL_demo.segment[i].L = length0;
 				}
				
 				pub1.publish(Command_ABL_demo);
 				for(int i = 0; i < 6; i++)
 				{
 					printf("A[%d]: %f, B[%d]: %f, L[%d]: %f\r\n", i, a_seg6[i], i, b_seg6[i], i, l_seg6[i]);
 				}

 				usleep(ts); 			
 			}

 			//beta
 			for (int i = 0; i < tp; i++)
 			{
 				a_seg6[0] = M_PI/18;
 				b_seg6[0] = genetraj(0, M_PI/3, i, tp);				
 				l_seg6[0] = length0;

 				a_seg6[1] = -M_PI/18;
 				b_seg6[1] = genetraj(0, 2*M_PI/3, i, tp);				
 				l_seg6[1] = length0;

 				a_seg6[2] = M_PI/18;
 				b_seg6[2] = genetraj(0, 3*M_PI/3, i, tp);				
 				l_seg6[2] = length0;

 				a_seg6[3] = -M_PI/18;
 				b_seg6[3] = genetraj(0, 4*M_PI/3, i, tp);				
 				l_seg6[3] = length0;

 				a_seg6[4] = M_PI/18;
 				b_seg6[4] = genetraj(0, 5*M_PI/3, i, tp);				
 				l_seg6[4] = length0;

 				a_seg6[5] = -M_PI/18;
 				b_seg6[5] = genetraj(0, 6*M_PI/3, i, tp);				
 				l_seg6[5] = length0;
 				
 				for (int i = 0; i < 6; i++)
 				{
 					b_seg6[i] = constrainb(b_seg6[i]);
 				}				

 				for (int i = 0; i < 6; i++)
 				{
 					Command_ABL_demo.segment[i].A = a_seg6[i];
 					Command_ABL_demo.segment[i].B = b_seg6[i];
 					Command_ABL_demo.segment[i].L = l_seg6[i];
 				}
 				for (int i = 6; i < seg; i++)
 				{
 					Command_ABL_demo.segment[i].A = 0;
 					Command_ABL_demo.segment[i].B = 0;
 					Command_ABL_demo.segment[i].L = length0;
 				}
				
 				pub1.publish(Command_ABL_demo);
 				for(int i = 0; i < 6; i++)
 				{
 					printf("A[%d]: %f, B[%d]: %f, L[%d]: %f\r\n", i, a_seg6[i], i, b_seg6[i], i, l_seg6[i]);
 				}

 				usleep(ts); 			
 			}

 			//reverse
 			//beta
 			for (int i = 0; i < tp; i++)
 			{ 
 				a_seg6[0] = M_PI/18;
 				b_seg6[0] = genetraj(M_PI/3, 0, i, tp);				
 				l_seg6[0] = length0;

 				a_seg6[1] = -M_PI/18;
 				b_seg6[1] = genetraj(2*M_PI/3, 0, i, tp);				
 				l_seg6[1] = length0;

 				a_seg6[2] = M_PI/18;
 				b_seg6[2] = genetraj(3*M_PI/3, 0, i, tp);				
 				l_seg6[2] = length0;

 				a_seg6[3] = -M_PI/18;
 				b_seg6[3] = genetraj(4*M_PI/3, 0, i, tp);				
 				l_seg6[3] = length0;

 				a_seg6[4] = M_PI/18;
 				b_seg6[4] = genetraj(5*M_PI/3, 0, i, tp);				
 				l_seg6[4] = length0;

 				a_seg6[5] = -M_PI/18;
 				b_seg6[5] = genetraj(6*M_PI/3, 0, i, tp);				
 				l_seg6[5] = length0;
 				
 				for (int i = 0; i < 6; i++)
 				{
 					b_seg6[i] = constrainb(b_seg6[i]);
 				}				

 				for (int i = 0; i < 6; i++)
 				{
 					Command_ABL_demo.segment[i].A = a_seg6[i];
 					Command_ABL_demo.segment[i].B = b_seg6[i];
 					Command_ABL_demo.segment[i].L = l_seg6[i];
 				}
 				for (int i = 6; i < seg; i++)
 				{
 					Command_ABL_demo.segment[i].A = 0;
 					Command_ABL_demo.segment[i].B = 0;
 					Command_ABL_demo.segment[i].L = length0;
 				}
				
 				pub1.publish(Command_ABL_demo);
 				for(int i = 0; i < 6; i++)
 				{
 					printf("A[%d]: %f, B[%d]: %f, L[%d]: %f\r\n", i, a_seg6[i], i, b_seg6[i], i, l_seg6[i]);
 				}

 				usleep(ts); 			
 			}

 			//alpha
 			for (int i = 0; i < tp; i++)
 			{
 				a_seg6[0] = genetraj(M_PI/18, 0, i, tp);
 				b_seg6[0] = 0;
 				l_seg6[0] = length0;

 				a_seg6[1] = genetraj(-M_PI/18, 0, i, tp);
 				b_seg6[1] = 0;
 				l_seg6[1] = length0;

 				a_seg6[2] = genetraj(M_PI/18, 0, i, tp);
 				b_seg6[2] = 0;
 				l_seg6[2] = length0;

 				a_seg6[3] = genetraj(-M_PI/18, 0, i, tp);
 				b_seg6[3] = 0;
 				l_seg6[3] = length0;

 				a_seg6[4] = genetraj(M_PI/18, 0, i, tp);
 				b_seg6[4] = 0;
 				l_seg6[4] = length0;

 				a_seg6[5] = genetraj(-M_PI/18, 0, i, tp);
 				b_seg6[5] = 0;
 				l_seg6[5] = length0;
 				 				
 				for (int i = 0; i < 6; i++)
 				{
 					Command_ABL_demo.segment[i].A = a_seg6[i];
 					Command_ABL_demo.segment[i].B = b_seg6[i];
 					Command_ABL_demo.segment[i].L = l_seg6[i];
 				}
 				for (int i = 6; i < seg; i++)
 				{
 					Command_ABL_demo.segment[i].A = 0;
 					Command_ABL_demo.segment[i].B = 0;
 					Command_ABL_demo.segment[i].L = length0;
 				}
				
 				pub1.publish(Command_ABL_demo);
 				for(int i = 0; i < 6; i++)
 				{
 					printf("A[%d]: %f, B[%d]: %f, L[%d]: %f\r\n", i, a_seg6[i], i, b_seg6[i], i, l_seg6[i]);
 				}

 				usleep(ts); 			
 			}

			flag = 0;
		}
		else
		{
			for (int i = 0; i < seg; i++)
			{
				Command_ABL_demo.segment[i].A = 0;
				Command_ABL_demo.segment[i].B = 0;
				Command_ABL_demo.segment[i].L = length0;
			}

			pub1.publish(Command_ABL_demo);
		}	
				
		//ros::spinOnce();
		r.sleep();
	}

	return 0;
}
