#include <ros/ros.h>
#include <geometry_msgs/Pose.h> 
#include <math.h>
#include <time.h>

#include "origarm_ros/Command_ABL.h"
#include "origarm_ros/modenumber.h"
#include "origarm_ros/segnumber.h"
#include "myData.h"

int tp = 1000;  //timestep

/*[0]arm:  length: l0->lmax->l0->lmin->l0
	[1]arm:  alpha:  0->0.7*6->0->-0.7*6 [->0]
	[2]arm:  beta:   0->2pi->0
	[3]2seg: alpha:  seg[0] 0->0.7*3; seg[1] 0->-0.7*3 [reverse]
	[4]2seg: beta:   seg[0] 0->2pi; seg[1] 0->-2pi  [reverse]
	[5]3seg:     :   seg[0] 0->0.7*2; seg[1] l->lmax; seg[2] 0->-0.7*2 
	[6]3seg:     :   seg[0] 0.7; seg[1] l->l0; seg[2] -0.7 
	[7]3seg:     :   seg[0] beta 0->pi; seg[2] beta 0->-pi [reverse]//beta,l
	[8]3seg:     :   seg[0] 0.7->0; seg[2] -0.7->0 
	[9]6seg: alpha[reverse]
	[10]6seg:beta[reverse]
	[11]unit segmeng alpha
	[12]unit segment beta
*/

const float a_useg = 0.7;

//int t_step[] = {500, 500, 1500, 500, 1500, 500, 500, 750, 500, 500, 1500, 250, 1500};
  int t_step[] = {400, 400,  750, 300,  750, 500, 300, 500, 100, 300,  600, 250, 750};

const int ms = 1000; //1ms
int ts = 10*ms; //time sleep at each step
int t_sleep[] = {10*ms, 5000*ms, 1000*ms};

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

float genetrajm(float ps, float pe, float pmiddle, int step, int tstep)
{
	float pn;

	if (step <= int(tstep/2)-1)
	{
		pn = ps + step*(pmiddle-ps)/(int(tstep/2)-1); 
	}
	else
	{
		pn = pmiddle + step*(pe-pmiddle)/(int(tstep/2)-1);
	}
	
	return pn;
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
			for (int i = 0; i < t_step[0]; i++)
 			{
 				demo_a = 0;
 				demo_b = 0;
 				demo_l = genetraj(length0*6, lengthmax*6, i, t_step[0]);
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
 			for (int i = 0; i < t_step[0]; i++)
 			{
 				demo_a = 0;
 				demo_b = 0;
 				demo_l = genetraj(lengthmax*6, length0*6, i, t_step[0]);

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
 			for (int i = 0; i < t_step[0]; i++)
 			{
 				demo_a = 0;
 				demo_b = 0;
 				demo_l = genetraj(length0*6, lengthmin*6, i, t_step[0]);
 
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
 			for (int i = 0; i < t_step[0]; i++)
 			{
 				demo_a = 0;
 				demo_b = 0;
 				demo_l = genetraj(lengthmin*6, length0*6, i, t_step[0]);

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

 			//writeArm, alpha: 0->0.7*6
			for (int i = 0; i < t_step[1]; i++)
 			{
 				demo_a = genetraj(0, a_useg*6, i, t_step[1]);
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
 			//writeArm, alpha: 0.7*6->0
 			for (int i = 0; i < t_step[1]; i++)
 			{
 				demo_a = genetraj(a_useg*6, 0, i, t_step[1]);
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
 			//writeArm, alpha: 0->-0.7*6
 			for (int i = 0; i < t_step[1]; i++)
 			{
 				demo_a = genetraj(0, -a_useg*6, i, t_step[1]);
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

 			//writeArm, beta: 0->2*pi
 			for (int i = 0; i < t_step[2]; i++)
 			{
 				demo_a = -a_useg*6;
 				demo_b = genetraj(0, 2*M_PI, i, t_step[2]);
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
 			//writeArm, beta: 2*pi->0
 			/*for (int i = 0; i < t_step[2]; i++)
 			{
 				demo_a = -a_useg*6;
 				demo_b = genetraj(2*M_PI, 0, i, t_step[2]);
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
 			}*/

 			//writeArm, alpha: -0.7*6->0
 			/*for (int i = 0; i < t_step[1]; i++)
 			{
 				demo_a = genetraj(-a_useg*6, 0, i, t_step[1]);
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
 			}*/

 			//pause for a while
 			//usleep(t_sleep[1]);

 			flag = 3;
		}
		else if (flag == 2)//writeTwoSegments
		{
			//seg1-alpha
 			for (int i = 0; i < t_step[11]; i++)
 			{
 				a_seg2[0] = genetraj(0, a_useg*3, i, t_step[11]);
 				b_seg2[0] = 0;
 				l_seg2[0] = length0*3;

 				a_seg2[1] = 0;
 				b_seg2[1] = 0;
 				l_seg2[1] = length0*3;

 				for (int i = 0; i < 3; i++)
 				{
 					Command_ABL_demo.segment[i].A = a_seg2[0]/3;
 					Command_ABL_demo.segment[i].B = b_seg2[0];
 					Command_ABL_demo.segment[i].L = l_seg2[0]/3;
 				}
 				for (int i = 3; i < 6; i++)
 				{
 					Command_ABL_demo.segment[i].A = a_seg2[1]/3;
 					Command_ABL_demo.segment[i].B = b_seg2[1];
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

			//seg1-beta
 			for (int i = 0; i < t_step[12]; i++)
 			{
 				a_seg2[0] = a_useg*3;
 				b_seg2[0] = genetraj(0, 2*M_PI, i, t_step[12]);
				b_seg2[0] = constrainb(b_seg2[0]);
 				l_seg2[0] = length0*3;

 				a_seg2[1] = 0;
 				b_seg2[1] = 0;
 				l_seg2[1] = length0*3;

 				for (int i = 0; i < 3; i++)
 				{
 					Command_ABL_demo.segment[i].A = a_seg2[0]/3;
 					Command_ABL_demo.segment[i].B = b_seg2[0];
 					Command_ABL_demo.segment[i].L = l_seg2[0]/3;
 				}
 				for (int i = 3; i < 6; i++)
 				{
 					Command_ABL_demo.segment[i].A = a_seg2[1]/3;
 					Command_ABL_demo.segment[i].B = b_seg2[1];
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
			
			//seg1, reset				
 			for (int i = 0; i < seg; i++)
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

			//seg2-alpha
 			for (int i = 0; i < t_step[11]; i++)
 			{
 				a_seg2[0] = 0;
 				b_seg2[0] = 0;
 				l_seg2[0] = length0*3;

 				a_seg2[1] = genetraj(0, a_useg*3, i, t_step[11]);
 				b_seg2[1] = 0;
 				l_seg2[1] = length0*3;

 				for (int i = 0; i < 3; i++)
 				{
 					Command_ABL_demo.segment[i].A = a_seg2[0]/3;
 					Command_ABL_demo.segment[i].B = b_seg2[0];
 					Command_ABL_demo.segment[i].L = l_seg2[0]/3;
 				}
 				for (int i = 3; i < 6; i++)
 				{
 					Command_ABL_demo.segment[i].A = a_seg2[1]/3;
 					Command_ABL_demo.segment[i].B = b_seg2[1];
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

			//seg2-beta
 			for (int i = 0; i < t_step[12]; i++)
 			{
 				a_seg2[0] = 0;
 				b_seg2[0] = 0;
 				l_seg2[0] = length0*3;

 				a_seg2[1] = a_useg*3;
 				b_seg2[1] = genetraj(0, 2*M_PI, i, t_step[12]);
				b_seg2[1] = constrainb(b_seg2[1]);
 				l_seg2[1] = length0*3;

 				for (int i = 0; i < 3; i++)
 				{
 					Command_ABL_demo.segment[i].A = a_seg2[0]/3;
 					Command_ABL_demo.segment[i].B = b_seg2[0];
 					Command_ABL_demo.segment[i].L = l_seg2[0]/3;
 				}
 				for (int i = 3; i < 6; i++)
 				{
 					Command_ABL_demo.segment[i].A = a_seg2[1]/3;
 					Command_ABL_demo.segment[i].B = b_seg2[1];
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

			//seg2, reset				
 			for (int i = 0; i < seg; i++)
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
			
			//first segment, alpha: 0->0.7*3
 			//second segment, alpha: 0->-0.7*3
 			for (int i = 0; i < t_step[3]; i++)
 			{
 				a_seg2[0] = genetraj(0, a_useg*3, i, t_step[3]);
 				b_seg2[0] = 0;
 				l_seg2[0] = length0*3;

 				a_seg2[1] = genetraj(0, -a_useg*3, i, t_step[3]);
 				b_seg2[1] = 0;
 				l_seg2[1] = length0*3;

 				for (int i = 0; i < 3; i++)
 				{
 					Command_ABL_demo.segment[i].A = a_seg2[0]/3;
 					Command_ABL_demo.segment[i].B = b_seg2[0];
 					Command_ABL_demo.segment[i].L = l_seg2[0]/3;
 				}
 				for (int i = 3; i < 6; i++)
 				{
 					Command_ABL_demo.segment[i].A = a_seg2[1]/3;
 					Command_ABL_demo.segment[i].B = b_seg2[1];
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
 			
 			//first segment, beta: 0->2pi
 			//second segment, beta: 0->-2pi
 			for (int i = 0; i < t_step[4]; i++)
 			{
 				a_seg2[0] = a_useg*3;
 				b_seg2[0] = genetraj(0, 2*M_PI, i, t_step[4]);
 				l_seg2[0] = length0*3;

 				a_seg2[1] = -a_useg*3;
 				b_seg2[1] = genetraj(0, -2*M_PI, i, t_step[4]);
 				l_seg2[1] = length0*3;

				for (int i = 0; i < 2; i++)
				{
					b_seg2[i] = constrainb(b_seg2[i]);
				}

 				for (int i = 0; i < 3; i++)
 				{
 					Command_ABL_demo.segment[i].A = a_seg2[0]/3;
 					Command_ABL_demo.segment[i].B = b_seg2[0];
 					Command_ABL_demo.segment[i].L = l_seg2[0]/3;
 				}
 				for (int i = 3; i < 6; i++)
 				{
 					Command_ABL_demo.segment[i].A = a_seg2[1]/3;
 					Command_ABL_demo.segment[i].B = b_seg2[1];
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
 			//first segment, alpha: 0.7*3->0
 			//second segment, alpha: -0.7*3->0
 			for (int i = 0; i < t_step[3]; i++)
 			{
 				a_seg2[0] = genetraj(a_useg*3, 0, i, t_step[3]);
 				b_seg2[0] = 0;
 				l_seg2[0] = length0*3;

 				a_seg2[1] = genetraj(-a_useg*3, 0, i, t_step[3]);
 				b_seg2[1] = 0;
 				l_seg2[1] = length0*3;

 				for (int i = 0; i < 3; i++)
 				{
 					Command_ABL_demo.segment[i].A = a_seg2[0]/3;
 					Command_ABL_demo.segment[i].B = b_seg2[0];
 					Command_ABL_demo.segment[i].L = l_seg2[0]/3;
 				}
 				for (int i = 3; i < 6; i++)
 				{
 					Command_ABL_demo.segment[i].A = a_seg2[1]/3;
 					Command_ABL_demo.segment[i].B = b_seg2[1];
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
 			//usleep(t_sleep[1]);
 			
 			flag = 3; 			
		}
		else if (flag == 3)//writeThreeSegments
		{
			//seg1-alpha
			/*for (int i = 0; i < t_step[11]; i++)
 			{
 				a_seg3[0] = genetraj(0, a_useg*2, i, t_step[11]);
 				b_seg3[0] = 0;
 				l_seg3[0] = length0*2;

 				a_seg3[1] = 0;
 				b_seg3[1] = 0;
 				l_seg3[1] = length0*2;

 				a_seg3[2] = 0;
 				b_seg3[2] = 0;
 				l_seg3[2] = length0*2;

 				for (int i = 0; i < 2; i++)
 				{
 					Command_ABL_demo.segment[i].A = a_seg3[0]/2;
 					Command_ABL_demo.segment[i].B = b_seg3[0];
 					Command_ABL_demo.segment[i].L = l_seg3[0]/2;
 				}
 				for (int i = 2; i < 4; i++)
 				{
 					Command_ABL_demo.segment[i].A = a_seg3[1]/2;
 					Command_ABL_demo.segment[i].B = b_seg3[1];
 					Command_ABL_demo.segment[i].L = l_seg3[1]/2;
 				}
 				for (int i = 4; i < 6; i++)
 				{
 					Command_ABL_demo.segment[i].A = a_seg3[2]/2;
 					Command_ABL_demo.segment[i].B = b_seg3[2];
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

			//seg1-beta
			for (int i = 0; i < t_step[12]; i++)
 			{
 				a_seg3[0] = a_useg*2;
 				b_seg3[0] = genetraj(0, 2*M_PI, i, t_step[12]);
 				l_seg3[0] = length0*2;

 				a_seg3[1] = 0;
 				b_seg3[1] = 0;
 				l_seg3[1] = length0*2;

 				a_seg3[2] = 0;
 				b_seg3[2] = 0;
 				l_seg3[2] = length0*2;

				for (int i = 0; i < 3; i++)
				{
					b_seg3[i] = constrainb(b_seg3[i]);
				}

 				for (int i = 0; i < 2; i++)
 				{
 					Command_ABL_demo.segment[i].A = a_seg3[0]/2;
 					Command_ABL_demo.segment[i].B = b_seg3[0];
 					Command_ABL_demo.segment[i].L = l_seg3[0]/2;
 				}
 				for (int i = 2; i < 4; i++)
 				{
 					Command_ABL_demo.segment[i].A = a_seg3[1]/2;
 					Command_ABL_demo.segment[i].B = b_seg3[1];
 					Command_ABL_demo.segment[i].L = l_seg3[1]/2;
 				}
 				for (int i = 4; i < 6; i++)
 				{
 					Command_ABL_demo.segment[i].A = a_seg3[2]/2;
 					Command_ABL_demo.segment[i].B = b_seg3[2];
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

			//seg1-reset	
			for (int i = 0; i < seg; i++)
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

			//seg2-alpha
			for (int i = 0; i < t_step[11]; i++)
 			{
 				a_seg3[0] = 0;
 				b_seg3[0] = 0;
 				l_seg3[0] = length0*2;
				
				a_seg3[1] = genetraj(0, a_useg*2, i, t_step[11]);
 				b_seg3[1] = 0;
 				l_seg3[1] = length0*2;

 				a_seg3[2] = 0;
 				b_seg3[2] = 0;
 				l_seg3[2] = length0*2;

 				for (int i = 0; i < 2; i++)
 				{
 					Command_ABL_demo.segment[i].A = a_seg3[0]/2;
 					Command_ABL_demo.segment[i].B = b_seg3[0];
 					Command_ABL_demo.segment[i].L = l_seg3[0]/2;
 				}
 				for (int i = 2; i < 4; i++)
 				{
 					Command_ABL_demo.segment[i].A = a_seg3[1]/2;
 					Command_ABL_demo.segment[i].B = b_seg3[1];
 					Command_ABL_demo.segment[i].L = l_seg3[1]/2;
 				}
 				for (int i = 4; i < 6; i++)
 				{
 					Command_ABL_demo.segment[i].A = a_seg3[2]/2;
 					Command_ABL_demo.segment[i].B = b_seg3[2];
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

			//seg2-beta
			for (int i = 0; i < t_step[12]; i++)
 			{
				a_seg3[0] = 0;
 				b_seg3[0] = 0;
 				l_seg3[0] = length0*2;
 				
				a_seg3[1] = a_useg*2;
 				b_seg3[1] = genetraj(0, 2*M_PI, i, t_step[12]);
 				l_seg3[1] = length0*2;

			
 				a_seg3[2] = 0;
 				b_seg3[2] = 0;
 				l_seg3[2] = length0*2;

				for (int i = 0; i < 3; i++)
				{
					b_seg3[i] = constrainb(b_seg3[i]);
				}

 				for (int i = 0; i < 2; i++)
 				{
 					Command_ABL_demo.segment[i].A = a_seg3[0]/2;
 					Command_ABL_demo.segment[i].B = b_seg3[0];
 					Command_ABL_demo.segment[i].L = l_seg3[0]/2;
 				}
 				for (int i = 2; i < 4; i++)
 				{
 					Command_ABL_demo.segment[i].A = a_seg3[1]/2;
 					Command_ABL_demo.segment[i].B = b_seg3[1];
 					Command_ABL_demo.segment[i].L = l_seg3[1]/2;
 				}
 				for (int i = 4; i < 6; i++)
 				{
 					Command_ABL_demo.segment[i].A = a_seg3[2]/2;
 					Command_ABL_demo.segment[i].B = b_seg3[2];
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

			//seg2-reset	
			for (int i = 0; i < seg; i++)
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

			//seg3-alpha
			for (int i = 0; i < t_step[11]; i++)
 			{
 				a_seg3[0] = 0;
 				b_seg3[0] = 0;
 				l_seg3[0] = length0*2;

				a_seg3[1] = 0;
 				b_seg3[1] = 0;
 				l_seg3[1] = length0*2;
				
				a_seg3[2] = genetraj(0, a_useg*2, i, t_step[11]);
 				b_seg3[2] = 0;
 				l_seg3[2] = length0*2;

 				for (int i = 0; i < 2; i++)
 				{
 					Command_ABL_demo.segment[i].A = a_seg3[0]/2;
 					Command_ABL_demo.segment[i].B = b_seg3[0];
 					Command_ABL_demo.segment[i].L = l_seg3[0]/2;
 				}
 				for (int i = 2; i < 4; i++)
 				{
 					Command_ABL_demo.segment[i].A = a_seg3[1]/2;
 					Command_ABL_demo.segment[i].B = b_seg3[1];
 					Command_ABL_demo.segment[i].L = l_seg3[1]/2;
 				}
 				for (int i = 4; i < 6; i++)
 				{
 					Command_ABL_demo.segment[i].A = a_seg3[2]/2;
 					Command_ABL_demo.segment[i].B = b_seg3[2];
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

			//seg3-beta
			for (int i = 0; i < t_step[12]; i++)
 			{
				a_seg3[0] = 0;
 				b_seg3[0] = 0;
 				l_seg3[0] = length0*2;

				a_seg3[1] = 0;
 				b_seg3[1] = 0;
 				l_seg3[1] = length0*2;
 				
				a_seg3[2] = a_useg*2;
 				b_seg3[2] = genetraj(0, 2*M_PI, i, t_step[12]);
 				l_seg3[2] = length0*2;

				for (int i = 0; i < 3; i++)
				{
					b_seg3[i] = constrainb(b_seg3[i]);
				}

 				for (int i = 0; i < 2; i++)
 				{
 					Command_ABL_demo.segment[i].A = a_seg3[0]/2;
 					Command_ABL_demo.segment[i].B = b_seg3[0];
 					Command_ABL_demo.segment[i].L = l_seg3[0]/2;
 				}
 				for (int i = 2; i < 4; i++)
 				{
 					Command_ABL_demo.segment[i].A = a_seg3[1]/2;
 					Command_ABL_demo.segment[i].B = b_seg3[1];
 					Command_ABL_demo.segment[i].L = l_seg3[1]/2;
 				}
 				for (int i = 4; i < 6; i++)
 				{
 					Command_ABL_demo.segment[i].A = a_seg3[2]/2;
 					Command_ABL_demo.segment[i].B = b_seg3[2];
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

			//seg3-reset	
			for (int i = 0; i < seg; i++)
 			{
 				Command_ABL_demo.segment[i].A = 0;
 				Command_ABL_demo.segment[i].B = 0;
 				Command_ABL_demo.segment[i].L = length0;
 			}
				
 			pub1.publish(Command_ABL_demo);
 			for(int i = 0; i < 6; i++)
 			{
 				printf("A[%d]: %f, B[%d]: %f, L[%d]: %f\r\n", i, a_seg6[i], i, b_seg6[i], i, l_seg6[i]);
 			}*/

			//first segment alpha: 0->0.7*2
			//second segment length: l0->lmax
			//third segment alpha: 0->-0.7*2
			for (int i = 0; i < t_step[5]; i++)
 			{
 				a_seg3[0] = genetraj(-a_useg*2, a_useg*2, i, t_step[5]);
 				b_seg3[0] = 0;
 				l_seg3[0] = length0*2;

 				a_seg3[1] = genetraj(-a_useg*2, 0, i, t_step[5]);
 				b_seg3[1] = 0;
 				l_seg3[1] = genetraj(length0*2, lengthmax*2, i, t_step[5]);

 				a_seg3[2] = genetraj(-a_useg*2, -a_useg*2, i, t_step[5]);
 				b_seg3[2] = 0;
 				l_seg3[2] = length0*2;

 				for (int i = 0; i < 2; i++)
 				{
 					Command_ABL_demo.segment[i].A = a_seg3[0]/2;
 					Command_ABL_demo.segment[i].B = b_seg3[0];
 					Command_ABL_demo.segment[i].L = l_seg3[0]/2;
 				}
 				for (int i = 2; i < 4; i++)
 				{
 					Command_ABL_demo.segment[i].A = a_seg3[1]/2;
 					Command_ABL_demo.segment[i].B = b_seg3[1];
 					Command_ABL_demo.segment[i].L = l_seg3[1]/2;
 				}
 				for (int i = 4; i < 6; i++)
 				{
 					Command_ABL_demo.segment[i].A = a_seg3[2]/2;
 					Command_ABL_demo.segment[i].B = b_seg3[2];
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

 			//first segment alpha: 0.7*2
			//second segment length: lmax->lmin
			//third segment alpha: -0.7*2
			for (int i = 0; i < t_step[6]; i++)
 			{
 				a_seg3[0] = a_useg*2;
 				b_seg3[0] = 0;
 				l_seg3[0] = length0*2;

 				a_seg3[1] = 0;
 				b_seg3[1] = 0;
 				l_seg3[1] = genetraj(lengthmax*2, lengthmin*2, i, t_step[6]);

 				a_seg3[2] = -a_useg*2;
 				b_seg3[2] = 0;
 				l_seg3[2] = length0*2;

 				for (int i = 0; i < 2; i++)
 				{
 					Command_ABL_demo.segment[i].A = a_seg3[0]/2;
 					Command_ABL_demo.segment[i].B = b_seg3[0];
 					Command_ABL_demo.segment[i].L = l_seg3[0]/2;
 				}
 				for (int i = 2; i < 4; i++)
 				{
 					Command_ABL_demo.segment[i].A = a_seg3[1]/2;
 					Command_ABL_demo.segment[i].B = b_seg3[1];
 					Command_ABL_demo.segment[i].L = l_seg3[1]/2;
 				}
 				for (int i = 4; i < 6; i++)
 				{
 					Command_ABL_demo.segment[i].A = a_seg3[2]/2;
 					Command_ABL_demo.segment[i].B = b_seg3[2];
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

 			//first segment alpha: 0.7*2, beta: 0->pi
			//second segment length: lmin->lmax
			//third segment alpha: -0.7*2, beta: 0->-pi
			for (int i = 0; i < t_step[7]; i++)
 			{
 				a_seg3[0] = a_useg*2;
 				b_seg3[0] = genetraj(0, M_PI, i, t_step[7]);
 				l_seg3[0] = length0*2;

 				a_seg3[1] = 0;
 				b_seg3[1] = 0;
 				l_seg3[1] = genetraj(lengthmin*2, lengthmax*2, i, t_step[7]);

 				a_seg3[2] = -a_useg*2;
 				b_seg3[2] = genetraj(0, -M_PI, i, t_step[7]);
 				l_seg3[2] = length0*2;

 				for (int i = 0; i < 2; i++)
 				{
 					Command_ABL_demo.segment[i].A = a_seg3[0]/2;
 					Command_ABL_demo.segment[i].B = b_seg3[0];
 					Command_ABL_demo.segment[i].L = l_seg3[0]/2;
 				}
 				for (int i = 2; i < 4; i++)
 				{
 					Command_ABL_demo.segment[i].A = a_seg3[1]/2;
 					Command_ABL_demo.segment[i].B = b_seg3[1];
 					Command_ABL_demo.segment[i].L = l_seg3[1]/2;
 				}
 				for (int i = 4; i < 6; i++)
 				{
 					Command_ABL_demo.segment[i].A = a_seg3[2]/2;
 					Command_ABL_demo.segment[i].B = b_seg3[2];
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

			//first segment alpha: 0.7*2, beta: -pi->0
			//second segment length: lmax->lmin
			//third segment alpha: -0.7*2, beta: pi->0
			for (int i = 0; i < t_step[7]; i++)
 			{
 				a_seg3[0] = a_useg*2;
 				b_seg3[0] = genetraj(-M_PI, 0, i, t_step[7]);
 				l_seg3[0] = length0*2;

 				a_seg3[1] = 0;
 				b_seg3[1] = 0;
 				l_seg3[1] = genetraj(lengthmax*2, lengthmin*2, i, t_step[7]);

 				a_seg3[2] = -a_useg*2;
 				b_seg3[2] = genetraj(M_PI, 0, i, t_step[7]);
 				l_seg3[2] = length0*2;

 				for (int i = 0; i < 2; i++)
 				{
 					Command_ABL_demo.segment[i].A = a_seg3[0]/2;
 					Command_ABL_demo.segment[i].B = b_seg3[0];
 					Command_ABL_demo.segment[i].L = l_seg3[0]/2;
 				}
 				for (int i = 2; i < 4; i++)
 				{
 					Command_ABL_demo.segment[i].A = a_seg3[1]/2;
 					Command_ABL_demo.segment[i].B = b_seg3[1];
 					Command_ABL_demo.segment[i].L = l_seg3[1]/2;
 				}
 				for (int i = 4; i < 6; i++)
 				{
 					Command_ABL_demo.segment[i].A = a_seg3[2]/2;
 					Command_ABL_demo.segment[i].B = b_seg3[2];
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
 			//first segment alpha: 0.7*2->0
			//second segment length: lmin-l0
			//third segment alpha: -0.7*2->0
			for (int i = 0; i < t_step[8]; i++)
 			{
 				a_seg3[0] = genetraj(a_useg*2, 0, i, t_step[8]);
 				b_seg3[0] = 0;
 				l_seg3[0] = length0*2;

 				a_seg3[1] = 0;
 				b_seg3[1] = 0;
 				l_seg3[1] = genetraj(lengthmin*2, length0*2, i, t_step[8]);

 				a_seg3[2] = genetraj(-a_useg*2, 0, i, t_step[8]);
 				b_seg3[2] = 0;
 				l_seg3[2] = length0*2;

 				for (int i = 0; i < 2; i++)
 				{
 					Command_ABL_demo.segment[i].A = a_seg3[0]/2;
 					Command_ABL_demo.segment[i].B = b_seg3[0];
 					Command_ABL_demo.segment[i].L = l_seg3[0]/2;
 				}
 				for (int i = 2; i < 4; i++)
 				{
 					Command_ABL_demo.segment[i].A = a_seg3[1]/2;
 					Command_ABL_demo.segment[i].B = b_seg3[1];
 					Command_ABL_demo.segment[i].L = l_seg3[1]/2;
 				}
 				for (int i = 4; i < 6; i++)
 				{
 					Command_ABL_demo.segment[i].A = a_seg3[2]/2;
 					Command_ABL_demo.segment[i].B = b_seg3[2];
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
 			//usleep(t_sleep[1]);

 			flag = 6;
		}
		else if (flag == 6)//writeSixSegments
		{
			/************************************************seg1*******************************************/
			//seg1, alpha->a_useg
			/*for (int i = 0; i < t_step[11]; i++)
 			{ 
 				a_seg6[0] = genetraj(0, a_useg, i, t_step[11]);
 				b_seg6[0] = 0;			
 				l_seg6[0] = length0;
				
				for(int i = 0; i < 5; i++)
				{
					a_seg6[i+1] = 0;
					b_seg6[i+1] = 0;
					l_seg6[i+1] = length0;
				} 
 				
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

			//seg1, beta 0->2pi
			for (int i = 0; i < t_step[12]; i++)
 			{ 
 				a_seg6[0] = a_useg;
 				b_seg6[0] = genetraj(0, 2*M_PI, i, t_step[12]);			
 				l_seg6[0] = length0;
				
				for(int i = 0; i < 5; i++)
				{
					a_seg6[i+1] = 0;
					b_seg6[i+1] = 0;
					l_seg6[i+1] = length0;
				} 
 				
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
			
			//seg1, reset				
 			for (int i = 0; i < seg; i++)
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
 			//usleep(t_sleep[2]); 						
			/*******************************************************************************************/
						

			/************************************************seg2*******************************************/
			//seg2, alpha->a_useg
			/*for (int i = 0; i < t_step[11]; i++)
 			{ 
 				a_seg6[0] = 0;
 				b_seg6[0] = 0;			
 				l_seg6[0] = length0;

				a_seg6[1] = genetraj(0, a_useg, i, t_step[11]);
 				b_seg6[1] = 0;			
 				l_seg6[1] = length0;

				for(int i = 0; i < 4; i++)
				{
					a_seg6[i+2] = 0;
					b_seg6[i+2] = 0;
					l_seg6[i+2] = length0;
				} 
 				
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

			//seg2, beta 0->2pi
			for (int i = 0; i < t_step[12]; i++)
 			{ 
 				a_seg6[0] = 0;
 				b_seg6[0] = 0;			
 				l_seg6[0] = length0;

				a_seg6[1] = a_useg;
 				b_seg6[1] = genetraj(0, 2*M_PI, i, t_step[12]);			
 				l_seg6[1] = length0;
				
				for(int i = 0; i < 4; i++)
				{
					a_seg6[i+2] = 0;
					b_seg6[i+2] = 0;
					l_seg6[i+2] = length0;
				} 
 				
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
			
			//seg2, reset				
 			for (int i = 0; i < seg; i++)
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
 			//usleep(t_sleep[2]); 						
			/*******************************************************************************************/

			/************************************************seg3*******************************************/
			//seg3, alpha->a_useg
			/*for (int i = 0; i < t_step[11]; i++)
 			{ 
 				a_seg6[0] = 0;
 				b_seg6[0] = 0;			
 				l_seg6[0] = length0;

				a_seg6[1] = 0;
 				b_seg6[1] = 0;			
 				l_seg6[1] = length0;

				a_seg6[2] = genetraj(0, a_useg, i, t_step[11]);
 				b_seg6[2] = 0;			
 				l_seg6[2] = length0;

				for(int i = 0; i < 3; i++)
				{
					a_seg6[i+3] = 0;
					b_seg6[i+3] = 0;
					l_seg6[i+3] = length0;
				} 
 				
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

			//seg3, beta 0->2pi
			for (int i = 0; i < t_step[12]; i++)
 			{ 
 				a_seg6[0] = 0;
 				b_seg6[0] = 0;			
 				l_seg6[0] = length0;

				a_seg6[1] = 0;
 				b_seg6[1] = 0;		
 				l_seg6[1] = length0;
				
				a_seg6[2] = a_useg;
 				b_seg6[2] = genetraj(0, 2*M_PI, i, t_step[12]);			
 				l_seg6[2] = length0;

				for(int i = 0; i < 3; i++)
				{
					a_seg6[i+3] = 0;
					b_seg6[i+3] = 0;
					l_seg6[i+3] = length0;
				} 
 				
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
			
			//seg3, reset				
 			for (int i = 0; i < seg; i++)
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
 			//usleep(t_sleep[2]); 						
			/*******************************************************************************************/

			/************************************************seg4*******************************************/
			//seg4, alpha->a_useg
			/*for (int i = 0; i < t_step[11]; i++)
 			{ 
 				a_seg6[0] = 0;
 				b_seg6[0] = 0;			
 				l_seg6[0] = length0;

				a_seg6[1] = 0;
 				b_seg6[1] = 0;			
 				l_seg6[1] = length0;

				a_seg6[2] = 0;
 				b_seg6[2] = 0;			
 				l_seg6[2] = length0;

				a_seg6[3] = genetraj(0, a_useg, i, t_step[11]);
 				b_seg6[3] = 0;			
 				l_seg6[3] = length0;

				for(int i = 0; i < 2; i++)
				{
					a_seg6[i+4] = 0;
					b_seg6[i+4] = 0;
					l_seg6[i+4] = length0;
				} 
 				
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

			//seg4, beta 0->2pi
			for (int i = 0; i < t_step[12]; i++)
 			{ 
 				a_seg6[0] = 0;
 				b_seg6[0] = 0;			
 				l_seg6[0] = length0;

				a_seg6[1] = 0;
 				b_seg6[1] = 0;		
 				l_seg6[1] = length0;
				
				a_seg6[2] = 0;
 				b_seg6[2] = 0;			
 				l_seg6[2] = length0;

				a_seg6[3] = a_useg;
 				b_seg6[3] = genetraj(0, 2*M_PI, i, t_step[12]);			
 				l_seg6[3] = length0;

				for(int i = 0; i < 2; i++)
				{
					a_seg6[i+4] = 0;
					b_seg6[i+4] = 0;
					l_seg6[i+4] = length0;
				} 
 				
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
			
			//seg4, reset				
 			for (int i = 0; i < seg; i++)
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
 			//usleep(t_sleep[2]); 						
			/*******************************************************************************************/

			/************************************************seg5*******************************************/
			//seg5, alpha->a_useg
			/*for (int i = 0; i < t_step[11]; i++)
 			{ 
				a_seg6[4] = genetraj(0, a_useg, i, t_step[11]);
 				b_seg6[4] = 0;			
 				l_seg6[4] = length0;

				a_seg6[5] = 0;
 				b_seg6[5] = 0;			
 				l_seg6[5] = length0;

				for(int i = 0; i < 4; i++)
				{
					a_seg6[i] = 0;
					b_seg6[i] = 0;
					l_seg6[i] = length0;
				} 
 				
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

			//seg5, beta 0->2pi
			for (int i = 0; i < t_step[12]; i++)
 			{ 
				a_seg6[4] = a_useg;
 				b_seg6[4] = genetraj(0, 2*M_PI, i, t_step[12]);			
 				l_seg6[4] = length0;

				a_seg6[5] = 0;
 				b_seg6[5] = 0;			
 				l_seg6[5] = length0;

				for(int i = 0; i < 4; i++)
				{
					a_seg6[i] = 0;
					b_seg6[i] = 0;
					l_seg6[i] = length0;
				} 
 				
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
			
			//seg5, reset				
 			for (int i = 0; i < seg; i++)
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
 			//usleep(t_sleep[2]); 						
			/*******************************************************************************************/

			/************************************************seg6*******************************************/
			//seg6, alpha->a_useg
			/*for (int i = 0; i < t_step[11]; i++)
 			{ 
				a_seg6[5] = genetraj(0, a_useg, i, t_step[11]);
 				b_seg6[5] = 0;			
 				l_seg6[5] = length0;

				for(int i = 0; i < 5; i++)
				{
					a_seg6[i] = 0;
					b_seg6[i] = 0;
					l_seg6[i] = length0;
				} 
 				
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

			//seg6, beta 0->2pi
			for (int i = 0; i < t_step[12]; i++)
 			{ 
				a_seg6[5] = a_useg;
 				b_seg6[5] = genetraj(0, 2*M_PI, i, t_step[12]);			
 				l_seg6[5] = length0;

				for(int i = 0; i < 5; i++)
				{
					a_seg6[i] = 0;
					b_seg6[i] = 0;
					l_seg6[i] = length0;
				} 
 				
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
			
			//seg6, reset				
 			for (int i = 0; i < seg; i++)
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
 			//usleep(t_sleep[2]); 						
			/*******************************************************************************************/


			//alpha
			for (int i = 0; i < t_step[9]; i++)
 			{
 				a_seg6[0] = genetraj(0, a_useg, i, t_step[9]);
 				b_seg6[0] = 0;
 				l_seg6[0] = length0;

 				a_seg6[1] = genetraj(0, -a_useg, i, t_step[9]);
 				b_seg6[1] = 0;
 				l_seg6[1] = length0;

 				a_seg6[2] = genetraj(0, a_useg, i, t_step[9]);
 				b_seg6[2] = 0;
 				l_seg6[2] = length0;

 				a_seg6[3] = genetraj(0, -a_useg, i, t_step[9]);
 				b_seg6[3] = 0;
 				l_seg6[3] = length0;

 				a_seg6[4] = genetraj(0, a_useg, i, t_step[9]);
 				b_seg6[4] = 0;
 				l_seg6[4] = length0;

 				a_seg6[5] = genetraj(0, -a_useg, i, t_step[9]);
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
 			for (int i = 0; i < t_step[10]; i++)
 			{
 				a_seg6[0] = a_useg;
 				b_seg6[0] = genetraj(0, M_PI/3, i, t_step[10]);				
 				l_seg6[0] = length0;

 				a_seg6[1] = -a_useg;
 				b_seg6[1] = genetraj(0, 2*M_PI/3, i, t_step[10]);				
 				l_seg6[1] = length0;

 				a_seg6[2] = a_useg;
 				b_seg6[2] = genetraj(0, 3*M_PI/3, i, t_step[10]);				
 				l_seg6[2] = length0;

 				a_seg6[3] = -a_useg;
 				b_seg6[3] = genetraj(0, 4*M_PI/3, i, t_step[10]);				
 				l_seg6[3] = length0;

 				a_seg6[4] = a_useg;
 				b_seg6[4] = genetraj(0, 5*M_PI/3, i, t_step[10]);				
 				l_seg6[4] = length0;

 				a_seg6[5] = -a_useg;
 				b_seg6[5] = genetraj(0, 6*M_PI/3, i, t_step[10]);				
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
 			for (int i = 0; i < t_step[10]; i++)
 			{ 
 				a_seg6[0] = a_useg;
 				b_seg6[0] = genetraj(M_PI/3, 0, i, t_step[10]);				
 				l_seg6[0] = length0;

 				a_seg6[1] = -a_useg;
 				b_seg6[1] = genetraj(2*M_PI/3, 0, i, t_step[10]);				
 				l_seg6[1] = length0;

 				a_seg6[2] = a_useg;
 				b_seg6[2] = genetraj(3*M_PI/3, 0, i, t_step[10]);				
 				l_seg6[2] = length0;

 				a_seg6[3] = -a_useg;
 				b_seg6[3] = genetraj(4*M_PI/3, 0, i, t_step[10]);				
 				l_seg6[3] = length0;

 				a_seg6[4] = a_useg;
 				b_seg6[4] = genetraj(5*M_PI/3, 0, i, t_step[10]);				
 				l_seg6[4] = length0;

 				a_seg6[5] = -a_useg;
 				b_seg6[5] = genetraj(6*M_PI/3, 0, i, t_step[10]);				
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
 			for (int i = 0; i < t_step[9]; i++)
 			{
 				a_seg6[0] = genetraj(a_useg, 0, i, t_step[9]);
 				b_seg6[0] = 0;
 				l_seg6[0] = length0;

 				a_seg6[1] = genetraj(-a_useg, 0, i, t_step[9]);
 				b_seg6[1] = 0;
 				l_seg6[1] = length0;

 				a_seg6[2] = genetraj(a_useg, 0, i, t_step[9]);
 				b_seg6[2] = 0;
 				l_seg6[2] = length0;

 				a_seg6[3] = genetraj(-a_useg, 0, i, t_step[9]);
 				b_seg6[3] = 0;
 				l_seg6[3] = length0;

 				a_seg6[4] = genetraj(a_useg, 0, i, t_step[9]);
 				b_seg6[4] = 0;
 				l_seg6[4] = length0;

 				a_seg6[5] = genetraj(-a_useg, 0, i, t_step[9]);
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
			
			//pause for a while
 			//usleep(t_sleep[1]);

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
