#include <ros/ros.h>
#include <geometry_msgs/Twist.h> 
#include <geometry_msgs/Pose.h> 
#include <sensor_msgs/Joy.h>
#include <math.h>
#include <time.h>
#include <iostream>
#include <vector>
#include <stdio.h>
#include <stdlib.h>

#include "origarm_ros/Command_ABL.h"
#include "origarm_ros/Seg_ABL.h"
#include "origarm_ros/SegOpening.h"
#include "origarm_ros/Command_Pre_Open.h"
#include "origarm_ros/Command_Position.h"
#include "origarm_ros/keynumber.h"
#include "origarm_ros/modenumber.h"
#include "origarm_ros/segnumber.h"

#include "myData.h"
#include "myFunction.h"
#include <Eigen/Eigen>
#include <Eigen/Dense>
#include <Eigen/Geometry>
using Eigen::Matrix4f;
using Eigen::Vector3f;

using namespace std;

//keyboard mapping
int key_no[10];

//joystick mapping
float joyLx;
float joyLy;
float joyRx;
float joyRy;
float joyLT;
float joyRT;
int joyLB;		//mode[5]
int joyRB;	    //mode[4]	
int joyX;		//mode[2]
int joyY;		//mode[3]
int joyA;       //mode[0]
int joyB;		//mode[1]
int joyCrossY;	
int joyCrossX;	
int joyUp;		
int joyDown;	
int joyLeft;	
int joyRight;	

int last_joyRB;
int last_joyLB;
int last_joyX;
int last_joyY;
int last_joyA;
int last_joyB;
int last_joyUp;		
int last_joyDown;	
int last_joyLeft;	
int last_joyRight;	

int enable;  
int disable; 
int last_enable;
int last_disable;
int status;

//Write ABL
int segNumber;
float alpha;
float beta;
float length = 0.055;
float segAlpha_[3];
float segBeta_[3];
float segLength_[3];
float segAlpha[9];
float segBeta[9];
float segLength[9];

float a_scale = 0.001;
float b_scale = 0.001;
float l_scale = 0.00001;

float l_max =  0.08;
float l_min =  0.03;
float a_max =  M_PI/2;
float a_min = -M_PI/2;
float b_max =  M_PI;
float b_min = -M_PI;

//write Opening
float belloConfigurationR = 0.06;
float Rmax = 1.0;

float rawAngle;
float rawAmplitude;
float rawAmplitudeMax;
float angleCommand;
float amplitudeCommand;
float bellowProjection[6];
float bellowConfigurationPx[6];
float bellowConfigurationPy[6];
float openingBase;
float OpeningResult[6];

//Write XYZ unit:m
float x = 0;
float y = 0;
float z = 0.055;
float segx_ = 0;
float segy_ = 0;
float segz_ = 0.055;
float segx = 0;
float segy = 0;
float segz = 0.055;

float x_scale = 0.00001;
float y_scale = 0.00001;
float z_scale = 0.0001;

/*float x_max =  0.01;
float x_min = -0.01;
float y_max =  0.01;
float y_min = -0.01;*/

float x_max =  0.06;
float x_min = -0.06;
float y_max =  0.06;
float y_min = -0.06;

float z_max =  0.08;
float z_min =  0.01;
float z_max9 = 0.055*9+(z_max-0.055)*9;
float z_min9 = 0.055*9-(0.055-z_min)*9;

//Internal parameters
//ABL calculated by XYZ
float alphad;
float betad;
float lengthd;
float segAlphad_[3];
float segBetad_[3];
float segLengthd_[3];
float segAlphad[9];
float segBetad[9];
float segLengthd[9];

//control mode
//mode[0]: 1 abl; mode[1]: 3 abl; mode[2]: 9 abl; mode[3]: 1 xyz; mode[4]: 3 xyz; mode[5]: 9 xyz
//mode[0]: joyA;  mode[1]: joyB;  mode[2]: joyX;  mode[3]: joyY;  mode[4]: joyRB; mode[5]: joyLB
int mode;

//keyboard callback
void keyCallback(const origarm_ros::keynumber& key)
{
	for (int i = 0; i < 10; i++)
	{
		key_no[i] = key.KEY_CODE[i];
		if (key_no[i] > 0)
		{
			segNumber = i;
		}
	}
}

//joystick callback
void joyCallback(const sensor_msgs::Joy::ConstPtr& joy)
{
	//joystick mapping
	joyLx = joy->axes[0];
	joyLy = joy->axes[1];
	joyRx = joy->axes[3];
	joyRy = joy->axes[4];
	joyLT = joy->axes[2];
	joyRT = joy->axes[5];	
	 
 	//enable = joy->buttons[0];
	disable = joy->buttons[6];//back

	joyA = joy->buttons[0];
	joyB = joy->buttons[1];
	joyX = joy->buttons[2];
	joyY = joy->buttons[3];
	joyLB = joy->buttons[4];
	joyRB = joy->buttons[5];

	joyCrossX = joy->axes[6];
	joyCrossY = joy->axes[7];

	if (joyCrossX == 1)
	{
		joyLeft = 1;
	}
	else if (joyCrossX == -1)
	{
		joyRight = 1;
	}
	else
	{
		joyLeft = 0;
		joyRight = 0;
	}
	
	if (joyCrossY == 1)
	{
		joyUp = 1;
	}
	else if (joyCrossY == -1)
	{
		joyDown = 1;
	}
	else
	{
		joyUp = 0;
		joyDown = 0;
	}

	if (joyA == 1 && last_joyA == 0)
	{
		mode = 0;
	}
	else if (joyB == 1 && last_joyB == 0)
	{
		mode = 1;
	}
	else if (joyY == 1 && last_joyY == 0)
	{
		mode = 3;
	}
	else if (joyX == 1 && last_joyX == 0)
	{
		mode = 2;
	}
	else if (joyRB == 1 && last_joyRB == 0)
	{
		mode = 4;
	}
	else if (joyLB == 1 && last_joyLB == 0)
	{
		mode = 5;
	}

	//only when joyRT && joyLT pressed together, joystick starts to control
	if (joyLT == -1 && joyRT == -1)
	{
		enable = 1;
	}
	else
	{
		enable = 0;
	}
			
	if (enable == 1 && last_enable == 0)
 	{
		status = 1;
 	}
	else if (disable == 1 && last_disable == 0)
	{
		status = 0;
	}

	last_enable = enable; 
  	last_disable = disable;	

  	last_joyA = joyA;
  	last_joyB = joyB;
  	last_joyX = joyX;
  	last_joyY = joyY;
  	last_joyUp = joyUp;
  	last_joyDown = joyDown;
  	last_joyLeft = joyLeft;
  	last_joyRight = joyRight;
  	last_joyRB = joyRB;
  	last_joyLB = joyLB;
}

//ABL from ik
void ABLCallback(const origarm_ros::Command_ABL& msg)
{
	alphad  = msg.segment[0].A;
	betad   = msg.segment[0].B;
	lengthd = msg.segment[0].L;

	for (int i = 0; i < 3; i++)
	{
		segAlphad_[i]  = msg.segment[i].A;
		segBetad_[i]   = msg.segment[i].B;
		segLengthd_[i] = msg.segment[i].L;
	}

	for (int i = 0; i < 9; i++)
	{
		segAlphad[i]  = msg.segment[i].A;
		segBetad[i]   = msg.segment[i].B;
		segLengthd[i] = msg.segment[i].L;
	}
}

//Joystick->ABL (joyLy->alpha, joyRx->beta, joyLT & joyRT->length)
void writeABL1(int joystickFLag)
{
	if (joystickFLag == 1)
	{
		if (joyLy > 0.05)
		{			
			alpha = alpha + a_scale;
		}
		else if (joyLy < -0.05)
		{
			alpha = alpha - a_scale;
		}

		if (joyRx > 0.05)
		{
			if (joyRx > 0.5)
			{
				beta = beta + 2*b_scale;
			}
			else
			{
				beta = beta + b_scale;
			}
		}
		else if (joyRx < -0.05)
		{
			if (joyRx < -0.5)
			{
				beta = beta - 2*b_scale;
			}
			else
			{
				beta = beta - b_scale;
			}
		}

		if (joyLT != 1 && joyRT != 1)
		{

		}
		else if (abs(joyLT-1) > 0.05)
		{
			if (abs(joyLT-1) > 1)
			{
				length = length + 2*l_scale;
			}
			else
			{
				length = length + l_scale;
			}
		}
		else if (abs(joyRT-1) > 0.05)
		{
			if (abs(joyRT-1) > 0.05)
			{
				if (abs(joyRT-1) > 1)
				{
					length = length - 2*l_scale;
				}
				else
				{
					length = length - l_scale;
				}
			}
		}
	}
	else if (joystickFLag == 0)  //calculate  ABL from xyz
	{
		alpha  = alphad;
		beta   = betad;
		length = lengthd;
	}

	alpha  = CONSTRAIN(alpha, a_min, a_max);
	beta   = CONSTRAIN(beta, b_min, b_max);
	length = CONSTRAIN(length, l_min, l_max);
}

void writeABL3(int joystickFLag)
{
	if (joystickFLag == 1)
	{
		if (segNumber < 3)
		{
			if (joyLy > 0.05)
			{			
				segAlpha_[segNumber] = segAlpha_[segNumber] + a_scale;
			}
			else if (joyLy < -0.05)
			{
				segAlpha_[segNumber] = segAlpha_[segNumber] - a_scale;
			}

			if (joyRx > 0.05)
			{
				if (joyRx > 0.5)
				{
					segBeta_[segNumber] = segBeta_[segNumber] + 2*b_scale;
				}
				else
				{
					segBeta_[segNumber] = segBeta_[segNumber] + b_scale;
				}
			}
			else if (joyRx < -0.05)
			{
				if (joyRx < -0.5)
				{
					segBeta_[segNumber] = segBeta_[segNumber] - 2*b_scale;
				}
				else
				{
					segBeta_[segNumber] = segBeta_[segNumber] - b_scale;
				}
			}

			if (joyLT != 1 && joyRT != 1)
			{

			}
			else if (abs(joyLT-1) > 0.05)
			{
				if (abs(joyLT-1) > 1)
				{
					segLength_[segNumber] = segLength_[segNumber] + 2*l_scale;
				}
				else
				{
					segLength_[segNumber] = segLength_[segNumber] + l_scale;
				}
			}
			else if (abs(joyRT-1) > 0.05)
			{
				if (abs(joyRT-1) > 0.05)
				{
					if (abs(joyRT-1) > 1)
					{
						segLength_[segNumber] = segLength_[segNumber] - 2*l_scale;
					}
					else
					{
						segLength_[segNumber] = segLength_[segNumber] - l_scale;
					}
				}
			}
		}		
	}
	else if (joystickFLag == 0)
	{
		for (int i = 0; i < 3; i++)
		{
			segAlpha_[i]  = segAlphad_[i];
			segBeta_[i]   = segBetad_[i];
			segLength_[i] = segLengthd_[i];
		}		
	}

	for (int i = 0; i < 3; i++)
	{
		segAlpha_[i]  = CONSTRAIN(segAlpha_[i], a_min, a_max);
		segBeta_[i]   = CONSTRAIN(segBeta_[i], b_min, b_max);
		segLength_[i] = CONSTRAIN(segLength_[i], 0.055*3-(0.055-l_min)*3, 0.055*3+(l_max-0.055)*3);
	}
}

void writeABL9(int joystickFLag)
{
	if (joystickFLag == 1)// calculated from joystick
	{
		if (joyLy > 0.05)
		{			
			segAlpha[segNumber] = segAlpha[segNumber] + a_scale;
		}
		else if (joyLy < -0.05)
		{
			segAlpha[segNumber] = segAlpha[segNumber] - a_scale;
		}

		if (joyRx > 0.05)
		{
			if (joyRx > 0.5)
			{
				segBeta[segNumber] = segBeta[segNumber] + 2*b_scale;
			}
			else
			{
				segBeta[segNumber] = segBeta[segNumber] + b_scale;
			}
		}
		else if (joyRx < -0.05)
		{
			if (joyRx < -0.5)
			{
				segBeta[segNumber] = segBeta[segNumber] - 2*b_scale;
			}
			else
			{
				segBeta[segNumber] = segBeta[segNumber] - b_scale;
			}
		}

		if (joyLT != 1 && joyRT != 1)
		{

		}
		else if (abs(joyLT-1) > 0.05)
		{
			if (abs(joyLT-1) > 1)
			{
				segLength[segNumber] = segLength[segNumber] + 2*l_scale;
			}
			else
			{
				segLength[segNumber] = segLength[segNumber] + l_scale;
			}
		}
		else if (abs(joyRT-1) > 0.05)
		{
			if (abs(joyRT-1) > 0.05)
			{
				if (abs(joyRT-1) > 1)
				{
					segLength[segNumber] = segLength[segNumber] - 2*l_scale;
				}
				else
				{
					segLength[segNumber] = segLength[segNumber] - l_scale;
				}
			}
		}
	}
	else if (joystickFLag == 0) //calcuated from XYZ_9 control mode
	{
		for (int i = 0; i < 9; i++)
		{
			segAlpha[i]  = segAlphad[i];
			segBeta[i]   = segBetad[i];
			segLength[i] = segLengthd[i];
		}		
	}
	else if (joystickFLag == 2) //calculated from ABL_3 control mode
	{
		for (int i = 0; i < 3; i++)
		{
			segAlpha[i] = segAlpha_[0]/3;
			segBeta[i]  = segBeta_[0];
			segLength[i]= segLength_[0]/3;
		}
		for (int i = 3; i < 6; i++)
		{
			segAlpha[i] = segAlpha_[1]/3;
			segBeta[i]  = segBeta_[1];
			segLength[i]= segLength_[1]/3;
		}
		for (int i = 6; i < 9; i++)
		{
			segAlpha[i] = segAlpha_[2]/3;
			segBeta[i]  = segBeta_[2];
			segLength[i]= segLength_[2]/3;
		}
	}

	for (int i = 0; i < 9; i++)
	{
		segAlpha[i]  = CONSTRAIN(segAlpha[i], a_min, a_max);
		segBeta[i]   = CONSTRAIN(segBeta[i], b_min, b_max);
		segLength[i] = CONSTRAIN(segLength[i], l_min, l_max);
	}		
}

//Joystick->Opening
void WriteOpening()
{
	//openingBase for elongation
	if (joyLy > 0)
	{
		openingBase = 0.5;
	}
	else if (joyLy < 0)
  	{
		openingBase = -0.8;
	}
	else
	{
		openingBase = 0;
	}
	
	//opening for rotation
	if (joyRx == 0 && joyRy == 0)
	{
		rawAngle = 0;
		rawAmplitude = 0;
		rawAmplitudeMax = Rmax;
	}
	else
	{
		rawAngle = atan2(joyRx, joyRy);
		rawAmplitude = sqrt(joyRx*joyRx+joyRy*joyRy);
	
		if (rawAmplitude > 0)
		{
			if (rawAngle < M_PI*0.25 || rawAngle > M_PI*0.75)
			{
				rawAmplitudeMax = Rmax/cos(rawAngle);
			}
			else
			{
				rawAmplitudeMax = Rmax/sin(rawAngle);
			}
		}
		else
		{
			if (rawAngle > -M_PI*0.25 || rawAngle < -M_PI*0.75)
			{
				rawAmplitudeMax = Rmax/cos(rawAngle+M_PI);
			}
			else
			{
				rawAmplitudeMax = Rmax/sin(rawAngle+M_PI);
			}						
		}			
	}
	
	angleCommand = rawAngle;
	amplitudeCommand = abs(rawAmplitude/rawAmplitudeMax);
	
	for (int i = 0; i < 6; i++)
	{
		bellowProjection[i] = cos(angleCommand)*bellowConfigurationPx[i] + sin(angleCommand)*bellowConfigurationPy[i];
		OpeningResult[i] = -(bellowProjection[i]/belloConfigurationR*amplitudeCommand*0.5)+openingBase;
	}
}

//Joystick->XYZ (joyRx->x, joyRy->y, joyLy->z)

//FK
Eigen::Vector3f FKtrans(float a[], float b[], float l[], int size)
{
	Matrix4f T = Matrix4f::Identity();
	Matrix4f Tr;
	Vector3f pos;
	float posx;
	float posy;
	float posz;

	for (int i = 0; i < size; i++)
	{
		if (abs(a[i]) < 1e-4)
		{	
			Tr << cos(b[i]), -sin(b[i]), 0,    0,
			  	  sin(b[i]),  cos(b[i]), 0,    0,
			  			  0,  		  0, 1, l[i],
			  			  0,		  0, 0,	   1;
		}
		else
		{
			Tr << cos(b[i])*cos(a[i]), -sin(b[i]),  cos(b[i])*sin(a[i]), (l[i]*cos(b[i])*(1-cos(a[i])))/a[i],
			  	  sin(b[i])*cos(a[i]),  cos(b[i]),  sin(b[i])*sin(a[i]), (l[i]*sin(b[i])*(1-cos(a[i])))/a[i],
			  			   -sin(a[i]),  		0, 			  cos(a[i]), 				 l[i]*sin(a[i])/a[i],
			  					    0,			0,					  0,								   1;
		}
		
		T  = T*Tr;
	}

	posx = T(0,3);
	posy = T(1,3);
	posz = T(2,3);

	pos << posx,
		   posy,
		   posz;

	return pos;
}

void writeXYZ1(int joystickFLag)
{
	if (joystickFLag == 1)
	{
		if (joyRx > 0.05)
		{		
			x = x + x_scale;
		}
		else if (joyRx < -0.05)
		{		
			x = x - x_scale;
		}

		if (joyRy > 0.05)
		{		
			y = y + y_scale;			
		}			
		else if (joyRy < -0.05)
		{			
			y = y - y_scale;
		}
		
		if (joyLy > 0.05)
		{
			z = z + z_scale;
		}			
		else if (joyLy < -0.05)
		{
			z = z - z_scale;	
		}
	}
	else if (joystickFLag == 0)
	{
		if (abs(alpha) < 1e-4)
		{
			x = 0;
			y = 0;
			z = length;
		}
		else
		{
			x = length*cos(beta)*(1-cos(alpha))/alpha;
			y = length*sin(beta)*(1-cos(alpha))/alpha;
			z = length*sin(alpha)/alpha;
		}

		/*float a[1] = {alpha};
		float b[1] = {beta};
		float l[1] = {length};
		int n = 1;
		x = FKtrans(a, b, l, n)(0);
		y = FKtrans(a, b, l, n)(1);
		z = FKtrans(a, b, l, n)(2);*/	
	}

	x = CONSTRAIN(x, x_min, x_max);
	y = CONSTRAIN(y, y_min, y_max);
	z = CONSTRAIN(z, z_min, z_max);
}

void writeXYZ3(int joystickFLag)
{
	if (joystickFLag == 1)
	{
		if (joyRx > 0.05)
		{
			segx_ = segx_ + x_scale;
		}
		else if (joyRx < -0.05)
		{		
			segx_ = segx_ - x_scale;		
		}

		if (joyRy > 0.05)
		{
			segy_ = segy_ + y_scale;	
		}			
		else if (joyRy < -0.05)
		{
			segy_ = segy_ - y_scale;			
		}
		
		if (joyLy > 0.05)
		{
			segz_ = segz_ + z_scale;
		}			
		else if (joyLy < -0.05)
		{		
			segz_ = segz_ - z_scale;	
		}
	}
	else if (joystickFLag == 0)
	{
		int s = 3;
		segx_ = FKtrans(segAlpha_, segBeta_, segLength_, s)(0);
		segy_ = FKtrans(segAlpha_, segBeta_, segLength_, s)(1);	
		segz_ = FKtrans(segAlpha_, segBeta_, segLength_, s)(2);			
	}

	segx_ = CONSTRAIN(segx_, x_min, x_max);
	segy_ = CONSTRAIN(segy_, y_min, y_max);
	segz_ = CONSTRAIN(segz_, z_min9, z_max9);
}

void writeXYZ9(int joystickFLag)
{
	if (joystickFLag == 1)
	{
		if (joyRx > 0.05)
		{
			segx = segx + x_scale;
		}
		else if (joyRx < -0.05)
		{		
			segx = segx - x_scale;		
		}

		if (joyRy > 0.05)
		{
			segy = segy + y_scale;	
		}			
		else if (joyRy < -0.05)
		{
			segy = segy - y_scale;			
		}
		
		if (joyLy > 0.05)
		{
			segz = segz + z_scale;
		}			
		else if (joyLy < -0.05)
		{		
			segz = segz - z_scale;	
		}
	}
	else if (joystickFLag == 0)
	{
		int s = 9;
		segx = FKtrans(segAlpha, segBeta, segLength, s)(0);
		segy = FKtrans(segAlpha, segBeta, segLength, s)(1);	
		segz = FKtrans(segAlpha, segBeta, segLength, s)(2);
	}
	else if (joystickFLag == 2) //XYZ3->XYZ9
	{
		segx = segx_;
		segy = segy_;
		segz = segz_;
	}

	segx = CONSTRAIN(segx, x_min, x_max);
	segy = CONSTRAIN(segy, y_min, y_max);
	segz = CONSTRAIN(segz, z_min9, z_max9);
}

void Init_parameter()
{
	//for Write ABL
	for (int i = 0; i < 9; i++)
	{
		segLength[i] = 0.055;
	}

	for (int i = 0; i < 3; i++)
	{
		segLength_[i] = 0.055*3;
	}

	//for Write Opening
	for (int i = 0; i < 6; i++)
	{
		bellowConfigurationPx[i] = belloConfigurationR*cos(i*2*M_PI/6);
		bellowConfigurationPy[i] = belloConfigurationR*sin(i*2*M_PI/6);
	}
}

int main(int argc, char **argv)
{
	ros::init(argc, argv, "joy_teleopt");
	ros::NodeHandle nh;	
	ros::Rate r(100);     //Hz

	ros::Subscriber sub1 = nh.subscribe("joy", 1, joyCallback);	
	ros::Subscriber sub2 = nh.subscribe("key_number", 1, keyCallback);
	ros::Subscriber sub3 = nh.subscribe("Cmd_ABL_ik",1, ABLCallback);
	ros::Publisher  pub1 = nh.advertise<origarm_ros::Command_ABL>("Cmd_ABL_joy", 100);		
	ros::Publisher  pub3 = nh.advertise<origarm_ros::Command_Position>("Cmd_Position", 100);
	ros::Publisher  pub4 = nh.advertise<origarm_ros::modenumber>("modenumber", 100);
	ros::Publisher  pub5 = nh.advertise<origarm_ros::segnumber>("segnumber", 100);
	//ros::Publisher  pub5  = nh.advertise<origarm_ros::SegOpening>("Cmd_Opening", 100);
	
	Init_parameter();

	while (ros::ok())
	{
		//check whether joystick is available
		if (status == 1)
		{
			if (mode == 0)           //ABL_1 control mode
			{		
				writeABL1(1);		 //joystick -> ABL1
				writeXYZ1(0);		 //ABL1     -> XYZ1
			}
			else if (mode == 1)      //ABL_3 control mode
			{
				writeABL3(1);        //joystick -> ABL3
				writeABL9(2);        //ABL3     -> ABL9
				writeXYZ3(0);		 //ABL3     -> XYZ3
				writeXYZ9(2);        //XYZ3     -> XYZ9
			}
			else if (mode == 2)      //ABL_9 control mode
			{
				writeABL9(1);        //joystick -> ABL9
				writeXYZ9(0);        //ABL9     -> XYZ9
			}
			else if (mode == 3)      //XYZ_1 control mode
			{
				writeXYZ1(1);        //joystick -> XYZ1
				writeABL1(0);        //XYZ1     -> ABL1				
			}
			else if (mode == 4)      //XYZ_3 control mode
			{
				writeXYZ3(1);        //joystick -> XYZ3
				writeABL3(0);        //XYZ3     -> ABL3
				writeABL9(2);        //ABL3     -> ABL9
				writeXYZ9(2);        //XYZ3     -> XYZ9								
			}
			else if (mode == 5)      //XYZ_9 control mode
			{
				writeXYZ9(1);        //joystick -> XYZ9
				writeABL9(0);        //XYZ9     -> ABL9				
			}

			//WriteOpening();

			ROS_INFO("status: %d", status);
			ROS_INFO("segment:%d", segNumber);
			ROS_INFO("mode   :%d", mode);						
		}						
		else if (status == 0)
		{
			//Reset
			for (int i = 0; i < 9; i++)
			{
				segAlpha[i]  = 0;
				segBeta[i]   = 0;
				segLength[i] = 0.055;
			}

			for (int i = 0; i < 3; i++)
			{
				segAlpha_[i]  = 0;
				segBeta_[i]   = 0;
				segLength_[i] = 0.055*3;
			}

			alpha = 0;
			beta = 0;
			length = 0.055;		

			x = 0;
			y = 0;
			z = 0.055; 

			segx_ = 0;
			segy_ = 0;
			segz_ = 0.055*9; 

			segx = 0;
			segy = 0;
			segz = 0.055*9; 

			for (int i = 0; i < 6; i++)
			{
				OpeningResult[i] = 0;
			}
			
			ROS_INFO("status: %d", status);	
		}
		
		origarm_ros::Command_ABL Cmd_ABL;	
		origarm_ros::Command_Position Cmd_Position;	
		//origarm_ros::SegOpening Cmd_Opening;
		origarm_ros::modenumber modenumber;	
		modenumber.modeNumber = mode;
		modenumber.status     = status;

		origarm_ros::segnumber segnumber;
		segnumber.segmentNumber = segNumber;

		/*
		for (int i = 0; i < 6; i++)
		{
			Cmd_Opening.Op[i] = OpeningResult[i];
		}*/
		
		//control mode
		//mode[0]: 1 abl; mode[1]: 3 abl; mode[2]: 9 abl; mode[3]: 1 xyz; mode[4]: 3 xyz; mode[5]: 9 xyz
		//mode[0]: joyA;  mode[1]: joyB;  mode[2]: joyX;  mode[3]: joyY;  mode[4]: joyRB; mode[5]: joyLB
		if (mode == 0)
		{
			Cmd_ABL.segment[0].A = alpha;
			Cmd_ABL.segment[0].B = beta;
			Cmd_ABL.segment[0].L = length;

			for (int i = 1; i < seg; i++)
			{
				Cmd_ABL.segment[i].A = 0;
				Cmd_ABL.segment[i].B = 0;
				Cmd_ABL.segment[i].L = 0.055;
			}

			pub1.publish(Cmd_ABL);
		}
		else if (mode == 1)
		{
			for (int i = 0; i < seg; i++)
			{
				Cmd_ABL.segment[i].A = segAlpha_[int(i/3)]/3;
				Cmd_ABL.segment[i].B = segBeta_[int(i/3)];
				Cmd_ABL.segment[i].L = segLength_[int(i/3)]/3;
				
				printf("ABL3: %f, %f, %f\r\n", segAlpha_[int(i/3)]/3, segBeta_[int(i/3)], segLength_[int(i/3)]/3);	
			}
			
			pub1.publish(Cmd_ABL);
		}
		else if (mode == 2)
		{			
			for (int i = 0; i < seg; i++)
			{
				Cmd_ABL.segment[i].A = segAlpha[i];
				Cmd_ABL.segment[i].B = segBeta[i];
				Cmd_ABL.segment[i].L = segLength[i];

				//printf("ABL9: %f, %f, %f\r\n", segAlpha[i], segBeta[i], segLength[i]);		
			}

			pub1.publish(Cmd_ABL);
		}
		else if (mode == 3)
		{
			Cmd_Position.pose.position.x = x;
			Cmd_Position.pose.position.y = y;
			Cmd_Position.pose.position.z = z;
			Cmd_Position.pose.orientation.x = 1;
			Cmd_Position.pose.orientation.w = 1;

			pub3.publish(Cmd_Position);	
		}
		else if (mode == 4)
		{
			Cmd_Position.pose.position.x = segx_;
			Cmd_Position.pose.position.y = segy_;
			Cmd_Position.pose.position.z = segz_;
			Cmd_Position.pose.orientation.x = 1;
			Cmd_Position.pose.orientation.w = 1;

			pub3.publish(Cmd_Position);	
		}
		else if (mode == 5)
		{
			Cmd_Position.pose.position.x = segx;
			Cmd_Position.pose.position.y = segy;
			Cmd_Position.pose.position.z = segz;
			Cmd_Position.pose.orientation.x = 1;
			Cmd_Position.pose.orientation.w = 1;

			pub3.publish(Cmd_Position);	
		}

		pub4.publish(modenumber);
		pub5.publish(segnumber);

		ros::spinOnce();
		r.sleep();    //sleep for 1/r sec
		//usleep(10000); // N*us
	}

	return 0;
}


