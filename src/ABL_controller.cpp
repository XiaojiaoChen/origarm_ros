#include "ros/ros.h"
#include "extensa/States.h"
#include "extensa/Command_Pre_Open.h"
#include "extensa/Command_ABL.h"

//ABL->Pressure
float k0 = 400;
float l0 = 0.055;
float radR = 0.06;
float radr = 0.02;
float crossA = 0.00126; //M_PI*radr^2
float C1 = 6*k0*radR*0.5/crossA;
float PressureD[6];
float alphad;
float betad;
float lengthd;


using namespace std;

//ABL->Pressure D:desired
void ABLD2PD()
{
	float b1 = 2*C1*(lengthd-l0)/(radR*6);
	float btem = C1*alphad/6;
	float b2 = btem*cos(betad);
	float b3 = (1.7320508)*btem*sin(betad);

	PressureD[0] = b1+b2*2;
	PressureD[1] = b1+b2+b3;
	PressureD[2] = b1-b2+b3;
	PressureD[3] = b1-b2*2;
	PressureD[4] = b1-b2-b3;
	PressureD[5] = b1+b2-b3;
}

class ABL_controller
{
  public:
    ABL_controller()
    {
      sub1_ = n_.subscribe("States", 300, &ABL_controller::States, this);
      //sub2_ = n_.subscribe("Command_ABL", 300, &ABL_controller::ABL, this);
			sub2_ = n_.subscribe("Cmd_ABL", 300, &ABL_controller::ABL, this);     //ABL for one segment, <extensa::Seg_ABL>
      pub_ = n_.advertise<extensa::Command_Pre_Open>("Command_Pre_Open", 300);
    }

    void States(const extensa::States& msg)
    {
        ;
    }

		//void ABL(const extensa::Command_ABL& msg){}
    void ABL(const extensa::Seg_ABL& msg)
    {
      //Cmd_ABL = msg;
			alphad = msg.A;
			betad = msg.B;
			lengthd = msg.L; 			
    }

    void pub()
    {
      pub_.publish(Cmd_P_O);
    }

  private:
    ros::NodeHandle n_;
    ros::Subscriber sub1_;
    ros::Subscriber sub2_;
    ros::Publisher pub_ ;

    extensa::Command_Pre_Open Cmd_P_O;
    extensa::Command_ABL Cmd_ABL;
};

int main(int argc, char **argv)
{

  ros::init(argc, argv, "ABL_controller_node");

  ABL_controller ABL_controller_node;

  ros::AsyncSpinner s(3);
  s.start();

  ros::Rate loop_rate(100); 
	
	ABLD2PD();
  ROS_INFO("Ready for ABL_controller_node");

  while(ros::ok())
  {
    ABL_controller_node.pub();
    ros::spinOnce();
    loop_rate.sleep();
  }

  return 0;
}
