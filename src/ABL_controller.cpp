#include "ros/ros.h"
#include "origarm_ros/States.h"
#include "origarm_ros/Command_Pre_Open.h"
#include "origarm_ros/Command_ABL.h"
#include "origarm_ros/Cmd_ABL.h"

//ABL->Pressure
float k0 = 1700;
float l0 = 0.055;
float radR = 0.06;
float radr = 0.02;
float crossA = 0.00126; //M_PI*radr^2
float C1 = 6*k0*radR*0.5/crossA;

#define seg 9
#define act 6

float b1[seg];
float btem[seg];
float b2[seg];
float b3[seg];
float PressureD[seg][act];
float alphad[seg];
float betad[seg];
float lengthd[seg];


using namespace std;

//ABL->Pressure D:desired
void ABLD2PD()
{
	/*float b1 = 2*C1*(lengthd-l0)/(radR*6);
	float btem = C1*alphad/6;
	float b2 = btem*cos(betad);
	float b3 = (1.7320508)*btem*sin(betad);
  PressureD[0] = b1+b2*2;
  PressureD[1] = b1+b2+b3;
  PressureD[2] = b1-b2+b3;
  PressureD[3] = b1-b2*2;
  PressureD[4] = b1-b2-b3;
  PressureD[5] = b1+b2-b3;
*/

  for (int i = 0; i < seg; i ++)
  {
    b1[i] = 2*C1*(lengthd[i]-l0)/(radR*6);
    btem[i] = C1*alphad[i]/6;
    b2[i] = btem[i]*cos(betad[i]);
    b3[i] = (1.7320508)*btem[i]*sin(betad[i]);
  }

  for (int i = 0; i < seg; i++)
  {
 
    PressureD[i][0] = b1[i]+b2[i]*2;
    PressureD[i][1] = b1[i]+b2[i]+b3[i];
    PressureD[i][2] = b1[i]-b2[i]+b3[i];
    PressureD[i][3] = b1[i]-b2[i]*2;
    PressureD[i][4] = b1[i]-b2[i]-b3[i];
    PressureD[i][5] = b1[i]+b2[i]-b3[i];
  }
	
}

class ABL_controller
{
  public:
    ABL_controller()
    {
      sub1_ = n_.subscribe("States", 300, &ABL_controller::States, this);
      //sub2_ = n_.subscribe("Command_ABL", 300, &ABL_controller::ABL, this);
      sub2_ = n_.subscribe("Cmd_ABL", 300, &ABL_controller::ABL, this);
      pub_ = n_.advertise<origarm_ros::Command_Pre_Open>("Command_Pre_Open", 300);
    }

    void States(const origarm_ros::States& msg)
    {
        ;
    }

		/*void ABL(const origarm_ros::Command_ABL& msg)
    {
			alphad = msg.segment[0].A;
			betad = msg.segment[0].B;
			lengthd = msg.segment[0].L; 			
    }*/

    void ABL(const origarm_ros::Cmd_ABL& msg)
    {
      for (int i = 0; i < seg; i++)
      {
        alphad[i] = msg.segment[i].A;
        betad[i] = msg.segment[i].B;
        lengthd[i] = msg.segment[i].L;    
      }       
    }

    void pub()
    {
      /*for (int i = 0; i< 6; i++)
      {
        Cmd_P_O.segment[0].command[i].pressure = PressureD[i]/1000; 
      }*/

      for (int i = 0; i < seg; i++)
      {
        for (int j = 0; j < act; j++)
        {
          Cmd_P_O.segment[i].command[j].pressure = PressureD[i][j]/1000; 
        }
      }

      pub_.publish(Cmd_P_O);
    }

  private:
    ros::NodeHandle n_;
    ros::Subscriber sub1_;
    ros::Subscriber sub2_;
    ros::Publisher pub_ ;

    origarm_ros::Command_Pre_Open Cmd_P_O;
    origarm_ros::Command_ABL Cmd_ABL;
};

int main(int argc, char **argv)
{

  ros::init(argc, argv, "ABL_controller_node");

  ABL_controller ABL_controller_node;

  ros::AsyncSpinner s(3);
  s.start();

  ros::Rate loop_rate(100); 
	
	
  ROS_INFO("Ready for ABL_controller_node");

  while(ros::ok())
  {
    ABLD2PD();
    ABL_controller_node.pub();

    /*for (int i = 0; i < seg; i++)
    {
      printf("Command[%d][0]: %f %f %f %f %f %f\r\n", i, 
        PressureD[i][0], 
        PressureD[i][1],
        PressureD[i][2], 
        PressureD[i][3],
        PressureD[i][4],
        PressureD[i][5]);     
    }*/

    ros::spinOnce();
    loop_rate.sleep();
  }

  return 0;
}
