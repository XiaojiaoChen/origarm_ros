#include "ros/ros.h"
#include "origarm_ros/States.h"
#include "origarm_ros/Command_Pre_Open.h"
#include "origarm_ros/Command_ABL.h"
#include "origarm_ros/SegOpening.h"
#include "origarm_ros/modenumber.h"

#include "myPID.h"
#include "myPID.cpp"
#include "myData.h"
#include <Eigen/Eigen>
#include <Eigen/Dense>
#include <Eigen/Geometry>

float b1[seg];
float btem[seg];
float b2[seg];
float b3[seg];
float pressureD[seg][act];
float alphad[seg];                //ABL desired value
float betad[seg];
float lengthd[seg];

float Texta[seg];                 //external torque
float Textb[seg];
float Tx[seg];
float Ty[seg];
float Fl = 0;
float phycD[seg];
float physD[seg];
float phypD[seg]; 

float pLimitOptimal = 0;
float dalphaRatio[seg];
float dbeta[seg];
float dlength[seg];

float alphar[seg];                //ABL real value, calculated by sensor data
float betar[seg];
float lengthr[seg];

float phycFeedbackAlpha[seg];
float physFeedbackAlpha[seg];
float phycFeedbackBeta[seg];
float physFeedbackBeta[seg];
float phycFeedback[seg];
float physFeedback[seg];
float phypFeedback[seg];
float pressureDFeed[seg][act];
float pressureDBack[seg][act];

int feedbackFlag = 0;

float openingD[seg][act];
int mode_;
float last_pressureD[seg][act];
float last_openingD[seg][act];

using namespace std;

//initialization
PID_Type *alphaPID  = newPID(0.8, 0.005, 0, 0.005, 0.6, 1);
PID_Type *betaPID   = newPID(0.8, 0.005, 0, 0.005, 0.5, 1.5);
PID_Type *lengthPID = newPID(  1,  0.01, 0, 0.005, 0.1, 0.1);


//ABL->Pressure D:desired
void ABLD2PD()
{
  for (int i = 0; i < seg; i ++)
  {
    b1[i] = 2*C1*(lengthd[i]-length0)/(radR*6);
    btem[i] = C1*alphad[i]/6;
    b2[i] = btem[i]*cos(betad[i]);
    b3[i] = (1.7320508)*btem[i]*sin(betad[i]);
    
    pressureD[i][0] = b1[i]+b2[i]*2;
    pressureD[i][1] = b1[i]+b2[i]+b3[i];
    pressureD[i][2] = b1[i]-b2[i]+b3[i];
    pressureD[i][3] = b1[i]-b2[i]*2;
    pressureD[i][4] = b1[i]-b2[i]-b3[i];
    pressureD[i][5] = b1[i]+b2[i]-b3[i];
  }  	
}

//calculate PressureFeedback
void FeedbackController(int feedbackFlag) 
{
  for (int i = 0; i < seg; i++)
  {
    Tx[i] = Texta[i]/crossA/radR-C1*alphad[i];
    Ty[i] = Textb[i]/crossA/radR;
    phycD[i] = cos(betad[i])*Tx[i]-sin(betad[i])*Ty[i];
    physD[i] = sin(betad[i])*Tx[i]+cos(betad[i])*Ty[i];
    phypD[i] = (Fl+6*k0*(lengthd[i]-length0))/crossA;

    pressureDFeed[i][0] = phypD[i]/6+phycD[i]/3;
    pressureDFeed[i][1] = (phycD[i]+phypD[i]+1.7320508*physD[i])/6;
    pressureDFeed[i][2] = (phypD[i]-phycD[i]+1.7320508*physD[i])/6;
    pressureDFeed[i][3] = phypD[i]/6-phycD[i]/3;
    pressureDFeed[i][4] = (phypD[i]-phycD[i]-1.7320508*physD[i])/6;
    pressureDFeed[i][5] = (phypD[i]+phycD[i]-1.7320508*physD[i])/6;

    pressureDFeed[i][0]-=pLimitOptimal;
    pressureDFeed[i][1]+=pLimitOptimal;
    pressureDFeed[i][2]-=pLimitOptimal;
    pressureDFeed[i][3]+=pLimitOptimal;
    pressureDFeed[i][4]-=pLimitOptimal;
    pressureDFeed[i][5]+=pLimitOptimal;
  }
    
  if(feedbackFlag)
  {
    for (int i = 0; i < seg; i++)
    {
      dalphaRatio[i] = updatePID(alphaPID,1,alphar[i]/alphad[i]);
      dbeta[i]       = updatePID(betaPID,betad[i],betar[i]);
      dlength[i]     = updatePID(lengthPID,lengthd[i],lengthr[i]);

      phycFeedbackBeta[i] = (cos(dbeta[i])-1)*phycD[i]-sin(dbeta[i])*physD[i];
      physFeedbackBeta[i] = sin(dbeta[i])*phycD[i]+(cos(dbeta[i])-1)*physD[i];

      phycFeedbackAlpha[i] = dalphaRatio[i]*(cos(dbeta[i])*phycD[i]-sin(dbeta[i])*physD[i]);
      physFeedbackAlpha[i] = dalphaRatio[i]*(sin(dbeta[i])*phycD[i]+cos(dbeta[i])*physD[i]);

      phycFeedback[i] = phycFeedbackBeta[i]+phycFeedbackAlpha[i];
      physFeedback[i] = physFeedbackBeta[i]+physFeedbackAlpha[i];
      phypFeedback[i] = 6*k0*dlength[i]/crossA;

      pressureDBack[i][0] = phypFeedback[i]/6+phycFeedback[i]/3;
      pressureDBack[i][1] = (phycFeedback[i]+phypFeedback[i]+1.7320508*physFeedback[i])/6;
      pressureDBack[i][2] = (phypFeedback[i]-phycFeedback[i]+1.7320508*physFeedback[i])/6;
      pressureDBack[i][3] = phypFeedback[i]/6-phycFeedback[i]/3;
      pressureDBack[i][4] = (phypFeedback[i]-phycFeedback[i]-1.7320508*physFeedback[i])/6;
      pressureDBack[i][5] = (phypFeedback[i]+phycFeedback[i]-1.7320508*physFeedback[i])/6;
    }  
  }
  else
  {
    for(int i = 0; i < seg; i++)
    {
      for (int j = 0; j < act; j++)
      {
        pressureDBack[i][j] = 0;
      }
    }
  }

  for(int i = 0; i < seg; i++)
  {
    for (int j = 0; j < act; j++)
    {
      pressureD[i][j] = pressureDFeed[i][j] + pressureDBack[i][j];
    }    
  }   
}


class ABL_controller
{
  public:
    ABL_controller()
    {
      sub1_ = n_.subscribe("States", 300, &ABL_controller::States, this);
      sub2_ = n_.subscribe("Cmd_ABL", 300, &ABL_controller::ABL, this);
      sub3_ = n_.subscribe("Cmd_Opening", 300, &ABL_controller::Opening, this);
      sub4_ = n_.subscribe("modenumber", 300, &ABL_controller::mode, this);
      pub_ = n_.advertise<origarm_ros::Command_Pre_Open>("Command_Pre_Open", 300);
    }

    void States(const origarm_ros::States& msg)
    {
      for (int i = 0; i < seg; i++)
      {
        alphar[i] = msg.ABL.segment[i].A;
        betar[i] = msg.ABL.segment[i].B;
        lengthr[i] = msg.ABL.segment[i].L;
      }
    }

    void ABL(const origarm_ros::Command_ABL& msg)
    {
      for (int i = 0; i < seg; i++)
      {
        alphad[i] = msg.segment[i].A;
        betad[i] = msg.segment[i].B;
        lengthd[i] = msg.segment[i].L;    
      }       
    }

    void Opening(const origarm_ros::SegOpening& msg)
    {
      for (int j = 0; j < act; j++)
      {
        openingD[0][j] = msg.Op[j];
      }

      /*for (int i = 0; i < seg; i++)
      {
        for (int j = 0; j < act; j++)
        {
          openingD[i][j] = msg.Op[j];
        }
      }*/
    }

    void mode (const origarm_ros::modenumber& msg)
    {
      mode_ = msg.modeNumber;
      
      for (int i = 0; i < seg; i++)
      {
        for (int j = 0; j < act; j++)
        {
          if (mode_ == 1)
          { 
            Cmd_P_O.segment[i].command[j].pressure = openingD[i][j]*32767; 
            Cmd_P_O.segment[i].command[j].valve    = 0;                       //bool == 0, commandType == openingCommandType            
          }
          else
          {
            Cmd_P_O.segment[i].command[j].pressure = pressureD[i][j]/1000; 
            Cmd_P_O.segment[i].command[j].valve    = 1;                     //bool == 1, commandType == pressureCommandType
          }                                
        }
      }
      
      for (int i = 0; i < seg; i++)
      {
        for (int j = 0; j < act; j++)
        {
          last_pressureD[i][j] = Cmd_P_O.segment[i].command[j].pressure;
        }          
      }  
    }
    
    void pub()
    {
      pub_.publish(Cmd_P_O);
    }

    private:
      ros::NodeHandle n_;
      ros::Subscriber sub1_;
      ros::Subscriber sub2_;
      ros::Subscriber sub3_;
      ros::Subscriber sub4_;
      ros::Publisher pub_ ;

      origarm_ros::Command_Pre_Open Cmd_P_O;
      origarm_ros::Command_ABL Command_ABL;
      origarm_ros::SegOpening Cmd_Opening;
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
    //ABLD2PD();
    FeedbackController(feedbackFlag);

    ABL_controller_node.pub();

    //printf("lastmode: %d, mode: %d\r\n",last_mode_, mode_);

    /*for (int i = 0; i < seg; i++)
    {
      printf("Command[%d]: %f %f %f %f %f %f\r\n", i, 
        pressureD[i][0], 
        pressureD[i][1],
        pressureD[i][2], 
        pressureD[i][3],
        pressureD[i][4],
        pressureD[i][5]);     
    }*/

    ros::spinOnce();
    loop_rate.sleep();
  }

  return 0;
}
