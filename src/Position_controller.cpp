#include "ros/ros.h"
#include "origarm_ros/States.h"
#include "origarm_ros/ik.h"
#include "origarm_ros/Command_Position.h"
#include "origarm_ros/Command_ABL.h"
#include "origarm_ros/modenumber.h"

using namespace std;

class POSISTION_CONTROLLER
{
  public:
    POSISTION_CONTROLLER()
    {
      pub_ = n_.advertise<origarm_ros::Command_ABL>("Cmd_ABL_ik", 100);
      sub1_ = n_.subscribe("States", 1, &POSISTION_CONTROLLER::States, this);
      sub2_ = n_.subscribe("Cmd_Position", 1, &POSISTION_CONTROLLER::Position, this);
      sub3_ = n_.subscribe("modenumber", 1, &POSISTION_CONTROLLER::Mode, this);
      clt_ = n_.serviceClient<origarm_ros::ik>("ik");
    } 

    void States(const origarm_ros::States& msg)
    {
      states_ = msg;
    }

    void Position(const origarm_ros::Command_Position& msg)
    {
      //cout<<"subscribe position"<<endl;
      position_ = msg;
      IK();
    }

    void Mode(const origarm_ros::modenumber& msg)
    {
      mode_ = msg;
    }

    void IK()
    {
      origarm_ros::ik Desired;
      Desired.request.input.pose = position_.pose; //desired
      Desired.request.input.ABL = states_.ABL;// present
      Desired.request.mode = mode_ // mode
      //cout << clt_.call(Desired);
  
      if(clt_.call(Desired))
      {
        Cmd_ = Desired.response.output;
        //cout << Desired.response.output;
      }
    }

    void pub()
    {
      pub_.publish(Cmd_);
    }

  private:
    ros::NodeHandle n_;
    ros::Publisher pub_;
    ros::Subscriber sub1_;
    ros::Subscriber sub2_;
    ros::ServiceClient clt_;

    origarm_ros::States states_;
    origarm_ros::Command_Position position_;
    origarm_ros::Command_ABL Cmd_;
    origarm_ros::modenumber mode_;
};

int main(int argc, char **argv)
{
  ros::init(argc, argv, "POSISTION_CONTROLLER_Node");
  
  POSISTION_CONTROLLER POSISTION_CONTROLLER_Node;

  ROS_INFO("Ready to do POSISTION_CONTROLLER");

  ros::AsyncSpinner s(2);
  s.start();

  ros::Rate loop_rate(100); 
  
  while(ros::ok())
  {
    POSISTION_CONTROLLER_Node.pub();
    //ros::spinOnce();
    loop_rate.sleep();
  }

  ros::waitForShutdown();

  return 0;
}

