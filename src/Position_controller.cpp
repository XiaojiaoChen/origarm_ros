#include "ros/ros.h"
#include "extensa/States.h"
#include "extensa/ik.h"
#include "extensa/Command_Position.h"
#include "extensa/Command_ABL.h"

using namespace std;

class POSISTION_CONTROLLER
{
  public:
    POSISTION_CONTROLLER()
    {
      pub_ = n_.advertise<extensa::Command_ABL>("Command_ABL", 300);
      sub1_ = n_.subscribe("States", 300, &POSISTION_CONTROLLER::States, this);
      sub2_ = n_.subscribe("Command_Position", 300, &POSISTION_CONTROLLER::Position, this);
      clt_ = n_.serviceClient<extensa::ik>("ik");
    } 

    void States(const extensa::States& msg)
    {
      states_ = msg;
    }

    void Position(const extensa::Command_Position& msg)
    {
      position_ = msg;
      IK();
    }

    void IK()
    {
      extensa::ik Desired;
      Desired.request.input.pose = position_.pose; //desired
      Desired.request.input.ABL = states_.ABL;// present
      if(clt_.call(Desired))
      {
        Cmd_ = Desired.response.output;
        cout << Desired.response.output;
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

    extensa::States states_;
    extensa::Command_Position position_;
    extensa::Command_ABL Cmd_;
};

int main(int argc, char **argv)
{
  ros::init(argc, argv, "POSISTION_CONTROLLER_node");
  
  POSISTION_CONTROLLER POSISTION_CONTROLLER_node;

  ROS_INFO("Ready to do POSISTION_CONTROLLER");

  // ros::AsyncSpinner s(3);
  // s.start();

  ros::Rate loop_rate(10); 
  
  while(ros::ok())
  {
    POSISTION_CONTROLLER_node.pub();
    loop_rate.sleep();
  }
  ros::spin();

  return 0;
}

