#include "ros/ros.h"
#include "extensa/States.h"
#include "extensa/Command_Pre_Open.h"
#include "extensa/Command_ABL.h"

using namespace std;

class ABL_controller
{
  public:
    ABL_controller()
    {
      sub1_ = n_.subscribe("States", 300, &ABL_controller::States, this);
      sub2_ = n_.subscribe("Command_ABL", 300, &ABL_controller::ABL, this);
      pub_ = n_.advertise<extensa::Command_Pre_Open>("Command_Pre_Open", 300);
    }

    void States(const extensa::States& msg)
    {
        ;
    }

    void ABL(const extensa::Command_ABL& msg)
    {
      ABL = msg;
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
    extensa::Command_ABL ABL;
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
    ABL_controller_node.pub();
    ros::spinOnce();
    loop_rate.sleep();
  }

  return 0;
}