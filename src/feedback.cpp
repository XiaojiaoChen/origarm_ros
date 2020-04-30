#include "ros/ros.h"
#include "extensa/Pressure.h"
#include "extensa/ik.h"
#include "extensa/Command.h"
#include "extensa/IKpose.h"

using namespace std;

class feedback
{
  public:
    feedback()
    {
      pub_ = n_.advertise<extensa::Pressure>("feedback", 300);
      sub1_ = n_.subscribe("command", 300, &feedback::command, this);
      sub2_ = n_.subscribe("sensor", 300, &feedback::control, this);
      clt_ = n_.serviceClient<extensa::ik>("ik");
    } 

    void command(const extensa::Command& msg)
    {
      mode_ = msg.command_type;
      cout<< mode_;
      switch (mode_)
       { 
        case 1:
        {
          mode_ = 1;

          extensa::ik Desired;
          extensa::Command msg;
          msg.pose.orientation.x=1;
          msg.pose.orientation.w=1;
          Desired.request.pose = msg.pose;
          Desired.request.pre = now;
          if(clt_.call(Desired))
          {
            cout<<Desired.response;
          }
        }break;

        case 2:
        {
          mode_ = 2;
        }break;

        case 3:
        {
          mode_ = 3;
        }break;
        }
    }

    void control(const extensa::Pressure& msg)
    {
      // function
      now = msg;
      switch (mode_)
      {
        case 1: // position
        {
          feedback_control();
        }break;

        case 2: // pressure
        {
          present_control = msg;
          cout << present_control.segment[1].P[3] << endl;
        }break;

        case 3: // opening
        {
          ;
        }break;
      }

      pub_.publish(present_control);
    }

    void feedback_control()
    {
      ;
    }

  private:
    ros::NodeHandle n_;
    ros::Publisher pub_;
    ros::Subscriber sub1_;
    ros::Subscriber sub2_;
    ros::ServiceClient clt_;

    extensa::Pressure target;
    extensa::Pressure now;
    extensa::Pressure present_control;
    int mode_;
    
};

int main(int argc, char **argv)
{
  ros::init(argc, argv, "feedback_node");
  ros::NodeHandle n;
  
  feedback feedback_node;

  ROS_INFO("Ready to do feedback");
  ros::Rate loop_rate(10); 
  while(ros::ok())
  {
    ros::spinOnce();
    loop_rate.sleep();
  }
  ros::spin();

  return 0;
}

