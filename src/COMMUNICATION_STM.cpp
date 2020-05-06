#include "ros/ros.h"
#include "origarm_ros/Sensor.h"
#include "origarm_ros/Command_Pre_Open.h"

using namespace std;

class COMMUNICATION_STM
{
  public:
    COMMUNICATION_STM(int seg = 1)
    {
      sub1_ = n_.subscribe("Command_Pre_Open", 300, &COMMUNICATION_STM::encoder, this);
      sub2_ = n_.subscribe("Carmera", 300, &COMMUNICATION_STM::encoder, this);
      pub_ = n_.advertise<origarm_ros::Sensor>("Sensor", 300);
      sensor_.segment.resize(seg);
    }

    void encoder(const origarm_ros::Command_Pre_Open& msg)
    {
      ;
    }

    void pub()
    {
      pub_.publish(sensor_);
    }

  private:
    ros::NodeHandle n_;
    ros::Subscriber sub1_;
    ros::Subscriber sub2_;
    ros::Publisher pub_;

    origarm_ros::Sensor sensor_;
};

int main(int argc, char **argv)
{

  ros::init(argc, argv, "COMMUNICATION_STM_Node");
  
  cout << *argv[1]<<endl;
  int seg = int(*argv[1])-48;
  COMMUNICATION_STM COMMUNICATION_STM_Node(seg);

  ros::AsyncSpinner s(3);
  s.start();

  ros::Rate loop_rate(100); 

  ROS_INFO("Ready for COMMUNICATION_STM_Node");

  while(ros::ok())
  {
    COMMUNICATION_STM_Node.pub();
    ros::spinOnce();
    loop_rate.sleep();
  }
  return 0;
}
