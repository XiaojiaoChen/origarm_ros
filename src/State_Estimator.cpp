#include "ros/ros.h"
#include "extensa/States.h"
#include "extensa/Sensor.h"

using namespace std;

class State_Estimator
{
  public:
    State_Estimator()
    {
      sub_ = n_.subscribe("Sensor", 300, &State_Estimator::callback, this);
      pub_ = n_.advertise<extensa::States>("States", 300);
    }

    void callback(const extensa::Sensor& msg)
    {
      ;
    }

    void pub()
    {
      pub_.publish(states_);
    }

  private:
    ros::NodeHandle n_;
    ros::Subscriber sub_ ;
    ros::Publisher pub_ ;

    extensa::States states_;
};

int main(int argc, char **argv)
{
  ros::init(argc, argv, "State_Estimator_Node");

  State_Estimator State_Estimator_Node;

  ros::Rate loop_rate(100); 

  ROS_INFO("Ready for State_Estimator_Node");

  while(ros::ok())
  {
    State_Estimator_Node.pub();
    ros::spinOnce();
    loop_rate.sleep();
  }
  return 0;
}