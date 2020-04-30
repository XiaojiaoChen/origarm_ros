#include "ros/ros.h"
#include "extensa/Pressure.h"

using namespace std;

class gas
{
  public:
    gas()
    {
      sub_ = n_.subscribe("feedback", 300, &gas::encoder, this);
    }

    void encoder(const extensa::Pressure& msg)
    {
      ;
    }

  private:
    ros::NodeHandle n_;
    ros::Subscriber sub_;
    extensa::Pressure pre_;
};

int main(int argc, char **argv)
{

  ros::init(argc, argv, "gas_node");

  gas gas_node;

  ros::spin();

  return 0;
}
