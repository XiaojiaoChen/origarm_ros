#include "ros/ros.h"
#include "std_msgs/String.h"

void chatterCallback(const std_msgs::String::ConstPtr& msg)
{
  ;
}

int main(int argc, char **argv)
{

  ros::init(argc, argv, "video_node");

  ros::NodeHandle n;

  ros::Subscriber sub = n.subscribe("video", 1000, chatterCallback);

  ros::spin();

  return 0;
}
