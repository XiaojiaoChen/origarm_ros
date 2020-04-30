#include "ros/ros.h"
#include "extensa/Pressure.h"

using namespace std;

int i, j;  

//set a array to store the decompressed pressure data
void decompress(extensa::Pressure& Pre)
{
  for(i=0;i<9;i++)
  {
    //pressure decompress
    for(j=0;j<6;j++)
    {
      Pre.segment[i].P[j] = 6*i+j ;
    }
  }  
}

int main(int argc, char **argv)
{
  ros::init(argc, argv, "sensor_node");

  ros::NodeHandle n;

  ros::Publisher pub = n.advertise<extensa::Pressure>("sensor", 300);

  ros::Rate loop_rate(10); 

  extensa::Pressure Pre;
  Pre.segment.resize(9);
  ROS_INFO("Ready for sensor");

  while(ros::ok())
  {
    decompress(Pre);
    pub.publish(Pre);
    ros::spinOnce();
    loop_rate.sleep();
  }
  return 0;
}