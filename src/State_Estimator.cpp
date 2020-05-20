#include "ros/ros.h"
#include "origarm_ros/States.h"
#include "origarm_ros/Sensor.h"

#include "math.h"
#include "myData.h"
#include "myFunction.h"

using namespace std;

float alphar[seg];
float betar[seg];
float lengthr[seg];

//local definition of pressure, distance, imu
int16_t pressurer[seg][act];
uint16_t distancer[seg][act];
int16_t imur[seg][act][4];
int actn = 0;

class State_Estimator
{
  public:
    State_Estimator()
    {
      sub_ = n_.subscribe("Sensor", 300, &State_Estimator::callback, this);
      pub_ = n_.advertise<origarm_ros::States>("States", 300);
    }

    void callback(const origarm_ros::Sensor& Sensor_)
    {
      for (int i = 0; i < seg; i++)
      {
        for (int j = 0; j < act; j++)
        {
          /*sensorData.data[i][j].pressure = Sensor_.sensor_segment[i].sensor_actuator[j].pressure;
          sensorData.data[i][j].distance = Sensor_.sensor_segment[i].sensor_actuator[j].distance;
          sensorData.data[i][j].quaternion.imuData[0] = Sensor_.sensor_segment[i].sensor_actuator[j].pose.orientation.w; //double check
          sensorData.data[i][j].quaternion.imuData[1] = Sensor_.sensor_segment[i].sensor_actuator[j].pose.orientation.x;
          sensorData.data[i][j].quaternion.imuData[2] = Sensor_.sensor_segment[i].sensor_actuator[j].pose.orientation.y;
          sensorData.data[i][j].quaternion.imuData[3] = Sensor_.sensor_segment[i].sensor_actuator[j].pose.orientation.z;*/

          pressurer[i][j] = Sensor_.sensor_segment[i].sensor_actuator[j].pressure;
          distancer[i][j] = Sensor_.sensor_segment[i].sensor_actuator[j].distance;
          imur[i][j][0] = Sensor_.sensor_segment[i].sensor_actuator[j].pose.orientation.w; //double check
          imur[i][j][1] = Sensor_.sensor_segment[i].sensor_actuator[j].pose.orientation.x;
          imur[i][j][2] = Sensor_.sensor_segment[i].sensor_actuator[j].pose.orientation.y;
          imur[i][j][3] = Sensor_.sensor_segment[i].sensor_actuator[j].pose.orientation.z;
        }
      }
    }

    void pub()
    {
      //real ABL calculated from sensor
      for (int i = 0; i < seg; i++)
      {
        states_.ABL.segment[i].A = alphar[i];
        states_.ABL.segment[i].B = betar[i];
        states_.ABL.segment[i].L = lengthr[i];
      }
      
      //real pose calculated from sensor
      /*states_.pose.position.x = ;
      states_.pose.position.y = ;
      states_.pose.position.z = ;
      states_.pose.orientation.w = ;
      states_.pose.orientation.x = ;
      states_.pose.orientation.y = ;
      states_.pose.orientation.z = ;*/

      pub_.publish(states_);
    }

  private:
    ros::NodeHandle n_;
    ros::Subscriber sub_ ;
    ros::Publisher pub_ ;

    origarm_ros::States states_;
};

void quat2AB()
{
  
  /*float n1=2*imuData.q0*imuData.q2 + 2*imuData.q1*imuData.q3;
  float n2=2*imuData.q2*imuData.q3 - 2*imuData.q0*imuData.q1;
  float n3=imuData.q0*imuData.q0 - imuData.q1*imuData.q1 - imuData.q2*imuData.q2 + imuData.q3*imuData.q3;

  n1=CONSTRAIN(n1,-1,1);
  n2=CONSTRAIN(n2,-1,1);
  n3=CONSTRAIN(n3,-1,1);

  alpha=acos(-n3);
  alpha=CONSTRAIN(alpha,0.001,M_PI);//avoid singularity
  beta=atan2(-n2,n1);
  if(alpha<0.13)//the smaller alpha is, the larger error beta would have.
  beta=0;*/
  
  float n1[seg];
  float n2[seg];
  float n3[seg];

  for (int i = 0; i < seg; i++)
  {
    n1[i] = 2*imur[i][actn][0]*imur[i][actn][2] + 2*imur[i][actn][1]*imur[i][actn][3];
    n2[i] = 2*imur[i][actn][2]*imur[i][actn][3] - 2*imur[i][actn][0]*imur[i][actn][1];
    n3[i] = imur[i][actn][0]*imur[i][actn][0] - imur[i][actn][1]*imur[i][actn][1] - imur[i][actn][2]*imur[i][actn][2] + imur[i][actn][3]*imur[i][actn][3];

    n1[i] = CONSTRAIN(n1[i],-1,1);
    n2[i] = CONSTRAIN(n2[i],-1,1);
    n3[i] = CONSTRAIN(n3[i],-1,1);

    alphar[i] = acos(-n3[i]);
    alphar[i] = CONSTRAIN(alphar[i], 0.001, M_PI);
    betar[i]  = atan2(-n2[i],n1[i]);
    
    if (alphar[i] < 0.13)
    {
      betar[i] = 0;
    }
  }
}

void dist2Length()
{
  for (int i = 0; i < seg; i++)
  {    
    /*lengthr[i] = (sensorData.data[i][0].distance + sensorData.data[i][1].distance + sensorData.data[i][2].distance + 
      sensorData.data[i][3].distance + sensorData.data[i][4].distance + sensorData.data[i][5].distance)/act;*/  

    lengthr[i] = (distancer[i][0] + distancer[i][1] + distancer[i][2] + distancer[i][3] + distancer[i][4] + distancer[i][5])/act-length0;  
  }
}

int main(int argc, char **argv)
{
  ros::init(argc, argv, "State_Estimator_Node");

  State_Estimator State_Estimator_Node;

  ros::AsyncSpinner s(2);
  s.start();

  ros::Rate loop_rate(100); 

  ROS_INFO("Ready for State_Estimator_Node");

  while(ros::ok())
  {
    quat2AB();
    dist2Length();

    State_Estimator_Node.pub();
    ros::spinOnce();
    loop_rate.sleep();
  }
  return 0;
}