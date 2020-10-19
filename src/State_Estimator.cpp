#include "ros/ros.h"
#include "origarm_ros/States.h"
#include "origarm_ros/Sensor.h"

#include "math.h"
#include "myData.h"
#include "myFunction.h"
#include "myState.cpp"

#include "myData.h"
#include "myFunction.h"
#include <Eigen/Eigen>
#include <Eigen/Dense>
#include <Eigen/Geometry>
#include <fstream>
#include <iostream>

using namespace Eigen;
using Eigen::Matrix4f;
using Eigen::Matrix3f;
using Eigen::MatrixXf;
using Eigen::Vector3f;
using Eigen::VectorXf;
using Eigen::Quaternionf;

using namespace std;

float alphar[seg];
float betar[seg];
float lengthr[seg];
float dist0;

//local definition of pressure, distance, imu
int16_t pressurer[seg][act];
uint16_t distancer[seg][act];
float imur[seg][act][4];
int actn = 0;

Quaternionf quat0, quat;
Matrix3f R[seg][act], dR, Rtest;
MatrixXf Rsegpair(6,3);

// predefined quat0, the initial sensor data, read from file
// either by initial manually or loading from imudata.yaml file/ .txt file
ifstream inFile;
float QUAT0[seg][act][4];


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
      // states_.pose.position.x = ;
      // states_.pose.position.y = ;
      // states_.pose.position.z = ;
      // states_.pose.orientation.w = ;
      // states_.pose.orientation.x = ;
      // states_.pose.orientation.y = ;
      // states_.pose.orientation.z = ;

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
  Matrix3f R1, R2;

  for (int i = 0; i < seg; i++)
  {
    for (int j = 0; j < act; j++)
    {
      quat.w() = imur[i][j][0];
      quat.x() = imur[i][j][1];
      quat.y() = imur[i][j][2];
      quat.z() = imur[i][j][3];

      ////Assuming actuator[i][0],[i][2],[i][4] have the same initial imu data
      /*if (j % 2 == 0) 
      {
        quat0.w() = 1;
        quat0.x() = 0;
        quat0.y() = 0;
        quat0.z() = 0;
      }
      else
      {
        quat0.w() = 0;
        quat0.x() = 1;
        quat0.y() = 0;
        quat0.z() = 0;
      }*/

       quat0.w() = QUAT0[i][j][0];
       quat0.x() = QUAT0[i][j][1];
       quat0.y() = QUAT0[i][j][2];
       quat0.z() = QUAT0[i][j][3];

       R[i][j] = getActuatorR(quat0, quat);
			 Rtest = R[i][j];

			/*printf("Rtest[0][1:3]:%f, %f, %f\n", Rtest(0,0),Rtest(0,1),Rtest(0,2));
			printf("Rtest[1][1:3]:%f, %f, %f\n", Rtest(1,0),Rtest(1,1),Rtest(1,2));
			printf("Rtest[2][1:3]:%f, %f, %f\n", Rtest(2,0),Rtest(2,1),Rtest(2,2));*/
			 			              
    }
  }
 
  for (int i = 0; i < seg; i++)
  {
    Rsegpair = getPlaneCoord(R[i][0], R[i][1], R[i][2], R[i][3], R[i][4], R[i][5]);
		

    R1 = Rsegpair.block<3,3>(0,0);
    R2 = Rsegpair.block<3,3>(3,0);
    dR = R1.transpose()*R2;

		std::cout<<R1.format(printmatrix)<<sep;
		std::cout<<R2.format(printmatrix)<<sep;
		std::cout<<dR.format(printmatrix)<<sep;

    alphar[i] = acos(dR(2,2));
    if (abs(dR(2,2)-1) < 1e-3)
    {
      betar[i] = 0;
    }
    else
    {
      betar[i] = atan2(dR(1,2),dR(0,2));
    }
    
    alphar[i] = CONSTRAIN(alphar[i], 0.0001, 0.5*M_PI);
		printf("Alphar[%d]:%f\n", i, alphar[i]);

  }
}

void dist2Length()
{
  for (int i = 0; i < seg; i++)
  {     
    lengthr[i] = (distancer[i][0] + distancer[i][1] + distancer[i][2] + distancer[i][3] + distancer[i][4] + distancer[i][5])/6-dist0;  
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


	Rtrans_init();
	inFile.open("/home/ubuntu/Desktop/imu_data.txt", ios::in);
  
   if (!inFile)
   {
     printf("%s\n", "unable to open the file.");
     exit(1);
   }

   if (!inFile.eof())
   {
     for (int p = 0; p < seg; p++)
     {
				for (int q = 0; q < act; q++)
				{
					inFile>>QUAT0[p][q][0]>>QUAT0[p][q][1]>>QUAT0[p][q][2]>>QUAT0[p][q][3];
				}
     }              
   }

  while(ros::ok())
  {
    quat2AB();
    dist2Length();

    State_Estimator_Node.pub();
    ros::spinOnce();
    loop_rate.sleep();
  }

  inFile.close(); 
  return 0;
}