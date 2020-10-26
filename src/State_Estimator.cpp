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
using Eigen::Matrix3f;
using Eigen::Matrix4f;
using Eigen::MatrixXf;
using Eigen::Quaternionf;
using Eigen::Vector3f;
using Eigen::VectorXf;

using namespace std;

float alphar[seg];
float betar[seg];
float lengthr[seg];
float dist0[seg];

//local definition of pressure, distance, imu
int16_t pressurer[seg][act];
int16_t distancer[seg][act];
float imur[seg][act][4];
int imu_act[seg][act];

Quaternionf quat0, quat;
Matrix3f R[seg][act], dR, Rinit, Rconfig[seg][act], Rtrans;
MatrixXf Rsegpair(6, 3);
Matrix3f E;

// predefined quat0, the initial sensor data, read from file
// either by initial manually or loading from imudata.yaml file/ .txt file
ifstream inFile;
float QUAT0[seg][act][4];
float rad2deg = 180 / M_PI;
Matrix3f RI;

Matrix3f rotx(float rad)
{
	Matrix3f ret;
	ret << 1, 0, 0,
		0, cos(rad), -sin(rad),
		0, sin(rad), cos(rad);
	return ret;
}
Matrix3f roty(float rad)
{

	Matrix3f ret;
	ret << cos(rad), 0, sin(rad),
		0, 1, 0,
		-sin(rad), 0, cos(rad);
	return ret;
}
Matrix3f rotz(float rad)
{

	Matrix3f ret;
	ret << cos(rad), -sin(rad), 0,
		sin(rad), cos(rad), 0,
		0, 0, 1;
	return ret;
}
void Rconfig_init()
{
	RI << 1, 0, 0,
		0, 1, 0,
		0, 0, 1;
	for (int i = 0; i < seg; i++)
	{
		Rconfig[i][0] = RI;

		Rconfig[i][2] = Rconfig[i][0] * rotz(M_PI * 2 / 3);

		Rconfig[i][4] = Rconfig[i][2] * rotz(M_PI * 2 / 3);

		Rconfig[i][1] = Rconfig[i][0] * rotx(M_PI) * rotz(-M_PI / 3);

		Rconfig[i][3] = Rconfig[i][1] * rotz(-M_PI * 2 / 3);

		Rconfig[i][5] = Rconfig[i][3] * rotz(-M_PI * 2 / 3);
	}
}

class State_Estimator
{
public:
	State_Estimator()
	{
		sub_ = n_.subscribe("Sensor", 300, &State_Estimator::callback, this);
		pub_ = n_.advertise<origarm_ros::States>("States", 300);
	}

	void callback(const origarm_ros::Sensor &Sensor_)
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

				imu_act[i][j] = Sensor_.sensor_segment[i].sensor_actuator[j].imu_active;
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
	ros::Subscriber sub_;
	ros::Publisher pub_;

	origarm_ros::States states_;
};

void quat2AB()
{
	Matrix3f R1, R2;

	for (int i = 0; i < seg; i++)
	{
		for (int j = 0; j < act; j++)
		{
			/*********************************************
			imu error: 
			imu[0][0]
			imu[1][0], [1][1], [1][3], [1][5];
			imu[2][3];
			imu[3][1], [3][2], [3][5];
			imu[5][2], [5][3], [5][5]	
			*********************************************/
			quat0.w() = QUAT0[i][j][0];
			quat0.x() = QUAT0[i][j][1];
			quat0.y() = QUAT0[i][j][2];
			quat0.z() = QUAT0[i][j][3];

			Rinit = quat0;
			// Rtrans = Rconfig[i][j] * Rinit.transpose();
			Rtrans << 1,  0,  0,
					  0, -1,  0,
					  0,  0, -1;	

			if ((i == 0 && j == 0) ||
				(i == 1 && j == 0) || (i == 1 && j == 1) || (i == 1 && j == 3) || (i == 1 && j == 5) ||
				(i == 2 && j == 3) ||
				(i == 3 && j == 1) || (i == 3 && j == 2) || (i == 3 && j == 5) ||
				(i == 5 && j == 2) || (i == 5 && j == 3) || (i == 5 && j == 5)) 
			//if (imu_act[i][j] == 0)
			{
				R[i][j] = E;
			}
			else
			{
				quat.w() = imur[i][j][0];
				quat.x() = imur[i][j][1];
				quat.y() = imur[i][j][2];
				quat.z() = imur[i][j][3];

				R[i][j] = getActuatorR(quat0, quat, Rtrans);

				if (i == 2)
				{
					printf("actuator[2][%d]\n", j);
					std::cout << R[i][j].format(printmatrix) << sep;		
				}
			}									
		}
	}

	for (int i = 0; i < 6; i++)
	{
		if (i == 0)
		{
			R1 = R[0][2];
			R2 = R[1][2];
		}
		else if (i == 1)
		{
			R1 = R[1][2];
			R2 = R[2][2];
		}
		else if (i == 2)
		{
			R1 = R[2][0];
			R2 = R[3][0];
		}
		else if (i == 3)
		{
			R1 = R[3][0];
			R2 = R[4][0];
		}
		else if (i == 4)
		{
			R1 = R[4][0];
			R2 = R[5][4];
		}
		else if (i == 5)
		{
			R1 = R[5][4];
			R2 = R[5][1];
		}

		dR = R1.transpose() * R2;

		//printf("Segment[%d]\n",i);
		/*if (i == 5)
		{
			printf("imu:%f, %f, %f, %f\n",imur[i][0][0],imur[i][0][1],imur[i][0][2],imur[i][0][3]);	
			printf("imu:%f, %f, %f, %f\n",imur[i][2][0],imur[i][2][1],imur[i][2][2],imur[i][2][3]);	
			printf("imu:%f, %f, %f, %f\n",imur[i][4][0],imur[i][4][1],imur[i][4][2],imur[i][4][3]);

			printf("imu:%f, %f, %f, %f\n",imur[i][1][0],imur[i][1][1],imur[i][1][2],imur[i][1][3]);	
			printf("imu:%f, %f, %f, %f\n",imur[i][3][0],imur[i][3][1],imur[i][3][2],imur[i][3][3]);	
			printf("imu:%f, %f, %f, %f\n",imur[i][5][0],imur[i][5][1],imur[i][5][2],imur[i][5][3]);				
		}*/


		// if (i == 4 || i == 5)
		// {
		// 	printf("segment[%d]\n", i);			
		// 	std::cout << R[4][0].format(printmatrix) << sep;

		// 	Rtemp = R[i][0].transpose() * R[i + 1][2];
		// 	std::cout << Rtemp.format(printmatrix) << sep;

		// 	Rtemp = R[i][0].transpose() * R[i][1];
		// 	std::cout << Rtemp.format(printmatrix) << sep;

		// 	Rtemp = R[i][0].transpose() * R[i][3];
		// 	std::cout << Rtemp.format(printmatrix) << sep;

		// 	Rtemp = R[i][0].transpose() * R[i][5];
		// 	std::cout << Rtemp.format(printmatrix) << sep;

		// 	Rtemp = R[i][2].transpose() * R[i + 1][0];
		// 	std::cout << Rtemp.format(printmatrix) << sep;

		// 	Rtemp = R[i][2].transpose() * R[i + 1][2];
		// 	std::cout << Rtemp.format(printmatrix) << sep;

		// 	Rtemp = R[i][2].transpose() * R[i][1];
		// 	std::cout << Rtemp.format(printmatrix) << sep;

		// 	Rtemp = R[i][2].transpose() * R[i][3];
		// 	std::cout << Rtemp.format(printmatrix) << sep;

		// 	Rtemp = R[i][2].transpose() * R[i][5];
		// 	std::cout << Rtemp.format(printmatrix) << sep;
		// }

		alphar[i] = acos(dR(2, 2));

		if (abs(dR(2, 2) - 1) < 1e-3)
		{
			betar[i] = 0;
		}
		else
		{
			betar[i] = atan2(dR(1, 2) / sin(alphar[i]), dR(0, 2) / sin(alphar[i]));
		}

		alphar[i] = CONSTRAIN(alphar[i], 0.0001, 0.5 * M_PI);
	}
}

void dist2Length()
{
	for (int i = 0; i < seg; i++)
	{
		lengthr[i] = (distancer[i][0] + distancer[i][1] + distancer[i][2] + distancer[i][3] + distancer[i][4] + distancer[i][5]) / 6;
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

	Rconfig_init();
	E = Matrix3f::Identity();
	inFile.open("/home/ubuntu/catkin_ws/src/origarm_ros/predefined_param/imu_data.txt", ios::in);

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
				inFile >> QUAT0[p][q][0] >> QUAT0[p][q][1] >> QUAT0[p][q][2] >> QUAT0[p][q][3];
			}
		}
	}

	while (ros::ok())
	{
		quat2AB();
		dist2Length();

		for (int i = 0; i < seg; i++)
		{
			alphar[i] = rad2deg * alphar[i];
			betar[i] = rad2deg * betar[i];
		}

		printf("A[0]:%d, [1]:%d, [2]:%d, [3]:%d, [4]:%d, [5]:%d  |  ", (int)alphar[0], (int)alphar[1], (int)alphar[2], (int)alphar[3], (int)alphar[4], (int)alphar[5]);
		printf("B[0]:%d, [1]:%d, [2]:%d, [3]:%d, [4]:%d, [5]:%d  |  ", (int)betar[0], (int)betar[1], (int)betar[2], (int)betar[3], (int)betar[4], (int)betar[5]);
		printf("L[0]:%d, [1]:%d, [2]:%d, [3]:%d, [4]:%d, [5]:%d\n", (int)lengthr[0], (int)lengthr[1], (int)lengthr[2], (int)lengthr[3], (int)lengthr[4], (int)lengthr[5]);

		State_Estimator_Node.pub();
		ros::spinOnce();
		loop_rate.sleep();
	}

	inFile.close();
	return 0;
}