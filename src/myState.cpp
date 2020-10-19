/**************************************************************************************************
	This file include functions for transforming sensor raw data to state information of the arm:
	laser -> length;
	imu   -> orientation -> alpha, beta
**************************************************************************************************/

#include "string.h"
#include "stdlib.h"
#include <stdint.h>
#include <iostream>

#include "myData.h"
#include "myFunction.h"
#include <Eigen/Eigen>
#include <Eigen/Dense>
#include <Eigen/Geometry>

using namespace Eigen;
using Eigen::Matrix4f;
using Eigen::Matrix3f;
using Eigen::MatrixXf;
using Eigen::Vector3f;
using Eigen::VectorXf;
using Eigen::Quaternionf;

using namespace std;

static Matrix3f Rtrans;

IOFormat printmatrix(4, 0, ", ", "\n", "[", "]");
std::string sep = "\n---------------------------------------\n";

void Rtrans_init()
{
	Rtrans << 0, 1,  0, 
		      1, 0,  0, 
		      0, 0, -1; 
}

// transform [quat_imu] imu pose (relative to imu base frame) -> [R_imu] imu pose (relative to arm base frame)
Matrix3f transR(Quaternionf quat_imu)
{	
	Matrix3f Rimu, Rarm;
	Rimu = quat_imu;
	Rarm = Rtrans*Rimu;

	return Rarm;					   
}

/*void printm(Matrix3f rotm)
{
	for (int i = 0; i < 3; i++)
	{
		for (int j = 0; j < 3; j++)
		{
			printf("rotm[%d][%d]:");
		}
	}
}*/

// rotational matrix change relative to arm base frame
Matrix3f getActuatorR(Quaternionf quat0, Quaternionf quat_imu)
{
	Matrix3f Rinit, Rimu, Ract, Rt, Rmid;

	Rinit = quat0;
	Rt = Rinit.transpose();
	Rimu = quat_imu;
	Rmid = Rimu*Rinit.transpose();

	Ract = Rtrans*(Rimu*Rinit.transpose())*Rtrans.transpose();
	
	/*std::cout<<Rinit.format(printmatrix)<<sep;
	std::cout<<Rimu.format(printmatrix)<<sep;
	std::cout<<Rt.format(printmatrix)<<sep;
	std::cout<<Rmid.format(printmatrix)<<sep;
	std::cout<<Ract.format(printmatrix)<<sep;*/

	return Ract;
}

//"averaging" three vectors
Vector3f AverageVect(Vector3f Vect1, Vector3f Vect2, Vector3f Vect3)
{
	Vector3f AVect;
	
	AVect = (Vect1 + Vect2 + Vect3)/3;
	AVect = AVect.normalized();

	return AVect;
}

//"averaging" three rotational matrices
// input are actuator rotational matrices changes based on arm base frame
Matrix3f AverageR(Matrix3f R1, Matrix3f R2, Matrix3f R3)
{
	Vector3f v1, v2, v3;
	float q1, q2, q3, q;
	Matrix3f dR1, dR2, dR3, averagedR, averageR;

	dR1 = R1.transpose()*R1;
	dR2 = R1.transpose()*R2;
	dR3 = R1.transpose()*R3;

	v1 = dR1.col(0);
	v2 = dR2.col(0);
	v3 = dR3.col(0);

	q1 = atan2(v1(1),v1(0));
	q2 = atan2(v2(1),v2(0));
	q3 = atan2(v3(1),v3(0));

	q = (q1 + q2 + q3)/3;

	averagedR<< cos(q), -sin(q), 0,
				sin(q),  cos(q), 0,
				     0,       0, 1;

	averageR = 	R1*averagedR;

	return averageR;		     
}

//get segmentR of each plane
//distinguish two planes based on the assumption that the imu unit on the same plane has the same z vector
//should be determined at the beginning not every step during motion, here assuming actuator [0][2][4] are the same group 
MatrixXf getPlaneCoord(Matrix3f R1, Matrix3f R2, Matrix3f R3, Matrix3f R4, Matrix3f R5, Matrix3f R6)
{
	MatrixXf planeCoord(6,3);
	Matrix3f planeCoord1, planeCoord2;

	planeCoord1 = AverageR(R1, R3, R5);
	planeCoord2 = AverageR(R2, R4, R6);

	planeCoord << planeCoord1,
				  planeCoord2;
	
	return planeCoord;			  
}

//normalVect2A()
float getSegmentA(Vector3f segmentVect1, Vector3f segmentVect2)
{
	float alpha;
	alpha = acos(segmentVect1.dot(segmentVect2));
	return alpha;
}

//segmentR2B()
float getSegmentB(Matrix3f segmentCoord1, Matrix3f segmentCoord2)
{
	float beta;
	Matrix3f R;
	R = segmentCoord1.transpose()*segmentCoord2;

	if (abs(R(2,2)-1) < 1e-3)
	{
		beta = 0;
	}
	else
	{
		beta = atan2(R(1,2),R(0,2));
	}
	
	return beta;
}

// quat2AB()
// input is the quaternion read from imu sensor data, calculate corresponding segment A,B
// write in State_Estimator.cpp


// dist2Length()
// input is the distance read from distance sensor data, calculate corresponding segment L
// write in State_Estimator.cpp