#ifndef MYFUNCTION_H
#define MYFUNCTION_H


#include "ros/ros.h"
#include <Eigen/Dense>
#include <Eigen/Eigen>
#include <Eigen/Geometry>
#include <fstream>
#include <iostream>
#include <stdlib.h>
#include <stdio.h>
#include "math.h"


Eigen::Matrix3f myRotx(float rad);
Eigen::Matrix3f myRoty(float rad);
Eigen::Matrix3f myRotz(float rad);

void dispMatrix(Eigen::Matrix3f &R);


#define CONSTRAIN(x, min, max) (((x) < (min)) ? (min) :  (((x) > (max)) ? (max) : (x)))

#endif // MYFUNCTION_H_
