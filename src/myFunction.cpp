#include "myFunction.h"


Eigen::Matrix3f myRotx(float rad)
{
    Eigen::Matrix3f ret;
    ret << 1, 0, 0, 0, cos(rad), -sin(rad), 0, sin(rad), cos(rad);
    return ret;
}
Eigen::Matrix3f myRoty(float rad)
{
    Eigen::Matrix3f ret;
    ret << cos(rad), 0, sin(rad), 0, 1, 0, -sin(rad), 0, cos(rad);
    return ret;
}
Eigen::Matrix3f myRotz(float rad)
{
    Eigen::Matrix3f ret;
    ret << cos(rad), -sin(rad), 0, sin(rad), cos(rad), 0, 0, 0, 1;
    return ret;
}


void dispMatrix(Eigen::Matrix3f &R){
  static Eigen::IOFormat dispMatFormat(4, 0, ", ", "\n", "[", "]");
  static std::string separator = "\n---------------------------------------\n";
  std::cout << R.format(dispMatFormat) << separator;
}
