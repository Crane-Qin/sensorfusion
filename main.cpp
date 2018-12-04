#include "euler_to_quaternion.h"
#include <iostream>


using std::cout;
using std::endl;
using Eigen::MatrixXd;
using Eigen::Vector3d;

int main(){//g++ main.cpp euler_to_quaternion.h euler_to_quaternion.cpp -o main

  // Create an instance
  CoordTransform coordTransform;

  double roll_1 = 0;
  double pitch_1 = M_PI / 4;
  double yaw_1 = 0;
  Vector3d attitude;
  attitude << pitch_1, roll_1, yaw_1;
  //Eigen::Quaterniond quaternion_1 = coordTransform.EulerToQuaternion(pitch_1, roll_1, yaw_1);
  Eigen::Quaterniond quaternion_1 = coordTransform.EulerToQuaternion(attitude[0], attitude[1], attitude[2]);
  Eigen::Matrix3d rotation = coordTransform.QuaterniontoRotation(quaternion_1);
  std::cout << "R=" << std::endl << rotation << std::endl;

  //raw sensor observations
  Vector3d observation_raw;
  observation_raw << 5, 8, 10;
  //transform raw sensor observations to vehcile
  Vector3d observation_trans =  rotation * observation_raw;
  Vector3d dsja=observation_trans-observation_raw;
  cout << dsja << endl;
  /*
  double x = 1;
  double y = 1;
  double z = 1;
  Eigen::Translation3d translation(Eigen::Vector3d(x, y, z));
  Eigen::Affine3d pose = translation * rotation;
  std::cout << "transform" << translation << std::endl;*/
    /*
  Eigen::Translation3d trans(Eigen::Vector3d(x, y, z));
  Eigen::Affine3d pose = translation * rotation;
  const Eigen::Translation3d &location,
           const Eigen::Quaterniond &attitude,*/



  /*Eigen::Matrix3d m;
  m= Eigen::AngleAxisd(yaw_1, Eigen::Vector3d::UnitZ()) *
         Eigen::AngleAxisd(pitch_1, Eigen::Vector3d::UnitX()) *
         Eigen::AngleAxisd(roll_1, Eigen::Vector3d::UnitY());
  std::cout <<'m'<< m << std::endl;*/
  //std::cout << coordTransform.AngleAxistoRotation(pitch_1, roll_1, yaw_1) << std::endl;
  return 0;
}