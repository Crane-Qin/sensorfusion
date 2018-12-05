#include "sensordata_to_vehicle_coordinate.h"
#include <iostream>

using std::cout;
using std::endl;
using Eigen::MatrixXd;
using Eigen::Vector3d;

int main(){//g++ main.cpp sensordata_to_vehicle_coordinate.h sensordata_to_vehicle_coordinate.cpp -o main

  // Create an instance
  CoordTransform coordTransform;
  
  //sensor attitude, relative to vehicle
  double roll_s = 0;
  double pitch_s = M_PI / 4;
  double yaw_s = 0;
  //sensor position, relative to vehicle
  double xs = -0.5;
  double ys = 0.3;
  double zs = 0.8;
  //raw sensor observations
  double xr = -8;
  double yr = 12;
  double zr = 3;
  //Transformed Sensor observation
  double x;
  double y;
  double z;

  Eigen::Quaterniond quaternion_ = coordTransform.EulerToQuaternion(pitch_s, roll_s, yaw_s);
  Eigen::Matrix3d rotation = coordTransform.QuaterniontoRotation(quaternion_);
  //std::cout << "R=" << std::endl << rotation << std::endl;

  coordTransform.SensorDataToVehicle(xr, yr, zr, xs, ys, zs, rotation, &x, &y, &z);
  cout <<"x"<< x <<"y" <<y <<"z" <<z << endl;

  return 0;


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

}