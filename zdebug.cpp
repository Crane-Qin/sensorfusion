#include "Eigen/Dense"
#include "Eigen/Geometry"
#include "Eigen/Core"
#include <iostream>
#include <vector>
#include <string>

int main(){//g++ zdebug.cpp -o zdebug

  double roll_1 = 0;
  double pitch_1 = M_PI / 4;
  double yaw_1 = 0;
  std::vector<double>attitude(3);
  attitude.push_back(roll_1);
  attitude.push_back(pitch_1);
  attitude.push_back(yaw_1);
  std::cout << attitude[3] <<std::endl;
  return 0;
}
