#include "Eigen/Dense"
#include "Eigen/Geometry"
#include "Eigen/Core"
#include <iostream>
#include <vector>
#include <string>

int main(){//g++ zdebug.cpp -o zdebug
/*
  double roll_1 = 0;
  double pitch_1 = M_PI / 4;
  double yaw_1 = 0;
  std::vector<double>attitude(3);
  attitude.push_back(roll_1);
  attitude.push_back(pitch_1);
  attitude.push_back(yaw_1);
  std::cout << attitude <<std::endl;*/

    std::vector<std::string> text;
    text.push_back("one");
    text.push_back("two");
    text.push_back("three");

    std::cout << "text = " << text << std::endl;
  return 0;
}
