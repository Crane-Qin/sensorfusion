#include "Eigen/Dense"
#include "Eigen/Geometry"
#include "Eigen/Core"
#include <iostream>

using namespace std;

int main(){//g++ test.cpp -o test
    

class CoordTransform{
 public:

   /**
   * Constructor
   */
  //CoordTransform();
  
  /**
   * Destructor
   */
  //virtual ~CoordTransform();
//~CoordTransform();

  /**
   * @brief Converts to a quaternion with a non-negative scalar part
   * @return Quaternion encoding this rotation.
   */
  //void EulerToQuaternion(double& pitch_, double& roll_, double& yaw_, Eigen::Quaterniond quaternion_);
void EulerToQuaternion(const double& pitch_, const double& roll_, const double& yaw_, Eigen::Quaterniond& quaternion_) {
    
    
    double r = roll_ * 0.5;
    double p = pitch_ * 0.5;
    double y = yaw_ * 0.5;

    double sr = std::sin(r);
    double sp = std::sin(p);
    double sy = std::sin(y);

    double cr = std::cos(r);
    double cp = std::cos(p);
    double cy = std::cos(y);
    
    /*Eigen::Quaterniond quaternion_2;
    quaternion_ = &quaternion_2;*/

   
    (quaternion_).w() = cr * cp * cy + sr * sp * sy;
    (quaternion_).x() = cr * sp * cy - sr * cp * sy;
    (quaternion_).y() = sr * cp * cy + cr * sp * sy;
    (quaternion_).z() = cr * cp * sy - sr * sp * cy;

    //return quaternion_;

}

  void outputAsMatrix(const Eigen::Quaterniond& q){
    std::cout << "R=" << std::endl << q.normalized().toRotationMatrix() << std::endl;
  }


};

  // Create an instance
  CoordTransform coordTransform;
  //Eigen::Quaterniond quaternion_;
  //quaternion_test = coordTransform.EulerToQuaternion(M_PI / 4, M_PI / 4, M_PI / 4);
  double roll_1 = 0;
  double pitch_1 = M_PI / 4;
  double yaw_1 = 0;
  Eigen::Quaterniond quaternion_1;
  coordTransform.EulerToQuaternion(pitch_1, roll_1, yaw_1, quaternion_1);
  //Eigen::Quaterniond quaternion_1 = coordTransform.EulerToQuaternion(pitch_1, roll_1, yaw_1);

  // convert a quaternion to a 3x3 rotation matrix 
  //std::cout << "R=" << std::endl << (*quaternion_).normalized().toRotationMatrix() << std::endl;
  //std::cout << (*quaternion_1).vec();


  std::cout << "R=" << std::endl << (quaternion_1).normalized().toRotationMatrix() << std::endl;
  /*Eigen::Matrix3d m;
  m= Eigen::AngleAxisd(yaw_1, Eigen::Vector3d::UnitZ()) *
         Eigen::AngleAxisd(pitch_1, Eigen::Vector3d::UnitX()) *
         Eigen::AngleAxisd(roll_1, Eigen::Vector3d::UnitY());
  cout << m << endl;*/
  return 0;
}



/*
The MIT License (MIT)

Copyright (c) <2016> <slambook>

Permission is hereby granted, free of charge, to any person obtaining a copy of this software and associated documentation files (the "Software"), to deal in the Software without restriction, including without limitation the rights to use, copy, modify, merge, publish, distribute, sublicense, and/or sell copies of the Software, and to permit persons to whom the Software is furnished to do so, subject to the following conditions:

The above copyright notice and this permission notice shall be included in all copies or substantial portions of the Software.

THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY, FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM, OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE SOFTWARE.
*/

#include <iostream>
#include <cmath>
using namespace std;

#include <Eigen/Core>
#include <Eigen/Geometry>

int main( )
{
    Eigen::Matrix3d rotation_matrix = Eigen::Matrix3d::Identity( );
	Eigen::AngleAxisd rotation_vector( M_PI / 2, Eigen::Vector3d( 0, 0, 1 ) );
	cout << "rotation_matrix = \n" << rotation_vector.matrix( ) << endl;  // 转换成矩阵
	rotation_matrix = rotation_vector.toRotationMatrix( );  // 直接赋值
	
	// 用AngleAxis进行坐标变换
	Eigen::Vector3d v( 1, 0, 0 );
	Eigen::Vector3d v_rotated = rotation_vector * v;
	cout << "( 1, 0, 0 ) after rotation = \n" << v_rotated << endl;
	
	// 用旋转矩阵进行坐标变换
	v_rotated = rotation_matrix * v;
	cout << "( 1, 0, 0 ) after rotation = \n" << v_rotated << endl;
	
	// 将旋转矩阵转换成欧拉角
	Eigen::Vector3d euler_angles = rotation_matrix.eulerAngles( 0, 1, 2 );  // x-y-z 顺序，即rpy
	cout << "roll pitch yaw = \n" << euler_angles << endl;
	
	// 用Eigen::Isometry表示欧氏变换矩阵
	Eigen::Isometry3d T = Eigen::Isometry3d::Identity( );  // 三维变换矩阵
	T.rotate( rotation_vector );  // 旋转部分赋值
	T.pretranslate( Eigen::Vector3d( 1, 0, 0 ) );  // 设置平移向量
	cout << "Transform matrix = \n" << T.matrix( ) << endl;
    
	// 用欧氏变换矩阵进行坐标变换，此处的乘法运算符是重载意义上的
	Eigen::Vector3d v_transformed = T * v;  // 相当于R*v + t
	cout << "v transformed = \n" << v_transformed << endl;
	
	// 相应地，仿射变换和摄影变换，使用Eigen::Affine3d和Eigen::Projective3d
	
	// 直接将旋转向量赋值给四元数，反之亦然
	Eigen::Quaterniond q = Eigen::Quaterniond( rotation_vector );
	cout << "quaternion = \n" << q.coeffs( ) << endl;  // Eigen的存储顺序为(x,y,z,w)，实部为w
	// 直接将旋转矩阵赋值给四元数
	q = Eigen::Quaterniond( rotation_matrix );	
	cout << "quaternion = \n" << q.coeffs( ) << endl;
	
	// 用四元数旋转一个向量，使用重载过的乘法运算符
	v_rotated = q * v;  // 相当于qvq^{-1}
	cout << "( 1, 0, 0 ) after rotation = \n" << v_rotated << endl;
		
	return 0;
}