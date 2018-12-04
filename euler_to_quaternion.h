#ifndef PID_H
#define PID_H

#include "Eigen/Geometry"

class CoordTransform{
 public:

   /**
   * Constructor
   */
  CoordTransform();
  
  /**
   * Destructor
   */
  virtual ~CoordTransform();

  /**
   * @brief Converts to a quaternion with a non-negative scalar part
   * @return Quaternion encoding this rotation.
   */
  Eigen::Quaterniond EulerToQuaternion(const double& pitch_, const double& roll_, const double& yaw_);

  //@brief Convert a quaternion to a 3x3 rotation matrix 
  Eigen::Matrix3d QuaterniontoRotation(const Eigen::Quaterniond& q){
    return q.normalized().toRotationMatrix();
  }
  
  //TODO not use
  //@brief Convert AngleAxis to a 3x3 rotation matrix
  Eigen::Quaterniond AngleAxistoRotation(const double& pitch_, const double& roll_, const double& yaw_){
    return Eigen::AngleAxisd(yaw_, Eigen::Vector3d::UnitZ()) *
           Eigen::AngleAxisd(pitch_, Eigen::Vector3d::UnitX()) *
           Eigen::AngleAxisd(roll_, Eigen::Vector3d::UnitY());
  }

};

 
/* from apollo
  Eigen::Quaternion<T> ToQuaternion() const {
    T r = roll_ * 0.5;
    T p = pitch_ * 0.5;
    T y = yaw_ * 0.5;

    T sr = std::sin(r);
    T sp = std::sin(p);
    T sy = std::sin(y);

    T cr = std::cos(r);
    T cp = std::cos(p);
    T cy = std::cos(y);

    T qw = cr * cp * cy - sr * sp * sy;
    T qx = cr * sp * cy - sr * cp * sy;
    T qy = cr * sp * sy + sr * cp * cy;
    T qz = cr * cp * sy + sr * sp * cy;
    if (qw < 0.0) return {-qw, -qx, -qy, -qz};
    return {qw, qx, qy, qz};
  }
*/

/* translation part
for (auto point : fused_object_->object->polygon.points) {
            point.x += translation[0];
            point.y += translation[1];
            point.z += translation[2];
          }

*/
/*
//https://blog.csdn.net/silangquan/article/details/39008903
Matrix4x4(
		1.0f - 2.0f*y*y - 2.0f*z*z, 2.0f*x*y - 2.0f*z*w, 2.0f*x*z + 2.0f*y*w, 0.0f,
		2.0f*x*y + 2.0f*z*w, 1.0f - 2.0f*x*x - 2.0f*z*z, 2.0f*y*z - 2.0f*x*w, 0.0f,
		2.0f*x*z - 2.0f*y*w, 2.0f*y*z + 2.0f*x*w, 1.0f - 2.0f*x*x - 2.0f*y*y, 0.0f,
		0.0f, 0.0f, 0.0f, 1.0f
		)
*/

#endif // PID_H 