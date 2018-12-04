#include "euler_to_quaternion.h"
  

CoordTransform::CoordTransform(){}

CoordTransform::~CoordTransform(){}

Eigen::Quaterniond CoordTransform::EulerToQuaternion(const double& pitch_, const double& roll_, const double& yaw_) {
    
    double r = roll_ * 0.5;
    double p = pitch_ * 0.5;
    double y = yaw_ * 0.5;

    double sr = std::sin(r);
    double sp = std::sin(p);
    double sy = std::sin(y);

    double cr = std::cos(r);
    double cp = std::cos(p);
    double cy = std::cos(y);

   Eigen::Quaterniond quaternion_;
    quaternion_.w() = cr * cp * cy + sr * sp * sy;
    quaternion_.x() = cr * sp * cy - sr * cp * sy;
    quaternion_.y() = sr * cp * cy + cr * sp * sy;
    quaternion_.z() = cr * cp * sy - sr * sp * cy;
    return quaternion_;
    /*
    Eigen::Quaterniond q;
    q.x() = cr * cp * sy - sr * sp * cy;
    q.y() = cr * sp * cy - sr * cp * sy;
    q.z() = sr * cp * cy + cr * sp * sy;
    q.w() = cr * cp * cy + sr * sp * sy;
    return q;*/
}
 