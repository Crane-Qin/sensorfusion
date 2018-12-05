#include "sensordata_to_vehicle_coordinate.h"
  

CoordTransform::CoordTransform(){}

CoordTransform::~CoordTransform(){}

Eigen::Quaterniond CoordTransform::EulerToQuaternion(
    const double& pitch_, const double& roll_, const double& yaw_) {
    
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

void CoordTransform::SensorDataToVehicle (const double &x_, const double &y_, const double &z_,
                                          const double &x_s, const double &y_s, const double &z_s,
                                          const Eigen::Matrix3d rotation_,
                                          double *x_v, double *y_v, double *z_v) {
  
  
  //data after translation
  double x = x_ + x_s;
  double y = y_ + y_s;
  double z = z_ + z_s;

  //raw sensor observations
  Eigen::Vector3d observation_trans;
  observation_trans << x, y, z;
  //transform raw sensor observations to vehcile
  Eigen::Vector3d observation_rot = rotation_ * observation_trans;
  *x_v = observation_rot[0];
  *y_v = observation_rot[1];
  *z_v = observation_rot[2];
                                          };