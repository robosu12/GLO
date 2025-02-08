/************************************************************************
 * Software License Agreement (BSD License)

 *@author Yun Su(robosu12@gmail.com)
 *@version 1.0
 *@data 2021-11-04
 ************************************************************************/
#ifndef CERES_FACTOR_HPP
#define CERES_FACTOR_HPP

#include <ceres/ceres.h>
#include <ceres/rotation.h>
#include <Eigen/Core>
#include <Eigen/Eigenvalues>
#include <Eigen/Geometry>
#include <cmath>
#include <algorithm>
#include "common/math_base/slam_math.hpp"

namespace GR_SLAM {

template <typename T>
inline void Quaternion2EulerAngle(const T q[4], T ypr[3]) {
  // roll (x-axis rotation)
  T sinr_cosp = T(2) * (q[0] * q[1] + q[2] * q[3]);
  T cosr_cosp = T(1) - T(2) * (q[1] * q[1] + q[2] * q[2]);
  ypr[2] = atan2(sinr_cosp, cosr_cosp);

  // pitch (y-axis rotation)
  T sinp = T(2) * (q[0] * q[2] - q[1] * q[3]);
  if (sinp >= T(1)) {
    ypr[1] = T(M_PI / 2);  // use 90 degrees if out of range
  } else if (sinp <= T(-1)) {
    ypr[1] = -T(M_PI / 2);  // use 90 degrees if out of range
  } else {
    ypr[1] = asin(sinp);
  }

  // yaw (z-axis rotation)
  T siny_cosp = T(2) * (q[0] * q[3] + q[1] * q[2]);
  T cosy_cosp = T(1) - T(2) * (q[2] * q[2] + q[3] * q[3]);
  ypr[0] = atan2(siny_cosp, cosy_cosp);
}

struct LidarPlaneNormFactor {
  LidarPlaneNormFactor(Eigen::Vector3d curr_point_, Eigen::Vector3d plane_unit_norm_,
                       double negative_OA_dot_norm_)
      : curr_point(curr_point_),
        plane_unit_norm(plane_unit_norm_),
        negative_OA_dot_norm(negative_OA_dot_norm_) {}

  template <typename T>
  bool operator()(const T *q, const T *t, T *residual) const {
    Eigen::Quaternion<T> q_w_curr{q[3], q[0], q[1], q[2]};
    Eigen::Matrix<T, 3, 1> t_w_curr{t[0], t[1], t[2]};
    Eigen::Matrix<T, 3, 1> cp{T(curr_point.x()), T(curr_point.y()), T(curr_point.z())};
    Eigen::Matrix<T, 3, 1> point_w;
    point_w = q_w_curr * cp + t_w_curr;

    Eigen::Matrix<T, 3, 1> norm(T(plane_unit_norm.x()), T(plane_unit_norm.y()),
                                T(plane_unit_norm.z()));
    residual[0] = norm.dot(point_w) + T(negative_OA_dot_norm);
    return true;
  }

  static ceres::CostFunction *Create(const Eigen::Vector3d curr_point_,
                                     const Eigen::Vector3d plane_unit_norm_,
                                     const double negative_OA_dot_norm_) {
    return (new ceres::AutoDiffCostFunction<LidarPlaneNormFactor, 1, 4, 3>(
        new LidarPlaneNormFactor(curr_point_, plane_unit_norm_, negative_OA_dot_norm_)));
  }

  Eigen::Vector3d curr_point;
  Eigen::Vector3d plane_unit_norm;
  double negative_OA_dot_norm;
};

struct LidarDistanceFactor {
  LidarDistanceFactor(Eigen::Vector3d curr_point_, Eigen::Vector3d closed_point_)
      : curr_point(curr_point_), closed_point(closed_point_) {}

  template <typename T>
  bool operator()(const T *q, const T *t, T *residual) const {
    Eigen::Quaternion<T> q_w_curr{q[3], q[0], q[1], q[2]};
    Eigen::Matrix<T, 3, 1> t_w_curr{t[0], t[1], t[2]};
    Eigen::Matrix<T, 3, 1> cp{T(curr_point.x()), T(curr_point.y()), T(curr_point.z())};
    Eigen::Matrix<T, 3, 1> point_w;
    point_w = q_w_curr * cp + t_w_curr;

    residual[0] = point_w.x() - T(closed_point.x());
    residual[1] = point_w.y() - T(closed_point.y());
    residual[2] = point_w.z() - T(closed_point.z());
    return true;
  }

  static ceres::CostFunction *Create(const Eigen::Vector3d curr_point_,
                                     const Eigen::Vector3d closed_point_) {
    return (new ceres::AutoDiffCostFunction<LidarDistanceFactor, 3, 4, 3>(
        new LidarDistanceFactor(curr_point_, closed_point_)));
  }

  Eigen::Vector3d curr_point;
  Eigen::Vector3d closed_point;
};

template <typename T>
inline void QuaternionInverse(const T q[4], T q_inverse[4]) {
  q_inverse[0] = q[0];
  q_inverse[1] = -q[1];
  q_inverse[2] = -q[2];
  q_inverse[3] = -q[3];
};

struct DeltaRFactor {
  DeltaRFactor(double q_x, double q_y, double q_z, double q_w, double q_var)
      : q_x(q_x), q_y(q_y), q_z(q_z), q_w(q_w), q_var(q_var) {}

  template <typename T>
  bool operator()(const T *q_i_j, T *residuals) const {
    T relative_q[4];
    relative_q[0] = T(q_w);  // ceres in w, x, y, z order
    relative_q[1] = T(q_x);
    relative_q[2] = T(q_y);
    relative_q[3] = T(q_z);

    T q_i_j_tmp[4];
    q_i_j_tmp[0] = q_i_j[3];  // ceres in w, x, y, z order
    q_i_j_tmp[1] = q_i_j[0];
    q_i_j_tmp[2] = q_i_j[1];
    q_i_j_tmp[3] = q_i_j[2];

    T relative_q_inv[4];
    QuaternionInverse(relative_q, relative_q_inv);

    T error_q[4];
    ceres::QuaternionProduct(relative_q_inv, q_i_j_tmp, error_q);

    residuals[0] = T(2) * error_q[1] / T(q_var);
    residuals[1] = T(2) * error_q[2] / T(q_var);
    residuals[2] = T(2) * error_q[3] / T(q_var);

    return true;
  }

  static ceres::CostFunction *Create(const double q_x, const double q_y, const double q_z,
                                     const double q_w, const double q_var) {
    return (new ceres::AutoDiffCostFunction<DeltaRFactor, 3, 4>(
        new DeltaRFactor(q_x, q_y, q_z, q_w, q_var)));
  }

  double q_x, q_y, q_z, q_w;
  double q_var;
};

struct RelativeRFactor {
  RelativeRFactor(double q_x, double q_y, double q_z, double q_w, double q_var)
      : q_x(q_x), q_y(q_y), q_z(q_z), q_w(q_w), q_var(q_var) {
    double d_q[4], ypr_measu[3];
    d_q[0] = q_w;
    d_q[1] = q_x;
    d_q[2] = q_y;
    d_q[3] = q_z;
    Quaternion2EulerAngle(d_q, ypr_measu);
    d_y = ypr_measu[0];
    d_p = ypr_measu[1];
    d_r = ypr_measu[2];
  }

  template <typename T>
  bool operator()(const T *const w_q_i, const T *w_q_j, T *residuals) const {
    T w_q_i_tmp[4], w_q_j_tmp[4];
    w_q_i_tmp[0] = w_q_i[3];  // ceres in w, x, y, z order
    w_q_i_tmp[1] = w_q_i[0];
    w_q_i_tmp[2] = w_q_i[1];
    w_q_i_tmp[3] = w_q_i[2];

    w_q_j_tmp[0] = w_q_j[3];  // ceres in w, x, y, z order
    w_q_j_tmp[1] = w_q_j[0];
    w_q_j_tmp[2] = w_q_j[1];
    w_q_j_tmp[3] = w_q_j[2];

    T i_q_w[4];
    QuaternionInverse(w_q_i_tmp, i_q_w);

    T q_i_j[4];
    ceres::QuaternionProduct(i_q_w, w_q_j_tmp, q_i_j);

    T relative_q[4];
    relative_q[0] = T(q_w);
    relative_q[1] = T(q_x);
    relative_q[2] = T(q_y);
    relative_q[3] = T(q_z);

    T relative_q_inv[4];
    QuaternionInverse(relative_q, relative_q_inv);

    T error_q[4];
    ceres::QuaternionProduct(relative_q_inv, q_i_j, error_q);

    residuals[0] = T(2) * error_q[1] / T(q_var);
    residuals[1] = T(2) * error_q[2] / T(q_var);
    residuals[2] = T(2) * error_q[3] / T(q_var);

    // T ypr_ij[3];
    // Quaternion2EulerAngle(q_i_j, ypr_ij);

    // T e[3];
    // e[0] = ypr_ij[0] - T(d_y);
    // e[1] = ypr_ij[1] - T(d_p);
    // e[2] = ypr_ij[2] - T(d_r);

    // residuals[0] = T(2) * e[0] / T(q_var);
    // residuals[1] = T(2) * e[1] / T(q_var);
    // residuals[2] = T(2) * e[2] / T(q_var);

    return true;
  }

  static ceres::CostFunction *Create(const double q_x, const double q_y, const double q_z,
                                     const double q_w, const double q_var) {
    return (new ceres::AutoDiffCostFunction<RelativeRFactor, 3, 4, 4>(
        new RelativeRFactor(q_x, q_y, q_z, q_w, q_var)));
  }

  double q_x, q_y, q_z, q_w, d_y, d_p, d_r;
  double q_var;
};

struct RelativeTFactor {
  RelativeTFactor(double t_x, double t_y, double t_z, double t_var)
      : t_x(t_x), t_y(t_y), t_z(t_z), t_var(t_var) {}

  template <typename T>
  bool operator()(const T *const w_q_i, const T *ti, const T *w_q_j, const T *tj,
                  T *residuals) const {
    T w_q_i_tmp[4];
    w_q_i_tmp[0] = w_q_i[3];  // ceres in w, x, y, z order
    w_q_i_tmp[1] = w_q_i[0];
    w_q_i_tmp[2] = w_q_i[1];
    w_q_i_tmp[3] = w_q_i[2];

    T t_w_ij[3];
    t_w_ij[0] = tj[0] - ti[0];
    t_w_ij[1] = tj[1] - ti[1];
    t_w_ij[2] = tj[2] - ti[2];

    T i_q_w[4];
    QuaternionInverse(w_q_i_tmp, i_q_w);

    T t_i_ij[3];
    ceres::QuaternionRotatePoint(i_q_w, t_w_ij, t_i_ij);

    residuals[0] = (t_i_ij[0] - T(t_x)) / T(t_var);
    residuals[1] = (t_i_ij[1] - T(t_y)) / T(t_var);
    residuals[2] = (t_i_ij[2] - T(t_z)) / T(t_var);

    return true;
  }

  static ceres::CostFunction *Create(const double t_x, const double t_y, const double t_z,
                                     const double t_var) {
    return (new ceres::AutoDiffCostFunction<RelativeTFactor, 3, 4, 3, 4, 3>(
        new RelativeTFactor(t_x, t_y, t_z, t_var)));
  }

  double t_x, t_y, t_z, t_norm;
  double q_x, q_y, q_z, q_w;
  double t_var, q_var;
};

struct RelativeRTFactor {
  RelativeRTFactor(double t_x, double t_y, double t_z, double q_x, double q_y, double q_z,
                   double q_w, double t_var, double q_var)
      : t_x(t_x),
        t_y(t_y),
        t_z(t_z),
        q_x(q_x),
        q_y(q_y),
        q_z(q_z),
        q_w(q_w),
        t_var(t_var),
        q_var(q_var) {}

  template <typename T>
  bool operator()(const T *const w_q_i, const T *ti, const T *w_q_j, const T *tj,
                  T *residuals) const {
    T w_q_i_tmp[4], w_q_j_tmp[4];
    w_q_i_tmp[0] = w_q_i[3];  // ceres in w, x, y, z order
    w_q_i_tmp[1] = w_q_i[0];
    w_q_i_tmp[2] = w_q_i[1];
    w_q_i_tmp[3] = w_q_i[2];

    w_q_j_tmp[0] = w_q_j[3];  // ceres in w, x, y, z order
    w_q_j_tmp[1] = w_q_j[0];
    w_q_j_tmp[2] = w_q_j[1];
    w_q_j_tmp[3] = w_q_j[2];

    T t_w_ij[3];
    t_w_ij[0] = tj[0] - ti[0];
    t_w_ij[1] = tj[1] - ti[1];
    t_w_ij[2] = tj[2] - ti[2];

    T i_q_w[4];
    QuaternionInverse(w_q_i_tmp, i_q_w);

    T t_i_ij[3];
    ceres::QuaternionRotatePoint(i_q_w, t_w_ij, t_i_ij);

    residuals[0] = (t_i_ij[0] - T(t_x)) / T(t_var);
    residuals[1] = (t_i_ij[1] - T(t_y)) / T(t_var);
    residuals[2] = (t_i_ij[2] - T(t_z)) / T(t_var);

    T relative_q[4];
    relative_q[0] = T(q_w);
    relative_q[1] = T(q_x);
    relative_q[2] = T(q_y);
    relative_q[3] = T(q_z);

    T q_i_j[4];
    ceres::QuaternionProduct(i_q_w, w_q_j_tmp, q_i_j);

    T relative_q_inv[4];
    QuaternionInverse(relative_q, relative_q_inv);

    T error_q[4];
    ceres::QuaternionProduct(relative_q_inv, q_i_j, error_q);

    residuals[3] = T(2) * error_q[1] / T(q_var);
    residuals[4] = T(2) * error_q[2] / T(q_var);
    residuals[5] = T(2) * error_q[3] / T(q_var);

    return true;
  }

  static ceres::CostFunction *Create(const double t_x, const double t_y, const double t_z,
                                     const double q_x, const double q_y, const double q_z,
                                     const double q_w, const double t_var, const double q_var) {
    return (new ceres::AutoDiffCostFunction<RelativeRTFactor, 6, 4, 3, 4, 3>(
        new RelativeRTFactor(t_x, t_y, t_z, q_x, q_y, q_z, q_w, t_var, q_var)));
  }

  double t_x, t_y, t_z, t_norm;
  double q_x, q_y, q_z, q_w;
  double t_var, q_var;
};

struct PitchRollFactor {
  PitchRollFactor(double p, double r, double q_var) : p(p), r(r), q_var(q_var) {}

  template <typename T>
  bool operator()(const T *const q_i, T *residuals) const {
    T q_i_tmp[4];
    q_i_tmp[0] = q_i[3];  // ceres in w, x, y, z order
    q_i_tmp[1] = q_i[0];
    q_i_tmp[2] = q_i[1];
    q_i_tmp[3] = q_i[2];

    T ypr[3];
    Quaternion2EulerAngle(q_i_tmp, ypr);

    T e[2];
    e[0] = ypr[1] - T(p);
    e[1] = ypr[2] - T(r);

    residuals[0] = T(2) * e[0] / T(q_var);
    residuals[1] = T(2) * e[1] / T(q_var);

    return true;
  }

  static ceres::CostFunction *Create(const double p, const double r, const double q_var) {
    return (
        new ceres::AutoDiffCostFunction<PitchRollFactor, 2, 4>(new PitchRollFactor(p, r, q_var)));
  }

  double p, r;
  double q_var;
};

struct RFactor {
  RFactor(double q_x, double q_y, double q_z, double q_w, double q_var)
      : q_x(q_x), q_y(q_y), q_z(q_z), q_w(q_w), q_var(q_var) {}

  template <typename T>
  bool operator()(const T *const q_i, T *residuals) const {
    T q_i_tmp[4];
    q_i_tmp[0] = q_i[3];  // ceres in w, x, y, z order
    q_i_tmp[1] = q_i[0];
    q_i_tmp[2] = q_i[1];
    q_i_tmp[3] = q_i[2];

    T q_m[4];
    q_m[0] = T(q_w);
    q_m[1] = T(q_x);
    q_m[2] = T(q_y);
    q_m[3] = T(q_z);

    T q_m_inv[4];
    QuaternionInverse(q_m, q_m_inv);

    T error_q[4];
    ceres::QuaternionProduct(q_m_inv, q_i_tmp, error_q);

    residuals[0] = T(2) * error_q[1] / T(q_var);
    residuals[1] = T(2) * error_q[2] / T(q_var);
    residuals[2] = T(2) * error_q[3] / T(q_var);

    return true;
  }

  static ceres::CostFunction *Create(const double q_x, const double q_y, const double q_z,
                                     const double q_w, const double q_var) {
    return (new ceres::AutoDiffCostFunction<RFactor, 3, 4>(new RFactor(q_x, q_y, q_z, q_w, q_var)));
  }

  double q_x, q_y, q_z, q_w;
  double q_var;
};

struct TFactor {
  TFactor(double t_x, double t_y, double t_z, double var)
      : t_x(t_x), t_y(t_y), t_z(t_z), var(var) {}

  template <typename T>
  bool operator()(const T *tj, T *residuals) const {
    residuals[0] = (tj[0] - T(t_x)) / T(var);
    residuals[1] = (tj[1] - T(t_y)) / T(var);
    residuals[2] = (tj[2] - T(t_z)) / T(var);

    return true;
  }

  static ceres::CostFunction *Create(const double t_x, const double t_y, const double t_z,
                                     const double var) {
    return (new ceres::AutoDiffCostFunction<TFactor, 3, 3>(new TFactor(t_x, t_y, t_z, var)));
  }

  double t_x, t_y, t_z, var;
};

struct GroundFactor {
  GroundFactor(double var) : var(var) {}

  template <typename T>
  bool operator()(const T *ti, const T *tj, T *residuals) const {
    residuals[0] = (ti[2] - tj[2]) / T(var);

    return true;
  }

  static ceres::CostFunction *Create(const double var) {
    return (new ceres::AutoDiffCostFunction<GroundFactor, 1, 3, 3>(new GroundFactor(var)));
  }

  double t_x, t_y, t_z, var;
};

// pose graph factor
////////////////////////////////////////////////////////////

template <typename T>
T NormalizeAngle(const T &angle_degrees) {
  if (angle_degrees > T(180.0))
    return angle_degrees - T(360.0);
  else if (angle_degrees < T(-180.0))
    return angle_degrees + T(360.0);
  else
    return angle_degrees;
};

class AngleLocalParameterization {
 public:
  template <typename T>
  bool operator()(const T *theta_radians, const T *delta_theta_radians,
                  T *theta_radians_plus_delta) const {
    *theta_radians_plus_delta = NormalizeAngle(*theta_radians + *delta_theta_radians);

    return true;
  }

  static ceres::LocalParameterization *Create() {
    return (new ceres::AutoDiffLocalParameterization<AngleLocalParameterization, 1, 1>);
  }
};

template <typename T>
void YawPitchRollToRotationMatrix(const T yaw, const T pitch, const T roll, T R[9]) {
  T y = yaw / T(180.0) * T(M_PI);
  T p = pitch / T(180.0) * T(M_PI);
  T r = roll / T(180.0) * T(M_PI);

  R[0] = cos(y) * cos(p);
  R[1] = -sin(y) * cos(r) + cos(y) * sin(p) * sin(r);
  R[2] = sin(y) * sin(r) + cos(y) * sin(p) * cos(r);
  R[3] = sin(y) * cos(p);
  R[4] = cos(y) * cos(r) + sin(y) * sin(p) * sin(r);
  R[5] = -cos(y) * sin(r) + sin(y) * sin(p) * cos(r);
  R[6] = -sin(p);
  R[7] = cos(p) * sin(r);
  R[8] = cos(p) * cos(r);
};

template <typename T>
void RotationMatrixTranspose(const T R[9], T inv_R[9]) {
  inv_R[0] = R[0];
  inv_R[1] = R[3];
  inv_R[2] = R[6];
  inv_R[3] = R[1];
  inv_R[4] = R[4];
  inv_R[5] = R[7];
  inv_R[6] = R[2];
  inv_R[7] = R[5];
  inv_R[8] = R[8];
};

template <typename T>
void RotationMatrixRotatePoint(const T R[9], const T t[3], T r_t[3]) {
  r_t[0] = R[0] * t[0] + R[1] * t[1] + R[2] * t[2];
  r_t[1] = R[3] * t[0] + R[4] * t[1] + R[5] * t[2];
  r_t[2] = R[6] * t[0] + R[7] * t[1] + R[8] * t[2];
};

struct FourDOFError {
  FourDOFError(double t_x, double t_y, double t_z, double relative_yaw, double pitch_i,
               double roll_i)
      : t_x(t_x),
        t_y(t_y),
        t_z(t_z),
        relative_yaw(relative_yaw),
        pitch_i(pitch_i),
        roll_i(roll_i) {}

  template <typename T>
  bool operator()(const T *const yaw_i, const T *ti, const T *yaw_j, const T *tj,
                  T *residuals) const {
    T t_w_ij[3];
    t_w_ij[0] = tj[0] - ti[0];
    t_w_ij[1] = tj[1] - ti[1];
    t_w_ij[2] = tj[2] - ti[2];

    // euler to rotation
    T w_R_i[9];
    YawPitchRollToRotationMatrix(yaw_i[0], T(pitch_i), T(roll_i), w_R_i);
    // rotation transpose
    T i_R_w[9];
    RotationMatrixTranspose(w_R_i, i_R_w);
    // rotation matrix rotate point
    T t_i_ij[3];
    RotationMatrixRotatePoint(i_R_w, t_w_ij, t_i_ij);

    residuals[0] = (t_i_ij[0] - T(t_x));
    residuals[1] = (t_i_ij[1] - T(t_y));
    residuals[2] = (t_i_ij[2] - T(t_z));
    residuals[3] = NormalizeAngle(yaw_j[0] - yaw_i[0] - T(relative_yaw));

    return true;
  }

  static ceres::CostFunction *Create(const double t_x, const double t_y, const double t_z,
                                     const double relative_yaw, const double pitch_i,
                                     const double roll_i) {
    return (new ceres::AutoDiffCostFunction<FourDOFError, 4, 1, 3, 1, 3>(
        new FourDOFError(t_x, t_y, t_z, relative_yaw, pitch_i, roll_i)));
  }

  double t_x, t_y, t_z;
  double relative_yaw, pitch_i, roll_i;
};

struct FourDOFWeightError {
  FourDOFWeightError(double t_x, double t_y, double t_z, double relative_yaw, double pitch_i,
                     double roll_i, double weight)
      : t_x(t_x),
        t_y(t_y),
        t_z(t_z),
        relative_yaw(relative_yaw),
        pitch_i(pitch_i),
        roll_i(roll_i),
        weight(weight) {
    // weight = 1;
  }

  template <typename T>
  bool operator()(const T *const yaw_i, const T *ti, const T *yaw_j, const T *tj,
                  T *residuals) const {
    T t_w_ij[3];
    t_w_ij[0] = tj[0] - ti[0];
    t_w_ij[1] = tj[1] - ti[1];
    t_w_ij[2] = tj[2] - ti[2];

    // euler to rotation
    T w_R_i[9];
    YawPitchRollToRotationMatrix(yaw_i[0], T(pitch_i), T(roll_i), w_R_i);
    // rotation transpose
    T i_R_w[9];
    RotationMatrixTranspose(w_R_i, i_R_w);
    // rotation matrix rotate point
    T t_i_ij[3];
    RotationMatrixRotatePoint(i_R_w, t_w_ij, t_i_ij);

    residuals[0] = (t_i_ij[0] - T(t_x)) * T(weight);
    residuals[1] = (t_i_ij[1] - T(t_y)) * T(weight);
    residuals[2] = (t_i_ij[2] - T(t_z)) * T(weight);
    // residuals[3] = NormalizeAngle((yaw_j[0] - yaw_i[0] - T(relative_yaw))) *
    // T(weight) / T(10.0);
    residuals[3] = NormalizeAngle((yaw_j[0] - yaw_i[0] - T(relative_yaw))) * T(weight);

    return true;
  }

  static ceres::CostFunction *Create(const double t_x, const double t_y, const double t_z,
                                     const double relative_yaw, const double pitch_i,
                                     const double roll_i, const double weight) {
    return (new ceres::AutoDiffCostFunction<FourDOFWeightError, 4, 1, 3, 1, 3>(
        new FourDOFWeightError(t_x, t_y, t_z, relative_yaw, pitch_i, roll_i, weight)));
  }

  double t_x, t_y, t_z;
  double relative_yaw, pitch_i, roll_i;
  double weight;
};

}  // namespace GR_SLAM
#endif