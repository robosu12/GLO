/************************************************************************
 * Software License Agreement (BSD License)

 *@author Yun Su(robosu12@gmail.com)
 *@version 1.0
 *@data 2024-08-05
 ************************************************************************/
#ifndef POSE_GRAPH_ERROR_TERM_HPP
#define POSE_GRAPH_ERROR_TERM_HPP

#include <ceres/ceres.h>
#include <ceres/rotation.h>
#include "common/math_base/slam_math.hpp"

namespace GR_SLAM {
/**
 * PosePara
 * @brief pose本地参数化类
 **/
class PosePara : public ceres::LocalParameterization {
 public:
  PosePara() noexcept = default;
  virtual ~PosePara() = default;

  virtual bool Plus(const double *x, const double *delta, double *x_plus_delta) const override {
    Eigen::Map<const Mat3d, 0, Eigen::OuterStride<4>> R(x, 3, 3);
    Eigen::Map<const Vec3d, 0, Eigen::InnerStride<4>> p(x + 3, 3);
    Eigen::Map<const Vec3d> dr(delta, 3);
    Eigen::Map<const Vec3d> dp(delta + 3, 3);
    Eigen::Map<Mat34d> T_se3(x_plus_delta);
    if (isTraditional) {
      Mat3d dR = Mathbox::Exp3d(dr);
      T_se3.block<3, 3>(0, 0) = R * dR;
      T_se3.block<3, 1>(0, 3) = p + dp;
    } else {
      // Mat3d dR, Jl;
      // Exp_SO3(dr, dR, Jl  );
      // T_se3.block<3,3>(0,0)  =  dR * R;
      // T_se3.block<3,1>(0,3) =   dR * p + Jl*dp;
    }
    return true;
  }
  virtual bool ComputeJacobian(const double *x, double *jacobian) const override {
    Eigen::Map<Mat126d> Jacobian(jacobian);
    Jacobian.setZero();
    Jacobian.block<6, 6>(0, 0) = Mat6d::Identity();
    if (this->isTraditional) {
      Jacobian.block<6, 6>(0, 0) = Mat6d::Identity();
    } else {
      Jacobian.block<6, 6>(0, 0) = Mat6d::Identity();
      Eigen::Map<const Vec3d, 0, Eigen::InnerStride<4>> p(x + 3, 3);
      Jacobian.block<3, 3>(3, 0) = -Mathbox::skew(p);
    }
    return true;
  }
  virtual int GlobalSize() const override { return 12; }
  virtual int LocalSize() const override { return 6; }

  bool isTraditional = true;
};

/**
 * RelativeFactor
 * @brief 相对2d pose的factor类
 **/
class OdomRelativeFactor : public ceres::SizedCostFunction<6,   // number of residuals
                                                           12,  // pose1
                                                           12>  // pose2
{
 public:
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW
  OdomRelativeFactor() = delete;
  OdomRelativeFactor(const Mat34d dT, const Mat6d SqrtInf) noexcept : mdT(dT), mSqrtInf(SqrtInf) {}
  virtual ~OdomRelativeFactor() = default;
  virtual size_t parameterBlocks() const { return 2; }
  virtual bool Evaluate(double const *const *parameters, double *residuals,
                        double **jacobians) const override {
    Eigen::Map<const Mat34d> pose1(parameters[0], 3, 4);
    Eigen::Map<const Mat34d> pose2(parameters[1], 3, 4);
    Eigen::Map<Vec6d> resVec(residuals);

    const Mat3d &dR = mdT.block<3, 3>(0, 0);
    const Mat3d &R1 = pose1.block<3, 3>(0, 0);
    const Mat3d &R2 = pose2.block<3, 3>(0, 0);
    const Vec3d &dt = mdT.block<3, 1>(0, 3);
    const Vec3d &t1 = pose1.block<3, 1>(0, 3);
    const Vec3d &t2 = pose2.block<3, 1>(0, 3);

    Vec3d rotion_err = Mathbox::Log(R2.transpose() * R1 * dR);
    Vec3d translation_err = R2.transpose() * (R1 * dt - (t2 - t1));
    Vec6d all_err;
    all_err << rotion_err, translation_err;

    resVec = weighting * mSqrtInf * all_err;

    if (jacobians != nullptr) {
      if (jacobians[0]) {
        Eigen::Map<Eigen::Matrix<double, 6, 12, Eigen::RowMajor>> J0(jacobians[0]);
        J0.setZero();
        J0.block<3, 3>(0, 0) = Mathbox::Jr_inv(rotion_err) * (R1 * dR).transpose();
        J0.block<3, 3>(3, 0) = -R2.transpose() * Mathbox::skew(R1 * dt);
        J0.block<3, 3>(3, 3) = R2.transpose();
        J0 = weighting * mSqrtInf * J0;
        // std::cout << "J0：  " << J0 << std::endl;
      }

      if (jacobians[1]) {
        Eigen::Map<Eigen::Matrix<double, 6, 12, Eigen::RowMajor>> J1(jacobians[1]);
        J1.setZero();
        J1.block<3, 3>(0, 0) = -Mathbox::Jr_inv(rotion_err) * (R1 * dR).transpose();
        J1.block<3, 3>(3, 0) = R2.transpose() * Mathbox::skew(R1 * dt - (t2 - t1));
        J1.block<3, 3>(3, 3) = -R2.transpose();
        J1 = weighting * mSqrtInf * J1;
        // std::cout << "J1：  " << J1 << std::endl;
      }
    }

    return true;
  }

  Mat34d mdT;
  Mat6d mSqrtInf;
  double weighting = 1.0;
};

class LoopRelativeFactor : public ceres::SizedCostFunction<6,   // number of residuals
                                                           12,  // pose1
                                                           12>  // pose2
{
 public:
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW
  LoopRelativeFactor() = delete;
  LoopRelativeFactor(const Mat34d dT, const Mat6d SqrtInf) noexcept : mdT(dT), mSqrtInf(SqrtInf) {}
  virtual ~LoopRelativeFactor() = default;
  virtual size_t parameterBlocks() const { return 2; }
  virtual bool Evaluate(double const *const *parameters, double *residuals,
                        double **jacobians) const override {
    Eigen::Map<const Mat34d> pose1(parameters[0], 3, 4);
    Eigen::Map<const Mat34d> pose2(parameters[1], 3, 4);
    Eigen::Map<Vec6d> resVec(residuals);

    const Mat3d &dR = mdT.block<3, 3>(0, 0);
    const Mat3d &R1 = pose1.block<3, 3>(0, 0);
    const Mat3d &R2 = pose2.block<3, 3>(0, 0);
    const Vec3d &dt = mdT.block<3, 1>(0, 3);
    const Vec3d &t1 = pose1.block<3, 1>(0, 3);
    const Vec3d &t2 = pose2.block<3, 1>(0, 3);

    Vec3d rotion_err = Mathbox::Log(R2.transpose() * R1 * dR);
    Vec3d translation_err = R2.transpose() * (R1 * dt - (t2 - t1));
    Vec6d all_err;
    all_err << rotion_err, translation_err;

    Mat6d s_mat;
    // s_mat.setZero();
    // for(int i = 0;i < 3;i++){
    //   double s = 1 / (0.5 + all_err(i)*all_err(i));
    //   s_mat(i,i) = s < 1.0 ? s : 1;
    // }
    for (int i = 0; i < 6; i++) {
      double s = 3 / (3 + std::fabs(all_err(i)));
      s_mat(i, i) = s < 1.0 ? s : 1;
    }

    resVec = weighting * mSqrtInf * all_err;

    if (jacobians != nullptr) {
      if (jacobians[0]) {
        Eigen::Map<Eigen::Matrix<double, 6, 12, Eigen::RowMajor>> J0(jacobians[0]);
        J0.setZero();
        J0.block<3, 3>(0, 0) = Mathbox::Jr_inv(rotion_err) * (R1 * dR).transpose();
        J0.block<3, 3>(3, 0) = -R2.transpose() * Mathbox::skew(R1 * dt);
        J0.block<3, 3>(3, 3) = R2.transpose();
        J0 = weighting * mSqrtInf * J0;
        // std::cout << "J0：  " << J0 << std::endl;
      }

      if (jacobians[1]) {
        Eigen::Map<Eigen::Matrix<double, 6, 12, Eigen::RowMajor>> J1(jacobians[1]);
        J1.setZero();
        J1.block<3, 3>(0, 0) = -Mathbox::Jr_inv(rotion_err) * (R1 * dR).transpose();
        J1.block<3, 3>(3, 0) = R2.transpose() * Mathbox::skew(R1 * dt - (t2 - t1));
        J1.block<3, 3>(3, 3) = -R2.transpose();
        J1 = weighting * mSqrtInf * J1;
        // std::cout << "J1：  " << J1 << std::endl;
      }
    }

    return true;
  }

  Mat34d mdT;
  Mat6d mSqrtInf;
  double weighting = 1.0;
};

/**
 * RelativeWithExFactor
 * @brief 相对pose的factor类(增加外参估计)
 **/
class RelativeWithExFactor : public ceres::SizedCostFunction<6, 12, 12, 12> {
 public:
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW
  RelativeWithExFactor(const Mat34d &T1, const Mat6d &information)
      : T_odom(T1), information(information) {}
  virtual ~RelativeWithExFactor() {}
  virtual bool Evaluate(double const *const *parameters, double *residuals,
                        double **jacobians) const {
    Eigen::Map<const Mat34d> T_ex(parameters[0], 3, 4);
    Eigen::Map<const Mat34d> T_ci(parameters[1], 3, 4);
    Eigen::Map<const Mat34d> T_cj(parameters[2], 3, 4);
    Eigen::Map<Vec6d> resVec(residuals);

    const Mat3d &R_ci = T_ci.block<3, 3>(0, 0);
    const Mat3d &R_cj = T_cj.block<3, 3>(0, 0);
    const Mat3d &dR_o = T_odom.block<3, 3>(0, 0);
    const Mat3d &R_co = T_ex.block<3, 3>(0, 0);
    const Vec3d &t_ci = T_ci.block<3, 1>(0, 3);
    const Vec3d &t_cj = T_cj.block<3, 1>(0, 3);
    const Vec3d &dt_o = T_odom.block<3, 1>(0, 3);
    const Vec3d &t_co = T_ex.block<3, 1>(0, 3);
    const Mat3d &dR_x = R_co * dR_o * R_co.transpose();

    Vec3d rotion_err = Mathbox::Log(R_cj.transpose() * R_ci * dR_x);
    Vec3d translation_err =
        R_cj.transpose() * (R_ci * (-dR_x * t_co + R_co * dt_o + t_co) - (t_cj - t_ci));
    Vec6d all_err;
    all_err << rotion_err, translation_err;
    all_err = information * all_err;
    resVec = all_err;
    if (jacobians != nullptr) {
      if (jacobians[0]) {
        Eigen::Map<Eigen::Matrix<double, 6, 12, Eigen::RowMajor>> J0(jacobians[0]);
        J0.setZero();
        J0.block<3, 3>(0, 0) =
            Mathbox::Jr_inv(rotion_err) * (dR_x.transpose() - Eigen::Matrix3d::Identity());
        J0.block<3, 3>(3, 0) =
            R_cj.transpose() * R_ci *
            (Mathbox::skew(dR_x * t_co) - dR_x * Mathbox::skew(t_co) - Mathbox::skew(R_co * dt_o));
        J0.block<3, 3>(3, 3) = R_cj.transpose() * R_ci * (-dR_x + Eigen::Matrix3d::Identity());
        J0 = information * J0;
      }
      if (jacobians[1]) {
        Eigen::Map<Eigen::Matrix<double, 6, 12, Eigen::RowMajor>> J1(jacobians[1]);
        J1.setZero();
        J1.block<3, 3>(0, 0) = Mathbox::Jr_inv(rotion_err) * (R_ci * dR_x).transpose();
        J1.block<3, 3>(3, 0) =
            -R_cj.transpose() * Mathbox::skew(R_ci * (-dR_x * t_co + R_co * dt_o + t_co));
        J1.block<3, 3>(3, 3) = R_cj.transpose();
        J1 = information * J1;
      }
      if (jacobians[2]) {
        Eigen::Map<Eigen::Matrix<double, 6, 12, Eigen::RowMajor>> J2(jacobians[2]);
        J2.setZero();
        J2.block<3, 3>(0, 0) = -Mathbox::Jr_inv(rotion_err) * (R_ci * dR_x).transpose();
        J2.block<3, 3>(3, 0) =
            R_cj.transpose() *
            Mathbox::skew(R_ci * (-dR_x * t_co + R_co * dt_o + t_co) - (t_cj - t_ci));
        J2.block<3, 3>(3, 3) = -R_cj.transpose();
        J2 = information * J2;
      }
    }

    return true;
  }

 private:
  Mat34d T_odom;
  Mat6d information;

};  // class

/**
 * MapFactor
 * @brief 地图pose的factor类
 **/
class MapFactor : public ceres::SizedCostFunction<6, 12> {
 public:
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW
  MapFactor(const Mat34d &T1, const Mat6d &information) : T_map(T1), information(information) {}
  virtual ~MapFactor() {}
  virtual bool Evaluate(double const *const *parameters, double *residuals,
                        double **jacobians) const {
    Eigen::Map<const Mat34d> T_ci(parameters[0], 3, 4);
    Eigen::Map<Vec6d> resVec(residuals);

    const Mat3d &R_ci = T_ci.block<3, 3>(0, 0);
    const Mat3d &R_mi = T_map.block<3, 3>(0, 0);
    const Vec3d &t_ci = T_ci.block<3, 1>(0, 3);
    const Vec3d &t_mi = T_map.block<3, 1>(0, 3);

    Vec3d rotion_err = Mathbox::Log(R_ci.transpose() * R_mi);
    Vec3d translation_err = R_ci.transpose() * (t_mi - t_ci);
    Vec6d all_err;
    all_err << rotion_err, translation_err;
    all_err = information * all_err;
    resVec = all_err;
    if (jacobians != nullptr) {
      if (jacobians[0]) {
        Eigen::Map<Eigen::Matrix<double, 6, 12, Eigen::RowMajor>> J0(jacobians[0]);
        J0.setZero();
        J0.block<3, 3>(0, 0) = -Mathbox::Jr_inv(rotion_err) * R_mi.transpose();
        J0.block<3, 3>(3, 0) = R_ci.transpose() * Mathbox::skew(t_mi - t_ci);
        J0.block<3, 3>(3, 3) = -R_ci.transpose();
        J0 = information * J0;
      }
    }

    return true;
  }

 private:
  Mat34d T_map;
  Mat6d information;

};  // class

/**
 * GPSExtrincsFactorO
 * @brief GPS坐标的factor类,包含外参优化
 **/
struct GPSExtrincsFactorO {
  GPSExtrincsFactorO(const Vec3d &g_pos, const Vec3d &m_pos, const Mat3d &information)
      : g_pos(g_pos), m_pos(m_pos), information(information) {}

  template <typename T>
  bool operator()(const T *q_ex, const T *t_ex, T *residual) const {
    Eigen::Quaternion<T> q_ex_curr{q_ex[3], q_ex[0], q_ex[1], q_ex[2]};
    Eigen::Matrix<T, 3, 1> t_ex_curr{t_ex[0], t_ex[1], t_ex[2]};
    Eigen::Matrix<T, 3, 1> point_m{T(m_pos.x()), T(m_pos.y()), T(m_pos.z())};
    Eigen::Matrix<T, 3, 1> point_g_w{T(g_pos.x()), T(g_pos.y()), T(g_pos.z())};
    Eigen::Matrix<T, 3, 1> point_g_loacl;
    point_g_loacl = q_ex_curr * point_g_w + t_ex_curr;

    residual[0] = (point_g_loacl[0] - point_m[0]) * information(0, 0);
    residual[1] = (point_g_loacl[1] - point_m[1]) * information(1, 1);
    residual[2] = (point_g_loacl[2] - point_m[2]) * information(2, 2);

    return true;
  }

  static ceres::CostFunction *Create(const Vec3d &g_pos, const Vec3d &m_pos,
                                     const Mat3d &information) {
    return (new ceres::AutoDiffCostFunction<GPSExtrincsFactorO, 3, 4, 3>(
        new GPSExtrincsFactorO(g_pos, m_pos, information)));
  }

 private:
  Vec3d g_pos;
  Vec3d m_pos;
  Mat3d information;
};

/**
 * GPSJointExtrincsFactor
 * @brief GPS坐标的factor类,包含外参优化
 **/
struct GPSJointExtrincsFactor {
  GPSJointExtrincsFactor(const Vec3d &g_pos, const Mat3d &information)
      : g_pos(g_pos), information(information) {}

  template <typename T>
  bool operator()(const T *t, const T *q_ex, const T *t_ex, T *residual) const {
    Eigen::Quaternion<T> q_ex_curr{q_ex[3], q_ex[0], q_ex[1], q_ex[2]};
    Eigen::Matrix<T, 3, 1> t_ex_curr{t_ex[0], t_ex[1], t_ex[2]};
    Eigen::Matrix<T, 3, 1> t_curr{t[0], t[1], t[2]};
    Eigen::Matrix<T, 3, 1> point_g_w{T(g_pos.x()), T(g_pos.y()), T(g_pos.z())};
    Eigen::Matrix<T, 3, 1> point_g_loacl;
    point_g_loacl = q_ex_curr * point_g_w + t_ex_curr;

    residual[0] = (point_g_loacl[0] - t_curr[0]) * information(0, 0);
    residual[1] = (point_g_loacl[1] - t_curr[1]) * information(1, 1);
    residual[2] = (point_g_loacl[2] - t_curr[2]) * information(2, 2);

    return true;
  }

  static ceres::CostFunction *Create(const Vec3d &g_pos, const Mat3d &information) {
    return (new ceres::AutoDiffCostFunction<GPSJointExtrincsFactor, 3, 3, 4, 3>(
        new GPSJointExtrincsFactor(g_pos, information)));
  }

 private:
  Vec3d g_pos;
  Mat3d information;
};

class GPSFactor : public ceres::SizedCostFunction<3, 12, 12> {
 public:
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW
  GPSFactor(const Vec3d &position, const Mat3d &information)
      : position(position), information(information) {}
  virtual ~GPSFactor() {}
  virtual bool Evaluate(double const *const *parameters, double *residuals,
                        double **jacobians) const {
    Eigen::Map<const Mat34d> T_ci(parameters[0], 3, 4);
    Eigen::Map<const Mat34d> T_ex(parameters[1], 3, 4);
    const Mat3d &R_ex = T_ex.block<3, 3>(0, 0);
    const Vec3d &t_ex = T_ex.block<3, 1>(0, 3);

    Eigen::Map<Vec3d> resVec(residuals);

    resVec = T_ci.block<3, 1>(0, 3) - R_ex * position - t_ex;
    resVec = information * resVec;
    if (jacobians != nullptr) {
      if (jacobians[0]) {
        Eigen::Map<Eigen::Matrix<double, 3, 12, Eigen::RowMajor>> J0(jacobians[0]);
        J0.setZero();
        J0.block<3, 3>(0, 3) = information * Mat3d::Identity();
      }
      if (jacobians[1]) {
        Eigen::Map<Eigen::Matrix<double, 3, 12, Eigen::RowMajor>> J1(jacobians[1]);
        J1.setZero();
        J1.block<3, 3>(0, 0) = information * Mathbox::skew(R_ex * position);
        J1.block<3, 3>(0, 3) = -information * Mat3d::Identity();
      }
    }

    return true;
  }

 private:
  Vec3d position;
  Mat3d information;

};  // class

struct UwbRangeRelativeFactor {
  UwbRangeRelativeFactor(const double range, const Mat34d &relative_pose, const double weights)
      : range(range), relative_pose(relative_pose), weights(weights) {}

  template <typename T>
  bool operator()(const T *k, const T *a, T *residual) const {
    Eigen::Matrix<T, 3, 1> keyframe_pos{k[3], k[7], k[11]};
    Eigen::Matrix<T, 3, 1> anchor_pos{a[0], a[1], a[2]};

    const Mat3d &R_c = relative_pose.block<3, 3>(0, 0);
    const Vec3d &t_c = relative_pose.block<3, 1>(0, 3);

    Eigen::Matrix<T, 3, 1> c_pos;
    c_pos = R_c * keyframe_pos + t_c;

    residual[0] = (T(range) - T((anchor_pos - c_pos).norm())) * T(weights);

    return true;
  }

  static ceres::CostFunction *Create(const double range, const Mat34d &relative_pose,
                                     const double weights) {
    return (new ceres::AutoDiffCostFunction<UwbRangeRelativeFactor, 1, 12, 3>(
        new UwbRangeRelativeFactor(range, relative_pose, weights)));
  }

 private:
  double range, weights;
  Mat34d relative_pose;
};

/**
 * GPSExtrincsFactor
 * @brief GPS外参的factor类
 **/
class GPSExtrincsFactor : public ceres::SizedCostFunction<3, 12> {
 public:
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW
  GPSExtrincsFactor(const Vec3d &position, const Vec3d &lidar_position, const Mat3d &information)
      : position(position), lidar_position(lidar_position), information(information) {}
  virtual ~GPSExtrincsFactor() {}
  virtual bool Evaluate(double const *const *parameters, double *residuals,
                        double **jacobians) const {
    Eigen::Map<const Mat34d> T_ex(parameters[0], 3, 4);
    const Mat3d &R_ex = T_ex.block<3, 3>(0, 0);
    const Vec3d &t_ex = T_ex.block<3, 1>(0, 3);

    Eigen::Map<Vec3d> resVec(residuals);

    resVec = lidar_position - R_ex * position - t_ex;
    resVec = information * resVec;

    if (jacobians != nullptr) {
      if (jacobians[0]) {
        Eigen::Map<Eigen::Matrix<double, 3, 12, Eigen::RowMajor>> J0(jacobians[0]);
        J0.setZero();
        J0.block<3, 3>(0, 0) = Mathbox::skew(R_ex * position);
        J0.block<3, 3>(0, 3) = -Mat3d::Identity();
        J0 = information * J0;
      }
    }

    return true;
  }

 private:
  Vec3d position;
  Vec3d lidar_position;
  Mat3d information;

};  // class

/**
 * GPSFactor
 * @brief GPS坐标的factor类
 **/
class GPS2dFactor : public ceres::SizedCostFunction<3, 12> {
 public:
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW
  GPS2dFactor(const Vec3d &position, const Mat3d &information)
      : position(position), information(information) {}
  virtual ~GPS2dFactor() {}
  virtual bool Evaluate(double const *const *parameters, double *residuals,
                        double **jacobians) const {
    Eigen::Map<const Mat34d> P_ci(parameters[0], 3, 4);
    Eigen::Map<Vec3d> resVec(residuals);

    resVec = P_ci.block<3, 1>(0, 3) - position;
    resVec = information * resVec;
    if (jacobians != nullptr) {
      if (jacobians[0]) {
        Eigen::Map<Eigen::Matrix<double, 3, 12, Eigen::RowMajor>> J0(jacobians[0]);
        J0.setZero();
        J0.block<3, 3>(0, 3) = information * Mat3d::Identity();
      }
    }

    return true;
  }

 private:
  Vec3d position;
  Mat3d information;

};  // end of class

/**
 * PlaneFactor
 * @brief 激光面点的factor类
 **/
class PlaneFactor : public ceres::SizedCostFunction<1, 12> {
 public:
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW
  PlaneFactor(const Vec3d &origin_point, const Vec3d &plane_norm, const double scale,
              double const const_value)
      : origin_point(origin_point),
        plane_norm(plane_norm),
        scale(scale),
        const_value(const_value) {}
  virtual ~PlaneFactor() {}
  virtual bool Evaluate(double const *const *parameters, double *residuals,
                        double **jacobians) const {
    Eigen::Map<const Mat34d> pose(parameters[0], 3, 4);
    Vec3d map_point = Mathbox::multiplePoint(pose, origin_point);

    // 点(x0, y0, z0)到平面Ax + By + Cz + D = 0 的距离 = fabs(A*x0 + B*y0 + C*z0
    // + D) / sqrt(A^2 + B^2 + C^2)； 因为法向量（A, B,
    // C）已经归一化了，所以距离公式可以简写为：距离 = fabs(A*x0 + B*y0 + C*z0 +
    // D) ；
    residuals[0] = scale * (plane_norm.transpose() * map_point + const_value);

    if (jacobians != nullptr) {
      if (jacobians[0]) {
        Eigen::Map<Eigen::Matrix<double, 1, 12, Eigen::RowMajor>> J0(jacobians[0]);
        J0.setZero();
        J0.block<1, 3>(0, 0) =
            -scale * plane_norm.transpose() * Mathbox::skew(map_point - pose.block<3, 1>(0, 3));
        J0.block<1, 3>(0, 3) = scale * plane_norm.transpose();
      }
    }

    return true;
  }

 private:
  Vec3d origin_point;
  Vec3d plane_norm;
  double scale;
  double const_value;
};  // end of class

/**
 * EdgeFactor
 * @brief 激光线点的factor类
 **/
class EdgeFactor : public ceres::SizedCostFunction<1, 12> {
 public:
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW
  EdgeFactor(const Vec3d &origin_point, const Vec3d &point_a, const Vec3d &point_b,
             const double scale)
      : origin_point(origin_point), point_a(point_a), point_b(point_b), scale(scale) {}
  virtual ~EdgeFactor() {}
  virtual bool Evaluate(double const *const *parameters, double *residuals,
                        double **jacobians) const {
    Eigen::Map<const Mat34d> pose(parameters[0], 3, 4);
    Vec3d map_point = Mathbox::multiplePoint(pose, origin_point);

    // 向量OA = (x0 - x1, y0 - y1, z0 - z1), 向量OB = (x0 - x2, y0 - y2, z0 -
    // z2)，向量AB = （x1 - x2, y1 - y2, z1 - z2）; 点到线的距离，d = |向量OA
    // 叉乘 向量OB|/|AB|; 向量OA 叉乘 向量OB 得到的向量模长 ：
    // 向量叉乘结果为向量组成的平行四边形的面积或者两个向量组成的三角形的面积的两倍;
    // 因此|向量OA 叉乘 向量OB|再除以|AB|的模长，则得到高度，即点到线的距离；

    // 点到线的距离，d = |向量OA 叉乘 向量OB|/|AB|;
    // residuals[0] = scale * ((map_point - point_a).cross(map_point -
    // point_b)).norm();
    residuals[0] = scale * ((map_point - point_a).cross(map_point - point_b).norm() /
                            (point_a - point_b).norm());

    double k = (point_b - point_a).transpose() *
               (map_point - point_a);  ///((point_b - point_a).transpose()*(point_b - point_a));
    k = k / ((point_b - point_a).transpose() * (point_b - point_a));
    Vec3d point_vetical = point_a + k * (point_b - point_a);
    Vec3d vetical_vec = map_point - point_vetical;
    vetical_vec.normalize();
    if (jacobians != nullptr) {
      if (jacobians[0]) {
        Eigen::Map<Eigen::Matrix<double, 1, 12, Eigen::RowMajor>> J0(jacobians[0]);
        J0.setZero();
        J0.block<1, 3>(0, 0) =
            -scale * (vetical_vec.transpose()) * Mathbox::skew(map_point - pose.block<3, 1>(0, 3));
        J0.block<1, 3>(0, 3) = scale * (vetical_vec.transpose());
      }
    }

    return true;
  }

 private:
  Vec3d origin_point;
  Vec3d point_a;
  Vec3d point_b;
  double scale;
};  // end of class

/**
 * CornerPointFactor
 * @brief 激光角点的factor类
 **/
class CornerPointFactor : public ceres::SizedCostFunction<3, 12> {
 public:
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW
  CornerPointFactor(const Vec3d &curr_point, const Vec3d &map_point, const double scale)
      : curr_point(curr_point), map_point(map_point), scale(scale) {}
  virtual ~CornerPointFactor() {}
  virtual bool Evaluate(double const *const *parameters, double *residuals,
                        double **jacobians) const {
    Eigen::Map<const Mat34d> pose(parameters[0], 3, 4);
    Vec3d point_w = Mathbox::multiplePoint(pose, curr_point);

    residuals[0] = (point_w.x() - map_point.x()) * scale;
    residuals[1] = (point_w.y() - map_point.y()) * scale;
    residuals[2] = (point_w.z() - map_point.z()) * scale;

    if (jacobians != nullptr) {
      if (jacobians[0]) {
        Eigen::Map<Eigen::Matrix<double, 3, 12, Eigen::RowMajor>> J0(jacobians[0]);
        J0.setZero();
        J0.block<3, 3>(0, 0) = -scale * Mathbox::skew(point_w - pose.block<3, 1>(0, 3));
        J0.block<3, 3>(0, 3) = scale * Eigen::Matrix3d::Identity();
      }
    }

    return true;
  }

 private:
  Vec3d curr_point;
  Vec3d map_point;
  double scale;
};  // end of EdgePointFactor class

struct PointFactor {
  PointFactor(Eigen::Vector3d curr_point_, Eigen::Vector3d closed_point_, double weight_)
      : curr_point(curr_point_), closed_point(closed_point_), weight(weight_) {}

  template <typename T>
  bool operator()(const T *q, const T *t, T *residual) const {
    Eigen::Quaternion<T> q_w_curr{q[3], q[0], q[1], q[2]};
    Eigen::Matrix<T, 3, 1> t_w_curr{t[0], t[1], t[2]};
    Eigen::Matrix<T, 3, 1> cp{T(curr_point.x()), T(curr_point.y()), T(curr_point.z())};
    Eigen::Matrix<T, 3, 1> point_w;
    point_w = q_w_curr * cp + t_w_curr;

    // residual[0] = point_w.x() - T(closed_point.x());
    // residual[1] = point_w.y() - T(closed_point.y());
    // residual[2] = point_w.z() - T(closed_point.z());
    residual[0] = (point_w - closed_point).norm() * weight;

    return true;
  }

  static ceres::CostFunction *Create(const Eigen::Vector3d curr_point_,
                                     const Eigen::Vector3d closed_point_, float weight_) {
    //
    return (new ceres::AutoDiffCostFunction<PointFactor, 1, 4, 3>(
        new PointFactor(curr_point_, closed_point_, weight_)));
  }

  Eigen::Vector3d curr_point;
  Eigen::Vector3d closed_point;
  double weight;
};

struct PointToLineDistanceFactor {
  PointToLineDistanceFactor(Eigen::Vector3d curr_point_, Eigen::Vector3d closed_point_a_,
                            Eigen::Vector3d closed_point_b_, double weight_)
      : curr_point(curr_point_),
        closed_point_a(closed_point_a_),
        closed_point_b(closed_point_b_),
        weight(weight_) {}

  template <typename T>
  bool operator()(const T *q, const T *t, T *residual) const {
    Eigen::Quaternion<T> q_w_curr{q[3], q[0], q[1], q[2]};
    Eigen::Matrix<T, 3, 1> t_w_curr{t[0], t[1], t[2]};

    Eigen::Matrix<T, 3, 1> cp{T(curr_point.x()), T(curr_point.y()), T(curr_point.z())};
    Eigen::Matrix<T, 3, 1> lpa{T(closed_point_a.x()), T(closed_point_a.y()), T(closed_point_a.z())};
    Eigen::Matrix<T, 3, 1> lpb{T(closed_point_b.x()), T(closed_point_b.y()), T(closed_point_b.z())};
    Eigen::Matrix<T, 3, 1> point_w;
    point_w = q_w_curr * cp + t_w_curr;

    Eigen::Matrix<T, 3, 1> nu = (point_w - lpa).cross(point_w - lpb);  // 向量OA 叉乘 向量OB
    T de = (lpa - lpb).norm();                                         // 向量AB

    // 点到线的距离，d = |向量OA 叉乘 向量OB|/|AB|;
    // residual[0] = nu.x() / de;
    // residual[1] = nu.y() / de;
    // residual[2] = nu.z() / de;

    residual[0] = nu.norm() / de * weight;

    return true;
  }

  static ceres::CostFunction *Create(const Eigen::Vector3d curr_point_,
                                     const Eigen::Vector3d closed_point_a_,
                                     Eigen::Vector3d closed_point_b_, double weight_) {
    return (new ceres::AutoDiffCostFunction<PointToLineDistanceFactor, 1, 4, 3>(
        new PointToLineDistanceFactor(curr_point_, closed_point_a_, closed_point_b_, weight_)));
  }

  Eigen::Vector3d curr_point;
  Eigen::Vector3d closed_point_a, closed_point_b;
  double weight;
};

struct PointToPlaneFactor {
  PointToPlaneFactor(Eigen::Vector3f p_c, Eigen::Vector4f pabcd, float weight, float dis)
      : p_c_(p_c), pabcd_(pabcd), weight_(weight), distance_(dis) {}

  template <typename T>
  bool operator()(const T *q, const T *t, T *residual) const {
    Eigen::Quaternion<T> q_w_curr{q[3], q[0], q[1], q[2]};
    Eigen::Matrix<T, 3, 1> t_w_curr{t[0], t[1], t[2]};

    Eigen::Matrix<T, 3, 1> cp{T(p_c_(0)), T(p_c_(1)), T(p_c_(2))};
    Eigen::Matrix<T, 4, 1> pabcd{T(pabcd_(0)), T(pabcd_(1)), T(pabcd_(2)), T(pabcd_(3))};
    Eigen::Matrix<T, 3, 1> pw;
    pw = q_w_curr * cp + t_w_curr;

    T d = pabcd(0) * pw(0) + pabcd(1) * pw(1) + pabcd(2) * pw(2) + pabcd(3);

    residual[0] = d * T(weight_);

    return true;
  }

  static ceres::CostFunction *Create(const Eigen::Vector3f p_c, const Eigen::Vector4f pabcd,
                                     float weight, float dis) {
    return (new ceres::AutoDiffCostFunction<PointToPlaneFactor, 1, 4, 3>(
        new PointToPlaneFactor(p_c, pabcd, weight, dis)));
  }

  void scale_weight(float w) { weight_ = weight_ * w; }

  ceres::CostFunction *Create() {
    return (new ceres::AutoDiffCostFunction<PointToPlaneFactor, 1, 4, 3>(
        new PointToPlaneFactor(p_c_, pabcd_, weight_, distance_)));
  }

  Eigen::Vector3f p_c_;
  Eigen::Vector4f pabcd_;
  float weight_;
  float distance_;
};

class PointToPlaneFactor2 : public ceres::SizedCostFunction<1, 12> {
 public:
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW
  PointToPlaneFactor2(Eigen::Vector3d p_c, Eigen::Vector4d pabcd, double weight)
      : p_c_(p_c), pabcd_(pabcd), weight_(weight) {
    plane_norm_(0) = pabcd_(0);
    plane_norm_(1) = pabcd_(1);
    plane_norm_(2) = pabcd_(2);
    D_ = pabcd_(3);
  }

  virtual ~PointToPlaneFactor2() {}
  virtual bool Evaluate(double const *const *parameters, double *residuals,
                        double **jacobians) const {
    Eigen::Map<const Mat34d> pose(parameters[0], 3, 4);
    Vec3d p_w = Mathbox::multiplePoint(pose, p_c_);

    // 点(x0, y0, z0)到平面Ax + By + Cz + D = 0 的距离 = fabs(A*x0 + B*y0 + C*z0
    // + D) / sqrt(A^2 + B^2 + C^2)； 因为法向量（A, B,
    // C）已经归一化了，所以距离公式可以简写为：距离 = fabs(A*x0 + B*y0 + C*z0
    // +D)
    residuals[0] = weight_ * (plane_norm_.transpose() * p_w + D_);

    if (jacobians != nullptr) {
      if (jacobians[0]) {
        Eigen::Map<Eigen::Matrix<double, 1, 12, Eigen::RowMajor>> J0(jacobians[0]);
        J0.setZero();
        J0.block<1, 3>(0, 0) =
            -weight_ * plane_norm_.transpose() * Mathbox::skew(p_w - pose.block<3, 1>(0, 3));
        J0.block<1, 3>(0, 3) = weight_ * plane_norm_.transpose();
      }
    }

    return true;
  }

  Eigen::Vector3d p_c_;
  Eigen::Vector4d pabcd_;
  double weight_;
  Vec3d plane_norm_;
  double D_;
};

template <typename T>
inline void QuaternionInverse(const T q[4], T q_inverse[4]) {
  q_inverse[0] = q[0];
  q_inverse[1] = -q[1];
  q_inverse[2] = -q[2];
  q_inverse[3] = -q[3];
};

template <typename T>
inline void QuaternionNormalize(const T q[4], T q_norm[4]) {
  T n = sqrt(q[0] * q[0] + q[1] * q[1] + q[2] * q[2] + q[3] * q[3]);
  n = T(1) / n;
  q_norm[0] = q[0] * n;
  q_norm[1] = q[1] * n;
  q_norm[2] = q[2] * n;
  q_norm[3] = q[3] * n;
};

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

struct DeltaGPSExtrincsFactor {
  DeltaGPSExtrincsFactor(Eigen::Quaterniond q_last, Eigen::Vector3d t_last, double weight)
      : weight(weight) {
    q_x = q_last.x();
    q_y = q_last.y();
    q_z = q_last.z();
    q_w = q_last.w();
    t_x = t_last.x();
    t_y = t_last.y();
    t_z = t_last.z();
  }

  template <typename T>
  bool operator()(const T *const q, const T *const t, T *residual) const {
    T q_tmp[4];
    q_tmp[0] = q[3];  // ceres in w, x, y, z order
    q_tmp[1] = q[0];
    q_tmp[2] = q[1];
    q_tmp[3] = q[2];

    T q_0[4];
    q_0[0] = T(q_w);
    q_0[1] = T(q_x);
    q_0[2] = T(q_y);
    q_0[3] = T(q_z);

    T q_0_inv[4];
    QuaternionInverse(q_0, q_0_inv);

    T delta_q[4];
    ceres::QuaternionProduct(q_0_inv, q_tmp, delta_q);

    T ypr[3];
    Quaternion2EulerAngle(delta_q, ypr);

    residual[0] = ypr[2] * T(57.3) * T(weight * 2.0);
    residual[1] = ypr[1] * T(57.3) * T(weight * 2.0);
    residual[2] = ypr[0] * T(57.3) * T(weight);

    T t_tmp[3];
    t_tmp[0] = t[0];
    t_tmp[1] = t[1];
    t_tmp[2] = t[2];

    residual[3] = (t_x - t_tmp[0]) * T(weight);
    residual[4] = (t_y - t_tmp[1]) * T(weight);
    residual[5] = (t_z - t_tmp[2]) * T(weight * 2.0);

    return true;
  }

  static ceres::CostFunction *Create(Eigen::Quaterniond q_last, Eigen::Vector3d t_last,
                                     double weight) {
    return (new ceres::AutoDiffCostFunction<DeltaGPSExtrincsFactor, 6, 4, 3>(
        new DeltaGPSExtrincsFactor(q_last, t_last, weight)));
  }

 private:
  double q_x, q_y, q_z, q_w;
  double t_x, t_y, t_z;
  double weight;
};

struct DeltaRFactor {
  DeltaRFactor(Eigen::Quaterniond q_last, Eigen::Quaterniond dq, double w_yaw, double w_pitch,
               double w_roll) {
    q_x = q_last.x();
    q_y = q_last.y();
    q_z = q_last.z();
    q_w = q_last.w();
    dq_x = dq.x();
    dq_y = dq.y();
    dq_z = dq.z();
    dq_w = dq.w();
    weight_y = w_yaw;
    weight_p = w_pitch;
    weight_r = w_roll;
  }

  template <typename T>
  bool operator()(const T *const q_i, T *residuals) const {
    T q_i_tmp[4];
    q_i_tmp[0] = q_i[3];  // ceres in w, x, y, z order
    q_i_tmp[1] = q_i[0];
    q_i_tmp[2] = q_i[1];
    q_i_tmp[3] = q_i[2];

    T q_0[4];
    q_0[0] = T(q_w);
    q_0[1] = T(q_x);
    q_0[2] = T(q_y);
    q_0[3] = T(q_z);

    T q_0_inv[4];
    QuaternionInverse(q_0, q_0_inv);

    T delta_q[4];
    ceres::QuaternionProduct(q_0_inv, q_i_tmp, delta_q);

    T dq_m[4];
    dq_m[0] = T(dq_w);
    dq_m[1] = T(dq_x);
    dq_m[2] = T(dq_y);
    dq_m[3] = T(dq_z);

    T dq_m_inv[4];
    QuaternionInverse(dq_m, dq_m_inv);

    T error_q[4];
    ceres::QuaternionProduct(dq_m_inv, delta_q, error_q);

    T ypr[3];
    Quaternion2EulerAngle(error_q, ypr);

    residuals[0] = ypr[2] * T(57.3) * T(weight_r);
    residuals[1] = ypr[1] * T(57.3) * T(weight_p);
    residuals[2] = ypr[0] * T(57.3) * T(weight_y);

    return true;
  }

  static ceres::CostFunction *Create(Eigen::Quaterniond q_last, Eigen::Quaterniond dq, double w_yaw,
                                     double w_pitch, double w_roll) {
    return (new ceres::AutoDiffCostFunction<DeltaRFactor, 3, 4>(
        new DeltaRFactor(q_last, dq, w_yaw, w_pitch, w_roll)));
  }

  double q_x, q_y, q_z, q_w;
  double dq_x, dq_y, dq_z, dq_w;
  double weight_y, weight_p, weight_r;
};

struct DeltaTFactor {
  DeltaTFactor(Eigen::Quaterniond q_last, Eigen::Vector3d t_last, Eigen::Vector3d dt, double w) {
    q_last = q_last;
    t_last = t_last;
    dt = dt;
    weight = w;
  }

  template <typename T>
  bool operator()(const T *tj, T *residuals) const {
    T w_q_i[4];
    w_q_i[0] = T(q_last.w());  // ceres in w, x, y, z order
    w_q_i[1] = T(q_last.x());
    w_q_i[2] = T(q_last.y());
    w_q_i[3] = T(q_last.z());

    T i_q_w[4];
    QuaternionInverse(w_q_i, i_q_w);

    T t_w_ij[3];
    t_w_ij[0] = tj[0] - T(t_last.x());
    t_w_ij[1] = tj[1] - T(t_last.y());
    t_w_ij[2] = tj[2] - T(t_last.z());

    T t_i_ij[3];
    ceres::QuaternionRotatePoint(i_q_w, t_w_ij, t_i_ij);

    residuals[0] = (t_i_ij[0] - T(dt.x())) * T(weight);
    residuals[1] = (t_i_ij[1] - T(dt.y())) * T(weight);
    residuals[2] = (t_i_ij[2] - T(dt.z())) * T(weight);

    return true;
  }

  static ceres::CostFunction *Create(Eigen::Quaterniond q_last, Eigen::Vector3d t_last,
                                     Eigen::Vector3d dt, double weight) {
    return (new ceres::AutoDiffCostFunction<DeltaTFactor, 3, 3>(
        new DeltaTFactor(q_last, t_last, dt, weight)));
  }

  Eigen::Quaterniond q_last;
  Eigen::Vector3d t_last;
  Eigen::Vector3d dt;
  double weight;
};

struct DeltaDisFactor {
  DeltaDisFactor(Eigen::Vector3d t_last, double dt_norm, double w) {
    t_last_ = t_last;
    dt_norm_ = dt_norm;
    weight_ = w;
  }

  template <typename T>
  bool operator()(const T *t, T *residuals) const {
    Eigen::Matrix<T, 3, 1> t_curr{t[0], t[1], t[2]};

    residuals[0] = ((t_curr - t_last_).norm() - dt_norm_) * T(weight_);

    return true;
  }

  static ceres::CostFunction *Create(Eigen::Vector3d t_last, double dt_norm, double w) {
    return (new ceres::AutoDiffCostFunction<DeltaDisFactor, 1, 3>(
        new DeltaDisFactor(t_last, dt_norm, w)));
  }

  Eigen::Vector3d t_last_;
  double dt_norm_;
  double weight_;
};

struct RelativeRFactor {
  RelativeRFactor(double q_x, double q_y, double q_z, double q_w, double q_var)
      : q_x(q_x), q_y(q_y), q_z(q_z), q_w(q_w), q_var(q_var) {}

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
  bool operator()(const T *w_q_i, const T *ti, const T *w_q_j, const T *tj, T *residuals) const {
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

struct RTFactor {
  RTFactor(Eigen::Quaterniond q_pre, Eigen::Vector3d t_pre, double w) {
    q_pre_inv_ = q_pre.conjugate();
    q_pre_inv_.normalize();
    t_pre_ = t_pre;
    weight_t_ = w;
    weight_q_ = 2.0 * w;
  }

  template <typename T>
  bool operator()(const T *qi, const T *ti, T *residuals) const {
    residuals[0] = (ti[0] - T(t_pre_.x())) * T(weight_t_);
    residuals[1] = (ti[1] - T(t_pre_.y())) * T(weight_t_);
    residuals[2] = (ti[2] - T(t_pre_.z())) * T(weight_t_);

    T i_q_w[4];
    i_q_w[0] = T(q_pre_inv_.w());  // ceres in w, x, y, z order
    i_q_w[1] = T(q_pre_inv_.x());
    i_q_w[2] = T(q_pre_inv_.y());
    i_q_w[3] = T(q_pre_inv_.z());

    T w_q_i_tmp[4];
    w_q_i_tmp[0] = qi[3];  // ceres in w, x, y, z order
    w_q_i_tmp[1] = qi[0];
    w_q_i_tmp[2] = qi[1];
    w_q_i_tmp[3] = qi[2];

    T q_e[4];
    ceres::QuaternionProduct(i_q_w, w_q_i_tmp, q_e);

    residuals[3] = q_e[1] * T(weight_q_);
    residuals[4] = q_e[2] * T(weight_q_);
    residuals[5] = q_e[3] * T(weight_q_);

    return true;
  }

  static ceres::CostFunction *Create(Eigen::Quaterniond q_pre, Eigen::Vector3d t_pre, double w) {
    return (new ceres::AutoDiffCostFunction<RTFactor, 6, 4, 3>(new RTFactor(q_pre, t_pre, w)));
  }

  Eigen::Quaterniond q_pre_inv_;
  Eigen::Vector3d t_pre_;
  double weight_t_, weight_q_;
};

struct GravityFactor {
  GravityFactor(double r, double p, double w) {
    roll = r;
    pitch = p;
    weight = w;
  }

  template <typename T>
  bool operator()(const T *const w_q_i, T *residuals) const {
    T q_i_tmp[4];
    q_i_tmp[0] = w_q_i[3];  // ceres in w, x, y, z order
    q_i_tmp[1] = w_q_i[0];
    q_i_tmp[2] = w_q_i[1];
    q_i_tmp[3] = w_q_i[2];

    T ypr[3];
    Quaternion2EulerAngle(q_i_tmp, ypr);

    residuals[0] = (ypr[2] * T(57.3) - T(roll)) * T(weight);
    residuals[1] = (ypr[1] * T(57.3) - T(pitch)) * T(weight);

    // T v_g[3];
    // v_g[0] = T(0);
    // v_g[1] = T(0);
    // v_g[2] = T(1);

    // T e_v[3];
    // ceres::QuaternionRotatePoint(error_q, v_g, e_v);

    // // 残差如果是绝对值，收敛的方向性可能会更加一致；
    // residuals[0] = e_v[0] * T(weight);
    // residuals[1] = e_v[1] * T(weight);

    return true;
  }

  static ceres::CostFunction *Create(double r, double p, double w) {
    return (new ceres::AutoDiffCostFunction<GravityFactor, 2, 4>(new GravityFactor(r, p, w)));
  }

  double roll, pitch;
  double weight;
};

struct LocalGroundFactor {
  LocalGroundFactor(const double t_z, const double weight) : t_z(t_z), weight(weight) {}

  template <typename T>
  bool operator()(const T *ti, T *residuals) const {
    residuals[0] = (ti[2] - t_z) * T(weight);

    return true;
  }

  static ceres::CostFunction *Create(const double t_z, const double weight) {
    return (new ceres::AutoDiffCostFunction<LocalGroundFactor, 1, 3>(
        new LocalGroundFactor(t_z, weight)));
  }

  double t_z, weight;
};

struct GroundFactor {
  GroundFactor(double weight) : weight(weight) {}

  template <typename T>
  bool operator()(const T *ti, const T *tj, T *residuals) const {
    residuals[0] = (ti[2] - tj[2]) * T(weight);

    return true;
  }

  static ceres::CostFunction *Create(const double weight) {
    return (new ceres::AutoDiffCostFunction<GroundFactor, 1, 3, 3>(new GroundFactor(weight)));
  }

  double t_x, t_y, t_z, weight;
};

}  // namespace GR_SLAM
#endif