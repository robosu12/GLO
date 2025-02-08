/************************************************************************
 * Software License Agreement (BSD License)
 ************************************************************************/
#ifndef SLAM_MATH_HPP_
#define SLAM_MATH_HPP_

#include <Eigen/Core>
#include <Eigen/Eigenvalues>
#include <Eigen/Geometry>
#include <iostream>
#include <vector>

namespace GR_SLAM {
static const double SMALL_EPS = 1e-9;
static const float SMALL_EPSf = 1e-9;

static const double PI = 3.1415926535897932384626433832795028842;
static const float PIf = 3.14159265358979323846f;
static const double Deg2Rad = PI / 180.0;
static const double Rad2Deg = 180.0 / PI;

typedef Eigen::Vector2d Vec2d;
typedef Eigen::Vector2f Vec2f;
typedef Eigen::Vector3d Vec3d;
typedef Eigen::Vector3f Vec3f;
typedef Eigen::Matrix<double, 5, 1> Vec5d;
typedef Eigen::Matrix<double, 6, 1> Vec6d;
typedef Eigen::Matrix<double, Eigen::Dynamic, 1> VecX;
typedef Eigen::Matrix<double, 2, 2, Eigen::RowMajor> Mat2d;
typedef Eigen::Matrix<float, 2, 2, Eigen::RowMajor> Mat2f;
typedef Eigen::Matrix<double, 3, 3, Eigen::RowMajor> Mat3d;
typedef Eigen::Matrix<double, 3, 2, Eigen::RowMajor> Mat32d;
typedef Eigen::Matrix<double, 3, 4, Eigen::RowMajor> Mat34d;
typedef Eigen::Matrix<double, 2, 4, Eigen::RowMajor> Mat24d;
typedef Eigen::Matrix<double, 4, 4, Eigen::RowMajor> Mat4d;
typedef Eigen::Matrix<double, 3, 6, Eigen::RowMajor> Mat36d;
typedef Eigen::Matrix<double, 3, 12, Eigen::RowMajor> Mat312d;
typedef Eigen::Matrix<double, 5, 3, Eigen::RowMajor> Mat53d;
typedef Eigen::Matrix<double, 6, 6, Eigen::RowMajor> Mat6d;
typedef Eigen::Matrix<double, 12, 6, Eigen::RowMajor> Mat126d;
typedef Eigen::Matrix<double, 12, 5, Eigen::RowMajor> Mat125d;
typedef Eigen::Matrix<double, Eigen::Dynamic, Eigen::Dynamic> MatXX;

typedef Eigen::Matrix<double, 12, 3, Eigen::RowMajor> Mat123d;
typedef Eigen::Matrix<double, 12, 2, Eigen::RowMajor> Mat122d;

typedef Eigen::Matrix<float, 4, 4, Eigen::RowMajor> Mat4f;
typedef Eigen::Matrix<double, 4, 4, Eigen::RowMajor> Mat4d;

typedef std::vector<Vec3d, Eigen::aligned_allocator<Vec3d>> vVec3d;
typedef std::vector<Vec2d, Eigen::aligned_allocator<Vec2d>> vVec2d;
typedef std::vector<Mat34d, Eigen::aligned_allocator<Mat34d>> vMat34d;
typedef std::vector<Mat4f, Eigen::aligned_allocator<Mat4f>> vMat4f;
typedef std::vector<Mat4d, Eigen::aligned_allocator<Mat4d>> vMat4d;

/**
 * QuatPose
 * @brief 四元数位姿表示类
 *
 **/
struct QuatPose {
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW
  QuatPose() : q_wb(Mat3d::Identity()), t_wb(Vec3d::Zero()) {}
  QuatPose(const Mat34d &pose) : q_wb(pose.block<3, 3>(0, 0)), t_wb(pose.block<3, 1>(0, 3)) {}
  QuatPose(const Eigen::Quaterniond &q, const Vec3d &t) : q_wb(q), t_wb(t) {}

  Mat34d getPose() const {
    Mat34d ret;
    ret.block<3, 3>(0, 0) = q_wb.toRotationMatrix();
    ret.block<3, 1>(0, 3) = t_wb;
    return ret;
  }
  Eigen::Quaterniond q_wb;
  Vec3d t_wb;
};  // end of class

/**
 * Pose2d
 * @brief 2d pose类
 *
 **/
class Pose2d {
 public:
  Pose2d() {
    x_ = 0.0;
    y_ = 0.0;
    theta_ = 0.0;
  }
  Pose2d(double x, double y, double theta) : x_(x), y_(y), theta_(theta) {}

  const Pose2d operator*(const Pose2d &p2) {
    Pose2d p;
    Eigen::Matrix2d R;
    double cos_value = cos(theta_);
    double sin_value = sin(theta_);
    R << cos_value, -sin_value, sin_value, cos_value;
    Vec2d pt2(p2.x_, p2.y_);
    Vec2d pt = R * pt2 + Vec2d(x_, y_);

    p.x_ = pt(0);
    p.y_ = pt(1);
    p.theta_ = theta_ + p2.theta_;
    NormAngle(p.theta_);
    return p;
  }

  const Vec2d operator*(const Vec2d &p) {
    Eigen::Matrix2d R;
    double cos_value = cos(theta_);
    double sin_value = sin(theta_);
    R << cos_value, -sin_value, sin_value, cos_value;
    Vec2d t(x_, y_);
    return R * p + t;
  }

  Pose2d inv() {
    double x = -(cos(theta_) * x_ + sin(theta_) * y_);
    double y = -(-sin(theta_) * x_ + cos(theta_) * y_);
    double theta = -theta_;
    return Pose2d(x, y, theta);
  }

  void NormAngle(double &angle) {
    while (angle >= M_PI) angle -= 2.0 * M_PI;
    while (angle < -M_PI) angle += 2.0 * M_PI;
  }
  double x() { return x_; }
  double y() { return y_; }
  double yaw() { return theta_; }

 private:
  double x_, y_, theta_;
};  // class Pose2d

/**
 * Mathbox
 * @brief 基础运算类
 *
 **/
class Mathbox {
 public:
  /**
   *rad2deg
   *@brief
   *弧度转角度
   *
   *@param[in] radians-弧度
   *@return double
   **/
  static double rad2deg(double radians) { return radians * 180.0 / M_PI; }

  /**
   *deg2rad
   *@brief
   *角度转弧度
   *
   *@param[in] degrees-角度
   *@return double
   **/
  static double deg2rad(double degrees) { return degrees * M_PI / 180.0; }

  /**
   *isRotationMatrix
   *@brief
   *判断矩阵是否是旋转矩阵
   *
   *@param[in] R-矩阵
   *@return bool
   **/
  static bool isRotationMatrix(const Mat3d &R) {
    Mat3d Rt;
    Rt = R.transpose();
    Mat3d shouldBeIdentity = Rt * R;
    Mat3d I = Mat3d::Identity();
    return (shouldBeIdentity - I).norm() < 2e-6;
  }

  /**
   *rotationMatrixToEulerAngles
   *@brief
   *旋转矩阵转换成欧拉角
   *
   *@param[in] R-旋转矩阵
   *@return Vec3d-欧拉角
   **/
  static Vec3d rotationMatrixToEulerAngles(const Mat3d &R) {
    assert(isRotationMatrix(R));
    double sy = sqrt(R(0, 0) * R(0, 0) + R(1, 0) * R(1, 0));
    bool singular = sy < 1e-6;  // true: `Y`方向旋转为`+/-90`度
    double x, y, z;
    if (!singular) {
      x = atan2(R(2, 1), R(2, 2));
      y = atan2(-R(2, 0), sy);
      z = atan2(R(1, 0), R(0, 0));
    } else {
      x = atan2(-R(1, 2), R(1, 1));
      y = atan2(-R(2, 0), sy);
      z = 0;
    }
    return Vec3d(x, y, z);
  }

  static Vec3d rotation2rpy(const Mat3d &R) {
    assert(isRotationMatrix(R));
    double sy = sqrt(R(0, 0) * R(0, 0) + R(1, 0) * R(1, 0));
    bool singular = sy < 1e-6;  // true: `Y`方向旋转为`+/-90`度
    double x, y, z;
    if (!singular) {
      x = atan2(R(2, 1), R(2, 2));
      y = atan2(-R(2, 0), sy);
      z = atan2(R(1, 0), R(0, 0));
    } else {
      x = atan2(-R(1, 2), R(1, 1));
      y = atan2(-R(2, 0), sy);
      z = 0;
    }
    return Vec3d(x, y, z);
  }

  static Mat3d rpyToRotationMatrix(const Vec3d &rpy) {
    double y = rpy(2);
    double p = rpy(1);
    double r = rpy(0);

    Mat3d Rz;
    Rz << cos(y), -sin(y), 0.0, sin(y), cos(y), 0, 0, 0, 1.0;

    Mat3d Ry;
    Ry << cos(p), 0.0, sin(p), 0.0, 1.0, 0.0, -sin(p), 0.0, cos(p);

    Mat3d Rx;
    Rx << 1.0, 0.0, 0.0, 0.0, cos(r), -sin(r), 0.0, sin(r), cos(r);

    return Rz * Ry * Rx;
  }

  //进行角度正则化．
  static double NormalizationAngle(double angle) {
    if (angle > M_PI)
      angle -= 2 * M_PI;
    else if (angle < -M_PI)
      angle += 2 * M_PI;

    return angle;
  }

  //进行角度正则化．
  static double NormalizationRollPitchAngle(double angle) {
    if (angle > M_PI / 2)
      angle -= M_PI;
    else if (angle < -M_PI / 2)
      angle += M_PI;

    return angle;
  }

  /**
   *multiplePose34d
   *@brief
   *pose乘法运算
   *
   *@param[in] pose1
   *@param[in] pose2
   *@return Mat34d
   **/
  static Mat34d multiplePose34d(const Mat34d &pose1, const Mat34d &pose2) {
    Mat34d ret = Identity34();
    ret.block<3, 3>(0, 0) = pose1.block<3, 3>(0, 0) * pose2.block<3, 3>(0, 0);
    ret.block<3, 1>(0, 3) =
        pose1.block<3, 3>(0, 0) * pose2.block<3, 1>(0, 3) + pose1.block<3, 1>(0, 3);
    return ret;
  }

  /**
   *Mat34d2Quat
   *@brief
   *四元数转换成MAT34表示
   *
   *@param[in] q－四元数
   *@param[in] t-平移
   *@return Mat34d
   **/
  static Mat34d Quat2Mat34d(const Eigen::Quaterniond &q, const Vec3d &t) {
    Mat34d ret = Identity34();
    ret.block<3, 3>(0, 0) = q.toRotationMatrix();
    ret.block<3, 1>(0, 3) = t;
    return ret;
  }

  /**
   *Mat34d2Quat
   *@brief
   *欧拉角转换成MAT34表示
   *
   *@param[in] angles-欧拉角
   *@param[in] t-平移
   *@return Mat34d
   **/
  static Mat34d Euler2Mat34d(const Vec3d &angles, const Vec3d &t) {
    Mat34d ret = Identity34();
    Mat3d R = (Eigen::AngleAxisd(angles[0], Vec3d::UnitZ()) *
               Eigen::AngleAxisd(angles[1], Vec3d::UnitY()) *
               Eigen::AngleAxisd(angles[2], Vec3d::UnitX()))
                  .toRotationMatrix();
    ret.block<3, 3>(0, 0) = R;
    ret.block<3, 1>(0, 3) = t;
    return ret;
  }

  /**
   *Mat34d2Quat
   *@brief
   *MAT34转换成四元数表示
   *
   *@param[in] pose
   *@param[out] q－四元数
   *@param[out] t-平移
   **/
  static void Mat34d2Quat(const Mat34d &pose, Eigen::Quaterniond &q, Vec3d &t) {
    Eigen::Quaterniond temp_q(pose.block<3, 3>(0, 0));
    q = temp_q;
    t = pose.block<3, 1>(0, 3);
  }

  /**
   *multiplePoint
   *@brief
   *对点进行坐标变换
   *
   *@param[in] pose
   *@param[in] point－坐标点
   *@return Vec3d
   **/
  static Vec3d multiplePoint(const Mat34d &pose, const Vec3d &point) {
    Vec3d ret = pose.block<3, 3>(0, 0) * point + pose.block<3, 1>(0, 3);
    return ret;
  }

  /**
   *deltaPose34d
   *@brief
   *计算相对pose
   *
   *@param[in] pose1
   *@param[in] pose2
   *@return Mat34d
   **/
  static Mat34d deltaPose34d(const Mat34d &pose1, const Mat34d &pose2) {
    return multiplePose34d(inversePose34d(pose1), pose2);
  }

  /**
   *inversePose
   *@brief
   *计算pose的逆
   *
   *@param[in] pose
   *@return Mat4d
   **/
  static Mat4d inversePose(const Mat4d &pose) {
    Mat4d ret = pose;
    ret.block<3, 3>(0, 0) = pose.block<3, 3>(0, 0).transpose();
    ret.block<3, 1>(0, 3) = -ret.block<3, 3>(0, 0) * pose.block<3, 1>(0, 3);
    return ret;
  }

  /**
   *inversePose34d
   *@brief
   *计算pose的逆
   *
   *@param[in] pose
   *@return Mat34d
   **/
  static Mat34d inversePose34d(const Mat34d &pose) {
    Mat34d ret = pose;
    ret.block<3, 3>(0, 0) = pose.block<3, 3>(0, 0).transpose();
    ret.block<3, 1>(0, 3) = -ret.block<3, 3>(0, 0) * pose.block<3, 1>(0, 3);
    return ret;
  }

  /**
   *Identity34
   *@brief
   *单位值pose
   *
   *@return Mat34d
   **/
  static Mat34d Identity34() {
    // static Mat34d identity = Mat34d::Constant(std::nan(""));
    // if (identity.hasNaN())
    // {
    //   identity.block<3, 3>(0, 0) = Mat3d::Identity();
    //   identity.block<3, 1>(0, 3) = Vec3d::Zero();
    // }
    Mat34d identity;
    identity.block<3, 3>(0, 0) = Mat3d::Identity();
    identity.block<3, 1>(0, 3) = Vec3d::Zero();

    return identity;
  }

  /**
   *skew
   *@brief
   *计算向量的反对称矩阵
   *@param[in] v-3维向量
   *
   *@return Mat3d
   **/
  static Mat3d skew(const Vec3d &v) {
    Mat3d mat;
    mat(0, 0) = 0.0;
    mat(0, 1) = -v(2);
    mat(0, 2) = v(1);
    mat(1, 0) = v(2);
    mat(1, 1) = 0.0;
    mat(1, 2) = -v(0);
    mat(2, 0) = -v(1);
    mat(2, 1) = v(0);
    mat(2, 2) = 0.0;
    return mat;
  }

  /**
   *Jr_inv
   *@brief
   *计算向量的右雅可比逆
   *@param[in] w-3维向量
   *
   *@return Mat3d
   **/
  static Mat3d Jr_inv(const Vec3d &w) {
    double theta = w.norm();
    if (theta < 0.00001)
      return Mat3d::Identity();
    else {
      Mat3d K = skew(w / theta);
      return Mat3d::Identity() + 0.5 * skew(w) +
             (1.0 - (1.0 + cos(theta)) * theta / (2.0 * sin(theta))) * K * K;
    }
  }

  /**
   *Exp3d
   *@brief
   *Exp计算
   *
   *@param[in] dx-李代数
   *@return Mat3d-李群
   **/
  static Mat3d Exp3d(const Vec3d &dx) {
    double theta = dx.norm();
    if (theta < SMALL_EPS) {
      return Mat3d::Identity();
    } else {
      Mat3d hatdx = skew(dx / theta);
      return Mat3d::Identity() + sin(theta) * hatdx + (1 - cos(theta)) * hatdx * hatdx;
    }
  }
  static Mat6d Adj(const Mat4d &T) {
    Mat6d adj = Mat6d::Zero();
    adj.block<3, 3>(0, 0) = adj.block<3, 3>(3, 3) = T.block<3, 3>(0, 0);
    adj.block<3, 3>(0, 3) = skew(T.block<3, 1>(0, 3)) * T.block<3, 3>(0, 0);
    return adj;
  }

  /**
   *Log
   *@brief
   *LOG计算
   *
   *@param[in] R-李群
   *@return Vec3d-李代数
   **/
  static Vec3d Log(const Mat3d &R) {
    Eigen::Quaterniond quaternion(R);
    quaternion.normalize();

    double n = quaternion.vec().norm();
    double w = quaternion.w();
    double squared_w = w * w;

    double two_atan_nbyw_by_n;

    if (n < SMALL_EPS) {
      assert(fabs(w) > SMALL_EPS);

      two_atan_nbyw_by_n = 2. / w - 2. * (n * n) / (w * squared_w);
    } else {
      if (fabs(w) < SMALL_EPS) {
        if (w > 0) {
          two_atan_nbyw_by_n = M_PI / n;
        } else {
          two_atan_nbyw_by_n = -M_PI / n;
        }
      }
      two_atan_nbyw_by_n = 2 * atan(n / w) / n;
    }

    return two_atan_nbyw_by_n * quaternion.vec();
  }

  /**
   *Interp_SO3
   *@brief
   *SO3插值计算
   *
   *@param[in] R1-插值起始值
   *@param[in] R2-插值终止值
   *@param[out] alpha-插值系数
   **/
  static Mat3d Interp_SO3(const Mat3d &R1, const Mat3d &R2, const double alpha) {
    if (alpha < 0.000001)
      return R1;

    if (alpha > 0.99999)
      return R2;

    Mat3d R = R1.transpose() * R2;
    Vec3d lie = alpha * Log(R);
    R = Exp3d(lie);
    Eigen::Quaterniond q(R1 * R);
    q.normalize();
    return q.toRotationMatrix();
  }

  /**
   *Interp_SE3
   *@brief
   *SE3插值计算
   *
   *@param[in] T1-插值起始值
   *@param[in] T2-插值终止值
   *@param[out] alpha-插值系数
   **/
  static Mat34d Interp_SE3(const Mat34d &T1, const Mat34d &T2, const double alpha) {
    Mat34d pose = Identity34();
    if (alpha < 0.00001)
      return T1;

    if (alpha > 0.99999)
      return T2;
    Mat3d R = T1.block<3, 3>(0, 0).transpose() * (T2.block<3, 3>(0, 0));
    Vec3d lie = alpha * Log(R);
    R = Exp3d(lie);
    Eigen::Quaterniond q(T1.block<3, 3>(0, 0) * R);
    q.normalize();
    pose.block<3, 3>(0, 0) = q.toRotationMatrix();
    pose.block<3, 1>(0, 3) = (1.0 - alpha) * T1.block<3, 1>(0, 3) + alpha * T2.block<3, 1>(0, 3);
    return pose;
  }

  static Pose2d Mat34d2Pose2d(const Mat34d &pose) {
    const double x = pose(0, 3);
    const double y = pose(1, 3);
    const Vec3d euler_angle = rotationMatrixToEulerAngles(pose.block<3, 3>(0, 0));
    const double theta = euler_angle[2];
    return Pose2d(x, y, theta);
  }

  /**
   *XYRtoMat34d
   *@brief
   *将三维向量x,y,yaw转换为Mat34d
   *
   **/
  static Mat34d XYRtoMat34d(const Vec3d &v) {
    Mat34d result = Mathbox::Identity34();

    Vec3d xyz;
    xyz.x() = v.x();
    xyz.y() = v.y();
    xyz.z() = 0.0;
    Vec3d rpy;
    rpy.x() = 0.0;
    rpy.y() = 0.0;
    rpy.z() = v.z();

    result.block<3, 1>(0, 3) = xyz;
    result.block<3, 3>(0, 0) = Mathbox::rpyToRotationMatrix(rpy);

    return result;
  }

  /**
   *XYRtoMat34d
   *@brief
   *将Mat34d转换为三维向量x,y,yaw
   *
   **/
  static Vec3d Mat34dtoXYR(const Mat34d &m) {
    Vec3d result;
    result.x() = m(0, 3);
    result.y() = m(1, 3);
    Vec3d angle_tmp = Mathbox::rotationMatrixToEulerAngles(m.block<3, 3>(0, 0));
    result.z() = angle_tmp(2);

    return result;
  }

  static void ToPlaneMode(Mat34d &source) {
    Vec3d p_predict = source.block<3, 1>(0, 3);
    Mat3d r_predict = source.block<3, 3>(0, 0);
    Vec3d rpy_predict = Mathbox::rotation2rpy(r_predict);

    p_predict.z() = 0.0;
    rpy_predict.x() = 0.0;
    rpy_predict.y() = 0.0;
    r_predict = Mathbox::rpyToRotationMatrix(rpy_predict);

    source.block<3, 1>(0, 3) = p_predict;
    source.block<3, 3>(0, 0) = r_predict;
  }

  // 计算两点间的距离和角度差
  static void deltaDisAndYawBetweenTwoPoints(const Mat34d &pose1, const Mat34d &pose2,
                                             double &diff_dis, double &diff_yaw) {
    Mat34d delta_pose = deltaPose34d(pose1, pose2);
    Vec3d delta_p = delta_pose.block<3, 1>(0, 3);
    Mat3d delta_r = delta_pose.block<3, 3>(0, 0);
    Vec3d delta_rpy = rotation2rpy(delta_r);

    diff_dis = std::sqrt((delta_p.x() * delta_p.x()) + (delta_p.y() * delta_p.y()));
    diff_yaw = delta_rpy.z();
  }
};  // end of class

}  // namespace GR_SLAM

#endif