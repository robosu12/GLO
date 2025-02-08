/************************************************************************
 * Software License Agreement (BSD License)
 ************************************************************************/
#ifndef SENSORS_TYPE_HPP_
#define SENSORS_TYPE_HPP_

#include "common/math_base/slam_math.hpp"
#include "common/data_struct/pc_base.hpp"

namespace GR_SLAM {
/**
 * OdomMeasure
 * @brief 里程计测量类
 *
 **/
struct OdomMeasure {
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW
  OdomMeasure() {}
  OdomMeasure(const Mat34d &pose, const double time_stamp) : pose(pose), time_stamp(time_stamp) {}
  OdomMeasure(const Mat34d &pose, const Eigen::Quaterniond Q_gyr_only, const double time_stamp)
      : pose(pose), Q_gyr_only(Q_gyr_only), time_stamp(time_stamp) {}
  OdomMeasure(const Mat34d &pose, const double linear, const double angular,
              const double time_stamp)
      : pose(pose), linear(linear), angular(angular), time_stamp(time_stamp) {}
  Mat34d pose;
  // Eigen::Vector3d linear_velocity;
  // Eigen::Vector3d angular_velocity;
  double linear;
  double angular;
  Eigen::Quaterniond Q_gyr_only;
  double time_stamp;
};  // end of class

/**
 * ImuMeasure
 * @brief imu测量类
 *
 **/
struct ImuMeasure {
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW
  ImuMeasure(){};
  ImuMeasure(const Vec3d acc, const Vec3d gyr, const double time_stamp)
      : acc(acc), gyr(gyr), time_stamp(time_stamp) {}

  Eigen::Vector3d acc = Eigen::Vector3d::Zero();
  Eigen::Vector3d gyr = Eigen::Vector3d::Zero();
  Eigen::Vector3d acc_smooth = Eigen::Vector3d::Zero();
  Eigen::Vector3d gyr_smooth = Eigen::Vector3d::Zero();
  double time_stamp = 0.0;
  Eigen::Quaterniond Q_wi = Eigen::Quaterniond::Identity();
  Eigen::Quaterniond Q_wi_gyr = Eigen::Quaterniond::Identity();
  Eigen::Quaterniond Q_wi_smooth = Eigen::Quaterniond::Identity();
  float gravity_weight = 1.0;
};  // end of class

struct ImageMeasure {
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW
  ImageMeasure(){};
  ImageMeasure(const double &time_stamp, std::vector<u_char> image_data, const uint32_t &height,
               const uint32_t &width, const unsigned char &id)
      : time_stamp(time_stamp), image_data(image_data), height(height), width(width), id(id) {}

  double time_stamp = 0.0;
  std::vector<u_char> image_data;
  uint32_t height;
  uint32_t width;
  unsigned char id;
  Mat34d odom_pose;
  ImuMeasure imu;
  bool odom_sync_flag = false;
  bool imu_sync_flag = false;
};

class Mid_Filter {
 public:
  double data_buf[500], filte_buf[500];
  int buf_size = 499, filter_count = 0, filter_size = 201;

  Mid_Filter(int size) {
    filter_size = size;
    if (filter_size > buf_size) {
      filter_size = buf_size;
      printf(
          "--Mid_Filter: filter_size exceed buf size, and filter_size is set "
          "499 !!! \n");
    }
  }

  double MFilter(double data) {
    double tem = 0;

    data_buf[filter_count] = data;
    filter_count++;
    if (filter_count >= filter_size)
      filter_count = 0;

    for (int i = 0; i < filter_size; i++) { filte_buf[i] = data_buf[i]; }

    for (int i = 0; i < filter_size - 1; i++) {
      for (int j = 0; j < filter_size - i - 1; j++) {
        if (filte_buf[j] > filte_buf[j + 1]) {
          tem = filte_buf[j];
          filte_buf[j] = filte_buf[j + 1];
          filte_buf[j + 1] = tem;
        }
      }
    }
    tem = filte_buf[(filter_size - 1) / 2];
    return tem;
  }
};

/**
 * GPSMeasure
 * @brief gps测量类
 *
 **/
struct GPSMeasure {
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW
  GPSMeasure(const Vec3d &nav_pos, const double time_stamp, const double cov, unsigned int loc_type)
      : nav_pos(nav_pos), time_stamp(time_stamp), cov(cov), loc_type(loc_type) {}
  Vec3d nav_pos;
  double time_stamp;
  double cov;
  unsigned int loc_type;
};  // end of class

/**
 * GpsMsg
 * @brief gps消息类
 *
 **/
typedef struct GpsMsg {
  GpsMsg() = default;
  GpsMsg(const double latitude, const double longitude, const double altitude, const double cov,
         const double time_stamp, const unsigned int loc_type)
      : latitude(latitude),
        longitude(longitude),
        altitude(altitude),
        cov(cov),
        time_stamp(time_stamp),
        loc_type(loc_type) {}
  double latitude;        ///< 纬度
  double longitude;       ///< 经度
  double altitude;        ///< 高度
  double cov;             ///< 定位误差
  double time_stamp;      ///< 时间
  unsigned int loc_type;  ///< 定位解类型： 4-fixed, 5-float

} GpsMsg;  ///< 定位pose数据

/**
 * CloudMeasure
 * @brief cloud测量类
 *
 **/

struct CloudMeasure {
  CloudMeasure(const laserCloud::Ptr cloud_in, const double time_stamp)
      : cloud(cloud_in), time_stamp(time_stamp) {}
  CloudMeasure() = default;
  laserCloud::Ptr cloud;
  double time_stamp;
};  // end of class

/**
 * RobotState
 * @brief robot state类
 *
 **/

struct RobotState {
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW
  RobotState() = default;
  RobotState(const Mat34d &pose, const Vec3d &speed, const double time_stamp, const double cov)
      : pose(pose), speed(speed), time_stamp(time_stamp), cov(cov) {}
  void Reset() {
    pose = Mat34d::Identity();
    speed = Vec3d::Zero();
    time_stamp = -1.;
    cov = -1.;
  }
  Mat34d pose;
  Vec3d speed;
  double time_stamp;
  double cov;
};  // end of class

struct KeyFrameInfo {
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW
  KeyFrameInfo() = default;
  KeyFrameInfo(const size_t frame_id, const Mat34d &pose, const double pose_cov,
               const bool has_gps = false, const Vec3d &gps_pos = Vec3d::Zero(),
               const double gps_cov = -1)
      : frame_id(frame_id),
        pose(pose),
        pose_cov(pose_cov),
        has_gps(has_gps),
        gps_pos(gps_pos),
        gps_cov(gps_cov) {}
  size_t frame_id;
  Mat34d pose;
  double pose_cov;
  bool has_gps;
  Vec3d gps_pos;
  double gps_cov;

};  // end of class

class DataWindow {
 public:
  DataWindow(int size) {
    d_size = size;
    sum = Eigen::Vector3d::Zero();
    ave = Eigen::Vector3d::Zero();
    d_buf.clear();
  }

  ~DataWindow() { d_buf.clear(); }

  u_int32_t d_size = 100;
  std::deque<Eigen::Vector3d> d_buf;
  Eigen::Vector3d ave;
  Eigen::Vector3d sum;

  const Eigen::Vector3d get_ave(const Eigen::Vector3d data) {
    if (d_size < 2) {
      return data;
    }
    d_buf.emplace_back(data);
    if (d_buf.size() > d_size) {
      sum -= d_buf.front();
      d_buf.pop_front();
    }
    sum += data;
    ave = sum / d_buf.size();

    return ave;
  }
};

struct SCD {
  Eigen::MatrixXf desc;
  std::vector<float> RingKey;
  std::vector<float> SectorKey;
};  // end of SCD

}  // namespace GR_SLAM

#endif  // SENSORS_TYPE_HPP_