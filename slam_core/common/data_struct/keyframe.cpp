/************************************************************************
 * Software License Agreement (BSD License)
 *
 *@Yun Su(robosu12@gmail.com)
 *@version 0.5
 *@data 2022-03-16
 ************************************************************************/
#include "common/data_struct/keyframe.hpp"
#include "common/math_base/slam_math.hpp"
#include "glog/logging.h"

namespace GR_SLAM {
KeyFrame::KeyFrame(const FramePosition &pos, const laserCloud::Ptr cornerCloud,
                   const laserCloud::Ptr surfCloud) {
  pos_ = pos;
  index_ = (size_t) pos.intensity;
  surf_cloud_.reset(new laserCloud());
  occ_cloud_.reset(new laserCloud());
  fusion_cloud_.reset(new laserCloud());
  *surf_cloud_ = *surfCloud;
}

KeyFrame::KeyFrame(const KeyFrameInfo &frame_info, const laserCloud::Ptr surfCloud)
    : index_(frame_info.frame_id), time_stamp_(0.), T_wo_(frame_info.pose), T_wb_(frame_info.pose) {
  laser_odom_cov_ = Mat6d::Identity();
  const double &distance_error = frame_info.pose_cov;
  laser_odom_cov_.block<3, 3>(0, 0) = 0.4 * distance_error * Mat3d::Identity();
  laser_odom_cov_.block<3, 3>(3, 3) = distance_error * Mat3d::Identity();
  map_cov_ = laser_odom_cov_;
  pos_.x = frame_info.pose(0, 3);
  pos_.y = frame_info.pose(1, 3);
  pos_.z = frame_info.pose(2, 3);
  surf_cloud_.reset(new laserCloud());
  occ_cloud_.reset(new laserCloud());
  fusion_cloud_.reset(new laserCloud());
  *surf_cloud_ = *surfCloud;
}

KeyFrame::KeyFrame(const size_t ID, const double time_stamp, const Mat34d &T_wo, const Mat34d &T_wl,
                   const Mat34d &T_wb, const double linear, const double angular,
                   const ImuMeasure &imu, const laserCloud::Ptr surfCloud,
                   const laserCloud::Ptr withoutdGroundCloud)
    : index_(ID),
      time_stamp_(time_stamp),
      T_wo_(T_wo),
      T_wl_(T_wl),
      T_wb_(T_wb),
      linear(linear),
      angular(angular) {
  pos_.x = T_wb(0, 3);
  pos_.y = T_wb(1, 3);
  pos_.z = T_wb(2, 3);
  pos_.intensity = ID;
  cur_imu_ = imu;

  surf_cloud_.reset(new laserCloud());
  occ_cloud_.reset(new laserCloud());
  fusion_cloud_.reset(new laserCloud());
  dense_cloud_.reset(new laserCloud());
  *surf_cloud_ = *surfCloud;
  *occ_cloud_ = *withoutdGroundCloud;
}

KeyFrame::~KeyFrame() {}

}  // namespace GR_SLAM