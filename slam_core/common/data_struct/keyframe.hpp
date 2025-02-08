/************************************************************************
 * Software License Agreement (BSD License)

 *@ Yun Su(robosu12@gmail.com)
 *@version 0.5
 *@data 2022-03-16
 ************************************************************************/
#ifndef KRYFRAME_HPP_
#define KRYFRAME_HPP_

#include <memory>
#include <mutex>
#include <atomic>

#include "common/data_struct/pc_base.hpp"
#include "common/math_base/slam_math.hpp"
#include "common/data_struct/sensors_type.hpp"

namespace GR_SLAM {

class KeyFrame {
 public:
  KeyFrame() {}

  /**
   *KeyFrame
   *@brief
   *关键帧构造类
   *
   *@param[in] pos-关键帧位置
   *@param[in] cornerCloud-线点点云
   *@param[in] surfCloud-面点点云
   **/
  KeyFrame(const FramePosition &pos, const laserCloud::Ptr cornerCloud,
           const laserCloud::Ptr surfCloud);

  /**
   *KeyFrame
   *@brief
   *关键帧构造类
   *
   *@param[in] frame_info-关键帧信息
   *@param[in] cornerCloud-线点点云
   *@param[in] surfCloud-面点点云
   **/
  KeyFrame(const KeyFrameInfo &frame_info, const laserCloud::Ptr surfCloud);

  /**
   *KeyFrame
   *@brief
   *关键帧构造类
   *
   *@param[in] ID-关键帧索引
   *@param[in] time_stamp-关键帧时间戳
   *@param[in] T_wo-关键帧里程计pose
   *@param[in] T_wb-关键帧pose
   *@param[in] linear-关键帧linear
   *@param[in] angular-关键帧angular
   *@param[in] cornerCloud-线点点云
   *@param[in] surfCloud-面点点云
   *@param[in] withoutdGroundCloud-没有地面点的点云(下采样)
   **/
  KeyFrame(const size_t ID, const double time_stamp, const Mat34d &T_wo, const Mat34d &T_wl,
           const Mat34d &T_wb, const double linear, const double angular, const ImuMeasure &imu,
           const laserCloud::Ptr surfCloud, const laserCloud::Ptr withoutdGroundCloud);
  /**
   *KeyFrame
   *@brief
   *关键帧构造类 uwb_localization
   *
   *@param[in] ID-关键帧索引
   *@param[in] time_stamp-关键帧时间戳
   *@param[in] T_wo-关键帧里程计pose
   *@param[in] T_wl-关键帧LidarOdom pose
   *@param[in] T_wu-关键帧UwbOdom pose
   *@param[in] T_wb-关键帧pose
   *@param[in] linear-关键帧linear
   *@param[in] angular-关键帧angular
   *@param[in] surfCloud-面点点云
   *@param[in] withoutdGroundCloud-没有地面点的点云(下采样)
   **/
  KeyFrame(const size_t ID, const double time_stamp, const Mat34d &T_wo, const Mat34d &T_wl,
           const Mat34d &T_wu, const Mat34d &T_wb, const double linear, const double angular,
           const ImuMeasure &imu, const laserCloud::Ptr surfCloud,
           const laserCloud::Ptr withoutdGroundCloud);

  ~KeyFrame();

  /**
   *updatePose
   *@brief
   *更新关键帧pose
   *
   *@param[in] pose-关键帧pose
   **/
  void updatePose(const Mat34d &pose) {
    std::unique_lock<std::mutex> lock(mutex_pose_);
    T_wb_ = pose;
    pos_.x = pose(0, 3);
    pos_.y = pose(1, 3);
    pos_.z = pose(2, 3);
  }

  void setBackPose() {
    T_wb_back_.block<3, 1>(0, 3) = t_;
    q_.normalize();
    T_wb_back_.block<3, 3>(0, 0) = q_.toRotationMatrix();
  }

  /**
   *updatePose
   *@brief
   *更新关键帧pose
   *
   *@param[in] pose-关键帧pose
   **/
  void updateLaserOdomPose(const Mat34d &pose) { T_wl_ = pose; }

  inline Mat34d getLaserOdomPose() { return T_wl_; }

  /**
   *updateGpsPos
   *@brief
   *更新keyframe绑定的gps pos 和cov
   *
   *@param[in]
   **/
  void updateGpsPos(const Vec3d &gps_pos, float gps_cov) {
    gps_pos_.x() = gps_pos.x();
    gps_pos_.y() = gps_pos.y();
    gps_pos_.z() = gps_pos.z();
    gps_cov_ = gps_cov;
    has_gps_ = true;
  }

  inline Vec3d getGpsPos() { return gps_pos_; }
  inline double getGpsCov() { return gps_cov_; }

  /**
   *updateOdomForGps
   *@brief
   *更新用于gpsOdom 的里程计pose
   *
   *@param[in] pose-lidarOdom pose
   **/
  void updateOdomForGps(const Mat34d &pose) { odom_for_gps_ = pose; }

  inline Mat34d getOdomForGps() { return odom_for_gps_; }

  /**
   *updateGpsOdomPose
   *@brief
   *更新gpsOdom pose
   *
   *@param[in] pose-gpsOdom pose
   **/
  void updateGpsOdomPose(const Mat34d &pose) { T_wg_ = pose; }

  inline Mat34d getGpsOdomPose() { return T_wg_; }

  void updateGpsOdomPoseConfidence(const double &confidence) {
    gps_odom_pose_confidence_ = confidence;
  }

  void updateImageMeasures(const std::vector<ImageMeasure> &image_measure) {
    image_measure_ = image_measure;
  }

  void updateDenseCloud(const laserCloud::Ptr &dense_cloud) {
    dense_cloud_.reset(new laserCloud());
    dense_cloud_ = dense_cloud;
  }

  void updateEigenVec(const Eigen::Vector3f &eigen_vec) { eigen_vec_ = eigen_vec; }

  /**
   *getWheelPose
   *@brief
   *获取关键帧里程计pose
   *
   *@return Mat34d
   **/
  inline Mat34d getWheelPose() { return T_wo_; }

  void updateWheelPose(const Mat34d &pose) { T_wo_ = pose; }

  /**
   *getMapMatchPose
   *@brief
   *获取关键帧map match pose
   *
   *@return Mat34d
   **/
  inline Mat34d getMapMatchPose() { return T_wm_; }

  /**
   *getPose
   *@brief
   *获取关键帧pose
   *
   *@return Mat34d
   **/
  inline Mat34d getPose() {
    std::unique_lock<std::mutex> lock(mutex_pose_);
    return T_wb_;
  }

  /**
   *getPosition
   *@brief
   *获取关键帧位置
   *
   *@return FramePosition
   **/
  inline FramePosition getPosition() { return pos_; }

  /**
   *getGPSStatus
   *@brief
   *获取关键帧gps状态
   *
   *@return false-无gps，true-有gps
   **/
  inline bool getGPSStatus() { return has_gps_; }

  /**
   *isActived
   *@brief
   *获取关键帧状态
   *
   *@return false-点云无效，true-点云有效
   **/
  bool isActived() const { return is_actived_; }

  /**
   *setActivedSatus
   *@brief
   *设置关键帧状态
   *@param[in] false-点云无效，true-点云有效
   **/
  void setActivedSatus(const bool is_actived) { is_actived_ = is_actived; }

 public:
  size_t index_;       ///< 点云索引ID
  double time_stamp_;  ///< 点云时间戳
  FramePosition pos_;  ///< 数据帧位置
  Mat34d T_wo_;        ///< wheel odom
  Mat34d T_wb_;        ///< body pose
  Mat34d T_wb_back_;   ///< body pose
  Mat34d T_wl_;        ///< laser odom
  Mat34d T_wm_;        ///< map match pose
  double linear;
  double angular;
  double moved_distance_ = 0.0;
  Mat6d laser_odom_cov_;  ///< 雷达里程计协防差
  Mat6d map_cov_;         ///< 地图观测协防差

  bool has_gps_ = false;         ///< gps标志位
  bool gps_avaliable_ = false;   ///< gps标志位
  bool gps_for_opt_ = false;     ///< 节点对应的gps一元边（gps定位point）是否有效
  bool is_sparse_node_ = false;  ///< 分层优化时的第一层稀疏node
  Vec3d gps_pos_;                ///< gps position
  double gps_cov_;               ///< gps covariance>
  Mat34d odom_for_gps_;
  Mat34d T_wg_;  ///< gps odom
  double gps_odom_pose_confidence_;

  laserCloud::Ptr surf_cloud_;
  laserCloud::Ptr occ_cloud_;     ///< 无地面点的点云，下采样
  laserCloud::Ptr fusion_cloud_;  ///< 多关键帧融合点云
  laserCloud::Ptr dense_cloud_;

  bool has_uwb_ = false;  ///< uwb标志位
  Mat34d T_wu_;           ///< uwb odom
  Mat34d odom_for_uwb_;   ///< 用于优化的odom，不管是wheelOdom还是lidarOdom
  double uwb_odom_pose_confidence_;
  Mat34d T_uwb_odom_to_odom_;  // 在frame里存入uwbodom的变换

  bool is_actived_ = true;         ///< 激活状态位
  bool is_wheel_skidded_ = false;  ///< 打滑标志位

  int environment_flag_ = 0;

  // 点云分布特征向量，用于判断退化方向
  Eigen::Vector3f eigen_vec_ = Eigen::Vector3f(999.0, 999.0, 999.0);
  bool degenerate_flag_ = false;

  Eigen::Vector3d t_;
  Eigen::Quaterniond q_;

  bool has_imu_ = false;
  ImuMeasure cur_imu_;

  std::vector<ImageMeasure> image_measure_;

  SCD scd_;

  float ground_height_ = 0.0;

 private:
  std::mutex mutex_pose_;  ///<对象锁>

};  // end of class

typedef std::shared_ptr<KeyFrame> KeyFramePtr;

}  // namespace GR_SLAM

#endif  // KRYFRAME_HPP_