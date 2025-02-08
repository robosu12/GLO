/************************************************************************
 * Software License Agreement (BSD License)
 *@brief
 * lidar odometry
 *
 *@author Yun Su(robosu12@gmail.com)
 *@version 1.0
 *@data 2024-08-05
 ************************************************************************/

#ifndef LIDAR_ODOM_HPP_
#define LIDAR_ODOM_HPP_

#include <atomic>
#include <condition_variable>
#include <list>
#include <memory>
#include <mutex>

#include "common/math_base/slam_math.hpp"
#include "common/data_struct/pc_base.hpp"
#include "common/iPvox/iPvox.h"
#include "common/data_struct/sensors_type.hpp"
#include "common/data_struct/space_voxel.hpp"

namespace GR_SLAM {
struct SystemConfig;
class KeyFrame;

class LidarOdom {
 public:
  /**
   * LidarOdom
   * @brief 地图跟踪类
   * @param[in] config-参数指针
   **/
  explicit LidarOdom(const std::shared_ptr<SystemConfig> config);

  ~LidarOdom();

  /**
   * Reset
   * @brief 重置
   **/
  void Reset();

  /**
   * InitParameters
   * @brief 系统参数初始化
   *
   **/
  void InitParameters();

  bool isMatchStable() { return is_match_stable_; }

  bool isWheelStable() { return is_wheel_stable_; }

  /**
   * setInitPose
   * @brief 设置初始位置
   * param[in] init pose-初始pose
   * param[in] init_odom-初始里程计
   *
   **/
  void setInitPose(const Mat34d &init_pose, const Mat34d &init_odom);

  /**
   * checkNewKeyFrames
   * @brief 查看列表中是否有等待被插入的关键帧
   * @return true-存在
   */
  bool checkNewKeyFrames();

  /**
   * processNewKeyFrame
   * @brief 处理列表中的关键帧
   */
  bool processNewKeyFrame(std::shared_ptr<KeyFrame> ptr_cur_keyframe, bool is_update_map);

  /**
   * getSurroundingKeyFrames
   * @brief 获取附近的关键帧
   * return true 表示获取成功
   */
  bool getSurroundingKeyFrames();

  bool updateLocalMap();

  /**
   * scanToLocalMap
   * @brief 优化求解的封装函数
   */
  void scanToLocalMap();

  void PCLGICPMatch();

  bool CeresPPICPMatch();

  bool CeresPLICPMatch();

  /**
   * dowmSampleCurFrame
   * @brief 下采样当前帧点云
   * return true 表示采样正常，点太少就会失败
   */
  bool dowmSampleCurFrame();

  /**
   * insertKeyFrame
   * @brief tracking线程向此线程插入关键帧
   * @param[in] ptr_keyframe-帧指针
   */
  void insertKeyFrame(std::shared_ptr<KeyFrame> ptr_keyframe);

  pcl::PointCloud<PointType>::Ptr getCurrSurroundMap();

  /**
   *getMap2Odom
   *@brief
   *获取map到odom的变换
   *
   *@return Mat34d
   **/
  Mat34d getMap2Odom();

  /**
   *setMap2Odom
   *@brief
   *设置map到odom的变换
   *
   *@param[in] map_to_odom－map到odom的变换
   **/
  void setMap2Odom(const Mat34d &map_to_odom);

  /**
   *isKeyFrame
   *@brief
   *当前帧到上个关键帧的几何距离
   *
   **/
  bool isKeyFrame(bool use_rotation = false);

  bool isKeyFrameForMapTrack();

  /**
   *odomDistance2LastFrame
   *@brief
   *根据里程计相对上一帧的运动量
   *
   **/
  bool isJunkFrame(const double time, const Mat34d &cur_odom_pose);

  /**
   *updateTransform
   *@brief
   *更新pose信息
   *
   **/
  void updateTransform();

  void voxMapUpdate();

  std::shared_ptr<KeyFrame> calculateKeyframeProbabilty();

  std::shared_ptr<KeyFrame> getKeyframeMaptrack() { return ptr_keyframe_maptrack_; }

  // 关键帧融合
  void LocalKeyFrameFusion(const bool &map_match_flag);

  void adjustDistortion(const Mat34d &delta_pose, laserCloud::Ptr cloud);

 private:
  std::shared_ptr<KeyFrame> ptr_cur_keyframe_ = nullptr;   ///< 当前帧指针
  std::shared_ptr<KeyFrame> ptr_last_keyframe_ = nullptr;  ///< 当前帧指针
  std::shared_ptr<KeyFrame> ptr_keyframe_maptrack_ = nullptr;

  std::shared_ptr<SystemConfig> config_ = nullptr;

  pclKdTree::Ptr ptr_kdtree_surf_map_;  // localmap平面点云的kdtree搜索结构
  pclKdTree::Ptr ptr_kdtree_key_pose_;

  laserCloud::Ptr surf_cloud_map_;       ///< localmap面点点云指针
  laserCloud::Ptr ds_surf_cloud_map_;    ///< localmap降采样面点点云指针
  laserCloud::Ptr ds_curr_surf_cloud_;   ///< 当前帧降采样面点点云指针
  laserCloud::Ptr ds_curr_surf_in_map_;  ///< 当前帧降采样线点点云指针

  std::deque<laserCloud::Ptr> d_recent_lidar_cloud_in_map_;
  laserCloud::Ptr key_pose_cloud_;

  long unsigned int id_;       ///< 关键帧索引
  size_t match_failed_count_;  ///< 配准失败计数
  size_t keyframe_count_;
  float curr_distance_error_ = 0.0;  ///< 误差距离
  float last_distance_error_;        ///< 误差距离
  double pose_cov_;                  ///< pose协方差
  double predict_error_ratio_;       ///< 预测误差率
  double last_frame_time_;           ///< 上一帧时间戳
  Mat6d cov_;                        ///< 协防差
  Mat34d last_keyframe_pose_;        ///< 上个关键帧的pose
  Mat34d last_frame_pose_;           ///< 上帧的pose
  Mat34d last_frame_odom_pose_;      ///< 上个帧里程计的pose
  Mat34d last_keyframe_odom_pose_;   ///< 上个关键帧里程计的pose
  Mat34d cur_frame_pose_;            ///< 当前关键帧的pose
  Mat34d map_to_odom_;               // TODO: 加锁
  Mat34d last_map_to_odom_;          // TODO: 加锁

  bool is_first_keyframe_;  ///< 第一个关键帧
  bool is_match_ok_;        ///< 匹配是否成功
  bool is_last_match_ok_;   ///< 匹配是否成功
  bool is_match_stable_;    ///< 匹配是否稳定
  bool is_wheel_stable_;    ///< 轮速是否稳定

  std::mutex new_keyframe_mutex_;       ///< 关键帧的锁
  std::mutex stop_mutex_;               ///< 停止标志位锁
  std::mutex correct_pose_mutex_;       ///< pose锁
  std::mutex curr_surround_map_mutex_;  ///< pose锁

  std::condition_variable cv_;  ///< 条件变量

  pclDownsampler downsize_filter_map_surf_;  // 下采样当前帧corner点云
  pclDownsampler downsize_filter_key_pose_;

  int max_match_iter_ = 20;
  float max_match_dis_ = 0.3;

  bool is_enough_prob_match_scan_ = false;  ///< 判断是否足够概率队列长度

  using IVoxType = IVox<3, IVoxNodeType::DEFAULT, PointType>;
  IVoxType::Options ivox_options_;
  std::shared_ptr<IVoxType> ptr_ivox_map_ = nullptr;  // localmap in ivox

  std::vector<PointVector> v_nearest_points_;  // nearest points of current scan

  ImuMeasure cur_imu_;
  ImuMeasure last_imu_;

  std::deque<std::shared_ptr<KeyFrame>> d_recent_keyframe_;

  std::ofstream lidar_odom_pose_file_;

};  // end of class

}  // namespace GR_SLAM

#endif  // LIDAR_ODOM_HPP_