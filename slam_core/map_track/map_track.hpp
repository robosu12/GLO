/************************************************************************
 * Software License Agreement (BSD License)
 *@author Yun Su(robosu12@gmail.com)
 *@version 1.0
 *@data 2024-08-05
 ************************************************************************/

#ifndef MAP_TRACK_HPP_
#define MAP_TRACK_HPP_

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

class MapTrack {
 public:
  /**
   * MapTrack
   * @brief 地图跟踪类
   * @param[in] config-参数指针
   **/
  explicit MapTrack(const std::shared_ptr<SystemConfig> config);

  ~MapTrack();

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

  /**
   * isStopped
   * @brief 是否停止标志
   * return true-成功
   *
   **/
  bool isStopped() { return is_stopped_; }

  /**
   * setInitPose
   * @brief 设置初始位置
   * param[in] init pose-初始pose
   * param[in] init_odom-初始里程计
   *
   **/
  void setInitPose(const Mat34d &init_pose, const Mat34d &init_odom);

  /**
   * relocalization
   * @brief 重定位成功标志
   * return true-成功
   *
   **/
  bool relocalization();

  /**
   * acceptKeyFrames
   * @brief 查看列表中是否有等待被插入的关键帧
   * @return 如果存在，返回true
   */
  bool acceptKeyFrames() { return is_accept_keyframes_; }

  /**
   * setAcceptKeyFrames
   * @brief 设置关键帧插入标志
   * param[in] flag-插入允许标志
   */
  void setAcceptKeyFrames(bool flag) { is_accept_keyframes_ = flag; }

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
   * buildSurroundingMap
   * @brief 获取附近的关键帧
   * return true 表示获取成功
   */
  bool buildSurroundingMap();

  /**
   * Optimization
   * @brief LM优化求解
   * @param[in] iter_count-迭代次数
   */
  bool Optimization(int iter_count);

  /**
   * scanToLocalMap
   * @brief 优化求解的封装函数
   */
  void scanToLocalMap();

  bool CeresPPICPMatch();

  bool CeresPLICPMatch();

  /**
   * dowmSampleCurFrame
   * @brief 下采样当前帧点云
   * return true 表示采样正常，点太少就会失败
   */
  bool dowmSampleCurFrame();

  /**
   * Run
   * @brief MapTrack的主流程
   */
  void Run();

  /**
   * buildKdtree
   * @brief 定位模式下构建kdtree
   */
  bool buildKdtree();

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
  void setMap2Odom(const Mat34d &map_to_odom, bool is_update_map);

  /**
   *getPoseCov
   *@brief
   *获取pose的协方差
   *
   **/
  inline double getPoseCov() const { return pose_cov_; }

  /**
   *isKeyFrame
   *@brief
   *当前帧到上个关键帧的几何距离
   *
   **/
  bool isKeyFrame(bool use_rotation = false);

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

  /**
   *setPoseCov
   *@brief
   *设置pose的协方差
   *
   **/
  void setPoseCov(const double cov);

  size_t getmatchFailedCount() { return localization_failed_count_; }

  size_t getLowOverlapCount() { return low_overlap_count_; }

  void updateMapPointProbability();

  laserCloud::Ptr getSurfMapCloud();

  laserCloud::Ptr getCurrSurfCloud();

  inline void setMatchProbIsEnough(bool is_enough_prob_match_scan) {
    is_enough_prob_match_scan_ = is_enough_prob_match_scan;
  }

  void save_all_keypose_to_file();

  void voxMapReset();
  void voxMapUpdate();

  float getGroundHeight(const laserCloud::Ptr &map_cloud);
  laserCloud::Ptr createOccCloud(float height, laserCloud::Ptr &dense_cloud);
  void updateOccCloud(bool global_opt_flag);

 private:
  std::shared_ptr<KeyFrame> ptr_cur_keyframe_ = nullptr;  ///< 当前帧指针
  std::shared_ptr<KeyFrame> ptr_last_keyframe_ = nullptr;
  std::shared_ptr<KeyFrame> ptr_keyframe_visu_ = nullptr;

  std::shared_ptr<SystemConfig> config_ = nullptr;

  pclKdTree::Ptr ptr_kdtree_surf_map_;  // localmap平面点云的kdtree搜索结构
  pclKdTree::Ptr ptr_kdtree_new_surf_map_;

  laserCloud::Ptr surf_cloud_map_;       ///< localmap面点点云指针
  laserCloud::Ptr ds_surf_cloud_map_;    ///< localmap降采样面点点云指针
  laserCloud::Ptr ds_curr_surf_cloud_;   ///< 当前帧降采样面点点云指针
  laserCloud::Ptr ds_curr_surf_in_map_;  ///< 当前帧降采样线点点云指针

  long unsigned int id_;       ///< 关键帧索引
  size_t match_failed_count_;  ///< 配准失败计数
  float overlap_ratio_ = 1.0;
  size_t low_overlap_count_;  ///< 低重叠度计数
  size_t keyframe_count_;
  float curr_distance_error_ = 0.0;  ///< 误差距离
  float last_distance_error_;        ///< 误差距离
  float pose_cov_;                   ///< pose协方差
  float predict_error_ratio_;        ///< 预测误差率
  float last_frame_time_;            ///< 上一帧时间戳
  Mat6d cov_;                        ///< 协防差
  Mat34d last_keyframe_pose_;        ///< 上个关键帧的pose
  Mat34d last_frame_pose_;           ///< 上帧的pose
  Mat34d last_frame_odom_pose_;      ///< 上个帧里程计的pose
  Mat34d last_keyframe_odom_pose_;   ///< 上个关键帧里程计的pose
  Mat34d cur_frame_pose_;            ///< 当前关键帧的pose
  Mat34d map_to_odom_;               // TODO: 加锁
  Mat34d last_map_to_odom_;          // TODO: 加锁
  float match_cov_;
  float moved_distance_;
  float static_duration_;
  size_t localization_failed_count_;

  std::atomic<bool> is_abortBA_;           ///< 打断标志
  std::atomic<bool> is_stopped_;           ///< 结束标志
  std::atomic<bool> is_not_stop_;          ///< 结束标志
  std::atomic<bool> is_accept_keyframes_;  ///< 关键帧接受标志位

  bool is_first_keyframe_;  ///< 第一个关键帧
  bool is_match_ok_;        ///< 匹配是否成功
  bool is_last_match_ok_;   ///< 匹配是否成功
  bool map_odom_update_flag_;

  std::mutex new_keyframe_mutex_;       ///< 关键帧的锁
  std::mutex stop_mutex_;               ///< 停止标志位锁
  std::mutex accept_mutex_;             ///< 接收标志锁
  std::mutex pose_mutex_;               ///< pose锁
  std::mutex correct_pose_mutex_;       ///< pose锁
  std::mutex cloud_mutex_;              ///< 点云锁
  std::mutex curr_surround_map_mutex_;  ///< pose锁

  std::condition_variable cv_;  ///< 条件变量

  pclDownsampler downsize_filter_surf_;
  ///< 下采样当前帧surf点云
  pclDownsampler downsize_filter_cur_surf_;
  ///< 下采样局部地图surf点云

  std::ofstream lidar_key_pose_file_;

  int max_match_iter_ = 20;
  float max_match_dis_ = 0.3;
  float corner_match_dis_ratio_ = 0.5;
  float line_match_dis_ratio_ = 1.0;

  bool is_enough_prob_match_scan_ = false;

  using IVoxType = IVox<3, IVoxNodeType::DEFAULT, PointType>;
  IVoxType::Options ivox_options_;
  std::shared_ptr<IVoxType> ptr_ivox_map_ = nullptr;  // localmap in ivox

  std::vector<PointVector> v_nearest_points_;  // nearest points of current scan

  ImuMeasure cur_imu_;
  ImuMeasure last_imu_;

  std::shared_ptr<IVoxType> ptr_ivox_ground_map_ = nullptr;
};  // end of class

}  // namespace GR_SLAM

#endif  // MAP_TRACK_HPP_