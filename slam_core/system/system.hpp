/************************************************************************
 * Software License Agreement (BSD License)

 *@author Yun Su(robosu12@gmail.com)
 *@version 1.0
 *@data 2024-08-05
 ************************************************************************/
#ifndef SYSTEM_HPP_
#define SYSTEM_HPP_

#include <glog/logging.h>

#include <atomic>
#include <deque>
#include <list>
#include <string>
#include <thread>
#include <iostream>
#include <fstream>
#include <map>

#include "common/config/system_config.hpp"
#include "common/data_struct/keyframe.hpp"
#include "common/data_struct/sensors_type.hpp"
#include "common/debug_tools/tic_toc.h"
#include "common/data_struct/space_voxel.hpp"
#include "frontend/base_feature_extractor.hpp"
#include "lidar_odom/lidar_odom.hpp"
#include "map_track/map_track.hpp"
#include "common/metric/CPUMetric.h"
#include "common/metric/MemoryMetric.h"
#include "load_params/load_params.hpp"

namespace GR_SLAM {
enum INIT_MODEL { NOTINIT = 0, RELIABLE_POSE, FIX_POSE, FIX_POSE_NOYAW, GPS, LIDAR };

/**
 * System
 * @brief 系统类
 **/
class System {
 public:
  ~System();

  static System *getInstance();

  void setConfigPath(const std::string &config_path) {
    config_path_ = config_path;
    LOG(WARNING) << "config_file_path: " << config_path_;
  }

  void setConfig(const std::shared_ptr<SystemConfig> ptr_config);

  std::shared_ptr<SystemConfig> getConfig() { return ptr_config_; }

  /**
   * setInitModel
   * @brief 设置初始化模式
   * @param[in] init_model-初始化模式
   **/
  void setInitModel(const INIT_MODEL init_model);

  /**
   * setInitPose
   * @brief 设置初始化模式
   * @param[in] init_pose-初始化姿态
   **/
  void setInitPose(const Mat34d &init_pose);

  void setReliablePose(const Mat34d &reliable_pose);

  void Init(const std::string &config_path);

  /**
   * addOdom
   * @brief 添加odom数据
   * @param[in] odom-odom数据
   **/
  void addOdom(const OdomMeasure &odom);
  void addOdom(const Mat34d &pose, const double time_stamp);

  void addIMU(const ImuMeasure &imu);
  void addIMU(const Vec3d &acc, const Vec3d &gyro, const double time_stamp);
  /**
   * addCloudFrame
   * @brief 跟踪帧
   * @param[in] cloud_in-输入点云
   * @param[in] time_stamp-时间戳
   **/
  void addCloudFrame(const CloudMeasure &cloud);
  void addCloudFrame(const laserCloud::Ptr cloud_in, const double time_stamp);

  void addImage(const ImageMeasure &image_data);
  bool getCurrImage(const double &cloud_time, std::vector<ImageMeasure> &v_image);

  bool AddRangeToKeyframe(const std::shared_ptr<KeyFrame> &cur_keyframe,
                          const std::shared_ptr<KeyFrame> &last_keyframe);

  bool AddGpsToKeyframe(const std::shared_ptr<KeyFrame> &cur_keyframe);

  void checkGpsAvaliable(std::shared_ptr<KeyFrame> ptr_cur_keyframe);

  bool evaluateGpsError();

  /**
   * creatFrame
   * @brief 生成待处理帧
   **/
  bool creatFrame();
  /**
   * failueDetect
   * @brief 失效检测
   **/
  bool failueDetect();

  /**
   * getPoseCov
   * @brief 获取pose协方差
   **/
  double getPoseCov() { return ptr_map_track_->getPoseCov(); }

  /**
   * Reset
   * @brief 重置
   **/
  void Reset();

  /**
   * shutdown
   * @brief 关闭系统
   **/
  void shutdown();

  const laserCloud::Ptr getCurKeyFrameSurf();
  const laserCloud::Ptr getCurrSurroundCloud();
  const laserCloud::Ptr getKeyPoseCloud();

  const RobotState &getRobotState();
  const RobotState getLidarOdomState();
  const Mat34d getKeyFramePose();
  const Mat34d getKeyFrameLidarOdomPose();
  const Mat34d getKeyFrameUwbOdomPose();
  const Mat34d getKeyFrameGpsOdomPose();
  const Mat34d getKeyFrameWheelOdomPose();

  const SCD getCurKeyFrameSCD();

  double getCurKeyFrameTimeStamp();

  void requestStop();

 public:
  // 地图和可视化信息
  double curr_keyframe_time_ = 0.0;  // 关键帧时间
  Mat34d curr_keyframe_pose_;
  laserCloud::Ptr cloud_in_;    // 输入点云指针
  laserCloud::Ptr surf_cloud_;  // 全局点云指针
  laserCloud::Ptr occ_cloud_;   // 用于occ map的点云
  laserCloud::Ptr key_pose_cloud_;

  // 获取定位状态
  bool getLocalizationStatus();

  inline bool getInitMapToOdomFlag() { return set_init_map_to_odom_flag_; }

  const Mat34d getInitMapToOdomTransform() { return init_map_to_odom_; }

  // 有uwb时就是到uwbOdom的变换 有gps时就是到gpsOdom的变换
  const Mat34d getMapToLidarOdomTransform() { return cur_map_to_odom_; }

  const Mat34d getLidarOdomToWheelOdomTransform() { return lidar_odom_to_odom_; }

  void saveCurPoseToFile();

  void calculateTrajError();

  bool getOdomInterval(double t0, double t1, std::vector<OdomMeasure> &v_odom);

  bool getImuInterval(double t0, double t1, std::vector<ImuMeasure> &v_imu);

  bool getImuSync(double t, ImuMeasure &imu);

  const ImuMeasure getCurIMU() { return cur_imu_; }

 private:
  /**
   * System
   * @brief 系统类
   * @param[in] config_file-参数指针
   **/
  System();

  void lidarOdomThread();

  void dynamicLocalMapThread();

  /**
   * mapTrackThread
   * @brief 地图跟踪线程
   **/
  void mapTrackThread();

  /**
   * loopTrackThread
   * @brief 闭环检测线程
   **/
  void loopTrackThread();

  void initLocalizationThread();

  // 历史信息记录

  Mat34d init_map_to_odom_;          ///< 上一地图转换
  Mat34d last_map_to_odom_;          ///< 上一地图转换
  Mat34d cur_map_to_odom_;           ///< 当前地图转换
  Mat34d last_success_map_to_odom_;  ///< 上一次匹配成功的地图转换
  Mat34d last_lidar_odom_to_odom_;   ///< 当前地图转换
  Mat34d lidar_odom_to_odom_;        ///< 当前地图转换
  Mat34d uwb_odom_to_odom_;          ///< UwbOdom到Odom的转换
  Mat34d gps_odom_to_odom_;          ///< GpsOdom到Odom的转换
  Mat34d last_frame_pose_;           ///< 上一地图转换
  Mat34d cur_frame_pose_;            ///< 当前地图转换

  Mat34d curr_correspond_odom_pose_;      ///<
  Mat34d last_correspond_odom_pose_;      ///<
  Mat34d curr_uwb_correspond_odom_pose_;  ///< 在uwb时间戳插值的wheelOdom pose
  Mat34d last_uwb_correspond_odom_pose_;  ///<
  Mat34d init_imu_pose_;                  ///<
  Mat34d curr_lidar_odom_pose_;           ///<
  Mat34d last_lidar_odom_pose_;

  RobotState cur_robot_state_;  ///< 当前机器人状态
  ImuMeasure cur_imu_;
  ImuMeasure last_imu_;
  ImuMeasure last_key_imu_;

  bool is_gpos_offset_init_;
  Vec3d init_gpos_;         ///< gps初始坐标 建图时save
  Vec3d init_gpos_offset_;  ///< gps初始坐标 定位时load
  Vec3d cur_gpos_;          ///< gps坐标 供显示用
  Vec3d cur_g2pos_;         ///< gps坐标 供显示用
  Mat34d T_gps_ex_;         ///< gps外参
  bool load_gps_ex_;        ///< 定位时是否以加载gps外参

  // 系统数据容器

  std::map<double, OdomMeasure> d_odom_measures_;  ///< 轮速里程计测量字典容器
  std::deque<ImuMeasure> d_imu_measures_;          ///< Imu测量队列容器
  std::deque<CloudMeasure> d_cloud_measures_;      ///< 点晕测量队列容器

  // 容器数据锁
  std::mutex gps_data_mutex_;    ///< gps   数据锁
  std::mutex uwb_data_mutex_;    ///< uwb   数据锁
  std::mutex odom_data_mutex_;   ///< odom  数据锁
  std::mutex imu_data_mutex_;    ///< imu   数据锁
  std::mutex cloud_data_mutex_;  ///< cloud 数据锁
  std::mutex keyframe_mutex_;    ///< 关键帧锁>

  // 系统子模块
  std::shared_ptr<SystemConfig> ptr_config_ = nullptr;  ///< 系统参数配置指针
  std::shared_ptr<LoadParams> ptr_load_params_ = nullptr;

  std::shared_ptr<LidarOdom> ptr_lidar_odom_ = nullptr;  ///< 地图跟踪模块指针
  std::shared_ptr<MapTrack> ptr_map_track_ = nullptr;    ///< 地图跟踪模块指针
  std::shared_ptr<BaseFeatureExtractor> ptr_feature_extractor_ = nullptr;  ///< 特征提取模块指针

  std::shared_ptr<KeyFrame> ptr_cur_keyframe_ = nullptr;  ///< 当前帧指针
  std::shared_ptr<KeyFrame> ptr_cur_lidar_odom_keyframe_ = nullptr;

  // 系统状态和标志位
  bool is_first_cloud_ = true;     ///< 第一个lidar数据标志
  bool is_first_odom_ = true;      ///< 第一个里程计数据标志
  bool is_first_keyframe_ = true;  ///< 第一帧点云数据
  bool new_keyframe_has_uwb_ = false;
  bool new_keyframe_flag_ = false;
  bool is_feature_data_ready_ = false;  ///< 特征数据初始化成功标志
  bool is_match_ok_ = false;
  size_t id_;                     ///< 帧id
  std::atomic<bool> is_stopped_;  ///< 是否停止
  double temp_frame_time_ = 0.;   ///< 当前帧时间
  double last_frame_time_ = 0.;   ///< 上一帧时间
  size_t odom_count_ = 0;
  size_t uwb_count_ = 0;
  size_t gps_count_ = 0;
  size_t imu_count_ = 0;
  size_t lidar_count_ = 0;

  CPUMetric cpu_metric;

  // 子模块多线程
  std::thread *ptr_lidarOdom_thread_;  ///< 地图跟踪线程指针
  std::thread *ptr_maptrack_thread_;   ///< 地图跟踪线程指针

  // 判断定位成功标志位
  bool is_localization_sucess_ = false;
  bool is_init_localization_sucess_ = false;

  bool set_init_map_to_odom_flag_ = false;

  std::thread save_keyframe_thread_;
  std::thread save_map_thread_;

  bool finish_save_keyframe_ = false;
  bool finish_save_map_ = false;
  bool finish_save_scandata_ = false;

  // 数据重置控制位
  bool is_gps_odom_working_ = false;
  bool is_uwb_odom_working_ = false;
  bool is_lidar_odom_working_ = false;
  bool is_map_track_working_ = false;
  bool is_loop_thread_working_ = false;
  bool is_occupy_thread_working_ = false;

  std::ofstream lidar_odom_pose_file_;
  std::ofstream lidar_key_pose_file_;
  std::ofstream runinfo_result_;

  // < cpu memory time ave_time >
  std::vector<std::vector<float>> v_run_info_;

  std::string config_path_ = "src/slam_3d/params/3DSLAM_RS16.yaml";
  std::string result_path_ = "~/";

};  // end of class

}  // namespace GR_SLAM

#endif  // SYSTEM_HPP_