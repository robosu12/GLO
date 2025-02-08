/************************************************************************
 * Software License Agreement (BSD License)

 *@author Yun Su(robosu12@gmail.com)
 *@version 0.5
 *@data 2022-03-16
 ************************************************************************/
#ifndef SYSTEM_CONFIG_H
#define SYSTEM_CONFIG_H

#include <cmath>
#include <iostream>
#include <vector>
#include <string>
#include "common/debug_tools/debug_color.h"
#include "common/math_base/slam_math.hpp"

namespace GR_SLAM {

enum RelocMode { BOTH, DIS_ONLY, SC_ONLY };

/**
 * GpsConfig
 * @brief gps参数类
 **/
struct GpsConfig {};

/**
 * UwbOdomConfig
 * @brief UwbOdom参数
 **/
struct UwbConfig {};

/**
 * FeatureConfig
 * @brief 点云特征提取参数类
 **/
struct FeatureConfig {};  // end of class

/**
 * LoopConfig
 * @brief 闭环参数类
 **/
struct LoopConfig {};  // end of class

/**
 * BackendConfig
 * @brief 后端参数类
 **/
struct BackendConfig {};  // end of class

/**
 * OccMapConfig
 * @brief 2d 栅格地图参数
 **/
struct OccMapConfig {};  // end of class

/**
 * SystemConfig
 * @brief 系统参数类
 **/
struct SystemConfig {
  /**
   * SystemConfig
   * @brief 系统参数类
   * @param[in] file-参数文件
   **/
  SystemConfig();
  SystemConfig(const std::string &file);

  std::string map_path = "";  ///< 地图保存或载入地址
  bool is_debug = false;
  bool publish_tf = false;
  bool publish_path = false;
  bool publish_cloud = false;

  std::string cloud_topic_name = "/livox";
  std::string scan_topic_name = "/scan";
  std::string uwb_topic_name;
  std::string gps_topic_name;
  std::string vis_gps_topic_name;
  std::string wheel_odom_topic_name = "/odom";
  std::string imu_topic_name = "/imu";
  std::string image_topic_name;
  bool use_lidar = true;
  bool use_odom = false;
  bool use_uwb = false;
  bool use_imu = false;
  bool use_gps = false;
  bool use_vis_gps = false;
  bool use_map_height_constraint = false;
  bool use_gravity_constraint = false;
  bool use_local_ground_constraint = false;
  bool use_global_ground_constraint = false;
  bool use_gyr_constraint = false;

  bool is_localization_mode = false;  // true-定位模式，false-构图模式
  bool plane_mode = false;            // true-平面环境模式，false-3D环境模式
  int global_map_mode = 3;            // 2-2D; 3-3D

  // FeatureConfig feature_config;
  std::string lidar_type = "livox";  ///< 特征提取的方法
  float surf_size = 0.4;
  float vox_size = 0.8;
  int nearby_type = 3;
  int near_search_num = 6;
  int surround_map_size = 40000;

  int lidar_undistortion = 0;  // 0-不畸变校正；1-imu畸变校正；2-里程计畸变校正
  int seg_method = 0;

  float surf_size_odom = 0.4;
  float vox_size_odom = 0.8;
  int nearby_type_odom = 5;
  int near_search_num_odom = 6;
  int surround_map_size_odom = 50000;

  int search_method = 2;  // 1-kdtree; 2-ivox;
  bool use_hierachy_vox = true;
  float high_ratio = 5.0;

  float match_keep_ratio = 0.99;
  bool use_prob_match = false;
  float view_map_min_prob = -1.0;
  float view_map_max_height = 100.0;
  float view_map_min_height = -100.0;

  int min_feature_points = 100;
  float narrow_candidate = 2.0;
  float narrow_candidate_range_ratio = 0.5;
  float narrow_region = 1.5;
  float narrow_ratio = 0.8;
  float small_cluster_size = 0.1;
  float small_cluster_radius = 0.8;
  int small_cluster_num = 3;
  int calculate_degenerate = 0;  // 0: no; 2: for 2D; 3: for 3D;
  float laser_min_range = 0.5;
  float laser_max_range = 30.0;
  float max_match_range = 15.0;
  float laser_min_height = -10.0;
  float laser_max_height = 100.0;
  float occ_min_range = 0.5;
  float occ_max_range = 30.0;
  float occ_min_height = -0.1;
  float occ_max_height = 0.1;
  int near_back_remove = 0;
  bool use_gravity_for_occ = false;
  bool generate_occ_map = false;
  bool generate_2d_cloud_map = false;
  float cloud_map_2d_reso = 0.1;

  // LoopConfig loop_config;
  int time_diff = 60;            ///< 闭环检测时间阈值
  int id_diff = 60;              ///< 闭环id阈值
  float height_diff = 2.5;       ///< 闭环id阈值
  float max_loop_angle = 3.14;   ///<
  float loop_match_error = 0.2;  ///< icp得分阈值
  float cloud_change_ratio = 0.1;
  float drift_ratio = 0.05;
  float loop_search_radius = 8.0;
  float loop_relative_pose = 4.0;
  int last_keyframe_size = 10;
  int history_search_num = 40;
  float loop_cloud_reso = 0.2;
  float relocalization_search_radius = 10.0;
  float relocalization_icp_score = 0.2;
  int relocalization_box_size_x = 2;
  int relocalization_box_size_y = 2;
  int relocalization_box_size_yaw = 2;
  float reloc_match_dis = 8.0;
  int loop_track_count = 8;
  float update_occ_map_time = 20.0;

  // BackendConfig backend_config;
  float angle_threshold = 0.4;  ///< 帧创建角度阈值
  float trans_threshold = 0.4;  ///< 帧创建距离阈值
  float time_threshold = 30.0;  ///< 帧创建时间阈值
  float imu_time_threshold = 0.25;
  float angular_threshold = 0.3;        ///< 帧创建角速度阈值
  float surround_search_radius = 10.0;  ///< 临域关键帧搜索半径
  int surround_search_num = 80;         ///< 最近关键帧集合数量
  bool is_search_by_time = true;        ///< 按时间搜索
  float min_distance = 0.25;            ///< 关键帧创建距离阈值
  int match_algorithm = 3;              ///< 匹配算法： 1-PCL，2-ceres
  float max_match_dis = 1.0;            ///< 匹配点最大距离阈值
  float line_weight = 1.0;
  float corner_weight = 1.0;
  float min_success_match_error = 0.2;
  float min_match_overlap_ratio = 0.4;
  bool save_keyPose_to_file = false;
  bool save_runinfo_to_file = false;
  float odom_match_ratio = 1.0;
  float mapping_match_smooth_ratio = 1.0;
  float fusion_pose_smooth_low_linear = 0.3;
  float fusion_pose_smooth_low_angular = 0.3;
  float fusion_pose_smooth_low_ratio = 0.005;
  float fusion_pose_smooth_ratio = 0.01;
  float lidar_odom_smooth_ratio = 1.0;
  float loc_match_smooth_ratio = 1.0;
  float lidar_odom_match_smooth_ratio = 1.0;
  float loop_cov = 0.2;
  float relative_cov = 0.02;
  int max_match_failed = 200;
  int max_low_overlap = 200;
  bool map_update = false;
  float increase_ratio = 0.5;
  float decrease_ratio = 0.8;
  float increase_duration = 600.0;
  float decrease_duration = 3600.0;
  float static_duration = 200.0;
  float init_map_prob = 60.0;
  float min_match_prob = 40.0;
  float new_add_prob = 30.0;
  float min_erase_prob = 20.0;
  float max_limit_prob = 100.0;
  float accept_interval = 2.2;
  int decrease_interval = 20;
  float new_add_dis = 3.0;
  float cover_radius = 0.6;
  float prob_increase_value = 0.1;
  float degenerate_threshold = 1.2;
  float degenerate_smooth_ratio = 0.01;
  int max_multi_keyframes = 5;          ///< 最大多关键帧融合参数
  int multi_keyframes_cloud_size = 30;  ///< 多关键帧点云数量
  bool is_set_cloud_point_prob = false;
  int max_prob_keyframes = 5;  ///< 概率判断维护局部地图点云帧数
  int prob_point_judge_times = 1;
  bool fix_first_node = true;    ///< 是否固定首个优化节点
  int opt_loop_count = 1;        ///< 多少个闭环才优化
  int new_node_size_to_opt = 1;  ///< 超过多少个新节点进行优化
  int opt_windows_size = 10;     ///< 优化窗口大小
  int keyframe_fusion_size = 0;

  bool fix_gps_extrincs = true;  ///< 是否固定gps外参
  int min_satellites_num = 20;
  float gps_gap_to_opt = 20.0;  ///< 间隔多少m GPS数据进行一次优化
  float gps_local_error_ratio = 0.3;
  float max_gps_ex_init_error = 2.0;
  bool only_esti_gps_ex = false;
  float gps_cov = 3.0;
  int muti_layer_opti_node_gap = 10;
  bool use_gps_init_loc = true;
  bool use_gps_odom = false;
  float min_gps_constrain_delta_trans = 2.0;
  int g_max_slide_windows = 20;
  float gps_loc_cov = 5.0;
  float odom_cov = 0.05;
  float g_max_optimize_dis = 20.0;
  float g_optimize_error_threshold = 1.0;
  float gps_odom_smooth_ratio = 0.1;

  // OccMapConfig occ_map_config;
  float laser_min_angle = -M_PI;
  float laser_max_angle = M_PI;
  float laser_angle_increment = 0.2f / 180 * M_PI;
  float point_size = 0.1;
  int local_map_size = 20;
  bool dynamic_cloud_remove = false;  // 是否使用局部栅格地图移除动态点云
  float local_map_resolution = 0.1;
  float occ_map_resolution = 0.1;
  float low_resolution_ratio = 4.0;
  float occp_ratio = 3.0;
  float free_ratio = 2.0;

  int imu_freq = 200;
  float imu_comp_gain = 0.001;
  int imu_smooth_window = 200;
  float imu_q_smooth_ratio = 0.0005;

  float gravity_weight_ratio = 4.0;
  float ground_weight_ratio = 5.0;
  float gyr_yaw_weight_ratio = 1.0;
  float gyr_pitch_weight_ratio = 0.1;
  float gyr_roll_weight_ratio = 0.1;
  float global_gravity_weight_ratio = 0.05;

  // UwbConfig uwb_config;
  int uwb_type = 2;
  float uwb_min_range = 2.0;
  float uwb_max_range = 200.0;
  int anchor_num = 4;

  bool use_ransac_separated_filter = true;
  int ransac_esti_algorithm = 2;
  bool fix_z_in_ransac = true;
  bool use_ransac_combined_filter = false;
  float outlier_threshold = 0.5;
  float ransac_outlier_rate_threshold = 1.0;
  int ransac_pick_num = 5;
  int ransac_pick_times = 200;
  bool use_unfiltered_data = false;
  bool use_residual_filter = true;
  bool use_reprojection_filter = false;

  int max_slide_windows = 30;
  int slide_windows_time_threshold = 180;
  float uwb_keyframe_delta_trans = 0.6;
  float min_data_num_threshlod_ratio = 0.5;
  float eigenvalue_scale_threshold = 0.0;
  int n_sets_largest_proportion_threshold = 0;
  bool use_ransac_check = false;
  int ransac_times = 3;
  int constrain_first_node_type = 1;
  float first_node_pos_cov = 10.0;
  float first_node_rotate_cov = 5.0;
  float odom_t_cov = 0.03;
  float odom_q_cov = 0.03;
  float uwb_range_cov_ratio = 1.0;
  float max_optimize_dis = 10.0;
  float optimize_score_threshold = 0.7;
  float uwb_odom_smooth_ratio = 1.0;

  bool esti_unknown_anchor_pos_in_localization = false;
  int esti_anchor_pos_time_threshold = 0;
  float range_span_ratio_max = 1.0;
  float range_span_ratio_min = 1.0;
  float priori_anchor_height_max = 2.0;
  float priori_anchor_height_min = 1.0;

  bool joint_optimize_when_saving_map = false;
  bool use_mark_outlier_data = false;
  bool use_uwb_in_msf = false;
  float uwb_weight_in_msf = 0.01;

  bool use_uwb_improve_loop_track = false;
  float uwb_loop_search_radius = 5.0;

  // scd
  bool use_scd = true;
  bool use_scd_augment = true;
  bool use_scd_rkey_only = true;
  int num_scd_candidates = 10;
  float max_scd_mat_distance = 0.3;
  float max_scd_rkey_distance = 60.0;
  float max_scd_range = 60.0;
  float scd_angle_resolution = 6.0;
  float scd_range_resolution = 3.0;
  float scd_max_z = 999.9;
  float scd_min_z = -0.3;

  int dataset_mode = 0;
  std::string kitti_sequence = "00";
  std::string dataset_path = "/home/suyun/CVTE/dataset/dataset-3D/kitti";

  // dense map
  bool is_use_dense_map = false;
  bool is_dm_use_image = false;
  bool pub_dense_map = false;
  bool show_lidar_in_image = false;
  bool use_manual_calibration = false;
  float image_time_offset = 0.05;
  int dm_keyframe_nums = 500;
  float dense_map_height = 1.8;
  float dense_map_reso = 0.05;
  int camera_int_size = 10;
  int camera_nums = 1;
  int width_crop = 1;
  int height_crop = 1;

  Mat34d T_lo = Mathbox::Identity34();
  Mat34d T_li = Mathbox::Identity34();
  Mat34d T_lu = Mathbox::Identity34();
  Mat34d T_cl = Mathbox::Identity34();

  // width height fx fy cx cy k1 k2 k3 k4
  std::vector<float> camera_intrinsics_vecs = {640.0,       480.0,       365.982815,   367.45406,
                                               343.703115,  264.478103,  -0.018745469, -0.05334608,
                                               0.067788082, -0.033810762};
};

}  // namespace GR_SLAM

#endif  // SYSTEM_CONFIG_H
