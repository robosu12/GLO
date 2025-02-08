/************************************************************************
 *@file load_params.cpp
 *
 *@brief
 * 相关参数配置类
 *
 *@author YunSu(robosu12@gmail.com)
 *@version v1.0
 *@data 2024-08-10
 ************************************************************************/
#include "load_params.hpp"

namespace GR_SLAM {

LoadParams::LoadParams() {
  ptr_config_ = std::make_shared<SystemConfig>();
}
LoadParams::~LoadParams() {}

void LoadParams::loadSystemConfig(const std::string &config_path) {
  LOG(WARNING) << "loading SystemConfig params from: " << config_path;

  yaml_reader_ = YAML::LoadFile(config_path);

  if (yaml_reader_["grslam"]["t_lo"]) {
    std::vector<float> t_lo = yaml_reader_["grslam"]["t_lo"].as<std::vector<float>>();
    ptr_config_->T_lo << t_lo[0], t_lo[1], t_lo[2], t_lo[3], t_lo[4], t_lo[5], t_lo[6], t_lo[7],
        t_lo[8], t_lo[9], t_lo[10], t_lo[11];
    LOG(INFO) << "T_lo: \n" << ptr_config_->T_lo;
  } else {
    LOG(ERROR) << "Get param failed! t_lo ! ";
  }

  if (yaml_reader_["grslam"]["t_li"]) {
    std::vector<float> t_li = yaml_reader_["grslam"]["t_li"].as<std::vector<float>>();
    ptr_config_->T_li << t_li[0], t_li[1], t_li[2], t_li[3], t_li[4], t_li[5], t_li[6], t_li[7],
        t_li[8], t_li[9], t_li[10], t_li[11];
    LOG(INFO) << "T_li: \n" << ptr_config_->T_li;
  } else {
    LOG(ERROR) << "Get param failed! t_li ! ";
  }

  getParameter("is_debug", ptr_config_->is_debug, false);
  getParameter("publish_tf", ptr_config_->publish_tf, false);
  getParameter("publish_path", ptr_config_->publish_path, false);
  getParameter("publish_cloud", ptr_config_->publish_cloud, false);

  getParameter("cloud_topic_name", ptr_config_->cloud_topic_name, std::string("/livox"));
  getParameter("scan_topic_name", ptr_config_->scan_topic_name, std::string("/scan"));
  getParameter("wheel_odom_topic_name", ptr_config_->wheel_odom_topic_name, std::string("/odom"));
  getParameter("imu_topic_name", ptr_config_->imu_topic_name, std::string("/livox/imu"));

  getParameter("dataset_mode", ptr_config_->dataset_mode, 0);
  getParameter("kitti_sequence", ptr_config_->kitti_sequence, std::string("00"));
  getParameter("dataset_path", ptr_config_->dataset_path,
               std::string("/home/suyun/CVTE/dataset/dataset-3D/kitti"));

  getParameter("use_lidar", ptr_config_->use_lidar, true);
  getParameter("use_odom", ptr_config_->use_odom, false);
  getParameter("use_imu", ptr_config_->use_imu, false);
  getParameter("use_uwb", ptr_config_->use_uwb, false);
  getParameter("use_gps", ptr_config_->use_gps, false);

  getParameter("use_map_height_constraint", ptr_config_->use_map_height_constraint, false);
  getParameter("use_gravity_constraint", ptr_config_->use_gravity_constraint, false);
  getParameter("use_local_ground_constraint", ptr_config_->use_local_ground_constraint, false);
  getParameter("use_global_ground_constraint", ptr_config_->use_global_ground_constraint, false);
  getParameter("use_gyr_constraint", ptr_config_->use_gyr_constraint, false);

  getParameter("lidar_type", ptr_config_->lidar_type, std::string("livox"));
  getParameter("lidar_undistortion", ptr_config_->lidar_undistortion, 0);
  getParameter("seg_method", ptr_config_->seg_method, 0);
  getParameter("small_cluster_radius", ptr_config_->small_cluster_radius, static_cast<float>(0.3));
  getParameter("small_cluster_num", ptr_config_->small_cluster_num, 3);
  getParameter("min_feature_points", ptr_config_->min_feature_points, 100);

  getParameter("laser_min_range", ptr_config_->laser_min_range, static_cast<float>(1.0));
  getParameter("laser_max_range", ptr_config_->laser_max_range, static_cast<float>(60.0));
  getParameter("max_match_range", ptr_config_->max_match_range, static_cast<float>(50.0));
  getParameter("laser_min_height", ptr_config_->laser_min_height, static_cast<float>(-10.0));
  getParameter("laser_max_height", ptr_config_->laser_max_height, static_cast<float>(100.0));
  getParameter("near_back_remove", ptr_config_->near_back_remove, static_cast<int>(0));

  getParameter("search_method", ptr_config_->search_method, 2);
  getParameter("use_hierachy_vox", ptr_config_->use_hierachy_vox, true);
  getParameter("high_ratio", ptr_config_->high_ratio, static_cast<float>(5.0));

  getParameter("surf_size_odom", ptr_config_->surf_size_odom, static_cast<float>(0.4));
  getParameter("vox_size_odom", ptr_config_->vox_size_odom, static_cast<float>(0.8));
  getParameter("nearby_type_odom", ptr_config_->nearby_type_odom, 5);
  getParameter("near_search_num_odom", ptr_config_->near_search_num_odom, 6);
  getParameter("surround_map_size_odom", ptr_config_->surround_map_size_odom, 50000);

  getParameter("surf_size", ptr_config_->surf_size, static_cast<float>(0.4));
  getParameter("vox_size", ptr_config_->vox_size, static_cast<float>(0.8));
  getParameter("nearby_type", ptr_config_->nearby_type, 3);
  getParameter("near_search_num", ptr_config_->near_search_num, 6);
  getParameter("surround_map_size", ptr_config_->surround_map_size, 50000);

  getParameter("plane_mode", ptr_config_->plane_mode, false);
  getParameter("global_map_mode", ptr_config_->global_map_mode, 3);

  getParameter("match_keep_ratio", ptr_config_->match_keep_ratio, static_cast<float>(0.98));
  getParameter("use_prob_match", ptr_config_->use_prob_match, false);
  getParameter("new_add_prob", ptr_config_->new_add_prob, static_cast<float>(30.0));
  getParameter("dynamic_cloud_remove", ptr_config_->dynamic_cloud_remove, false);
  getParameter("keyframe_fusion_size", ptr_config_->keyframe_fusion_size, 1);

  getParameter("trans_threshold", ptr_config_->trans_threshold, static_cast<float>(0.3));
  getParameter("angle_threshold", ptr_config_->angle_threshold, static_cast<float>(0.3));
  getParameter("time_threshold", ptr_config_->time_threshold, static_cast<float>(30.0));
  getParameter("imu_time_threshold", ptr_config_->imu_time_threshold, static_cast<float>(0.25));
  getParameter("angular_threshold", ptr_config_->angular_threshold, static_cast<float>(0.2));
  getParameter("max_match_dis", ptr_config_->max_match_dis, static_cast<float>(0.8));

  getParameter("min_success_match_error", ptr_config_->min_success_match_error,
               static_cast<float>(9.9));
  getParameter("min_match_overlap_ratio", ptr_config_->min_match_overlap_ratio,
               static_cast<float>(0.4));

  getParameter("view_map_min_prob", ptr_config_->view_map_min_prob, static_cast<float>(-1.0));
  getParameter("view_map_max_height", ptr_config_->view_map_max_height, static_cast<float>(100.0));
  getParameter("view_map_min_height", ptr_config_->view_map_min_height, static_cast<float>(-100.0));

  getParameter("save_keyPose_to_file", ptr_config_->save_keyPose_to_file, true);
  getParameter("save_runinfo_to_file", ptr_config_->save_runinfo_to_file, true);

  LOG(WARNING) << "finish load SystemConfig params !";
}

void LoadParams::loadFeatureConfig() {
  getParameter("lidar_type", ptr_config_->lidar_type, std::string("livox"));
  getParameter("surf_size", ptr_config_->surf_size, static_cast<float>(0.5));
  getParameter("vox_size", ptr_config_->vox_size, static_cast<float>(0.5));
  getParameter("use_hierachy_vox", ptr_config_->use_hierachy_vox, true);

  getParameter("lidar_undistortion", ptr_config_->lidar_undistortion, 0);
  getParameter("seg_method", ptr_config_->seg_method, 0);
  getParameter("min_feature_points", ptr_config_->min_feature_points, 100);
  getParameter("small_cluster_radius", ptr_config_->small_cluster_radius, static_cast<float>(0.3));
  getParameter("small_cluster_num", ptr_config_->small_cluster_num, 3);
  getParameter("laser_min_range", ptr_config_->laser_min_range, static_cast<float>(0.5));
  getParameter("laser_max_range", ptr_config_->laser_max_range, static_cast<float>(30.0));
  getParameter("max_match_range", ptr_config_->max_match_range, static_cast<float>(20.0));
  getParameter("laser_min_height", ptr_config_->laser_min_height, static_cast<float>(-10.0));
  getParameter("laser_max_height", ptr_config_->laser_max_height, static_cast<float>(100.0));

  getParameter("calculate_degenerate", ptr_config_->calculate_degenerate, 0);
  getParameter("degenerate_threshold", ptr_config_->degenerate_threshold, static_cast<float>(1.2));
  getParameter("degenerate_smooth_ratio", ptr_config_->degenerate_smooth_ratio,
               static_cast<float>(0.01));
}

void LoadParams::loadBackConfig() {
  getParameter("plane_mode", ptr_config_->plane_mode, false);
  getParameter("global_map_mode", ptr_config_->global_map_mode, 3);

  getParameter("surround_map_size", ptr_config_->surround_map_size, 40000);
  getParameter("search_method", ptr_config_->search_method, 2);
  getParameter("near_search_num", ptr_config_->near_search_num, 5);
  getParameter("nearby_type", ptr_config_->nearby_type, 4);
  getParameter("match_keep_ratio", ptr_config_->match_keep_ratio, static_cast<float>(0.99));
  getParameter("use_prob_match", ptr_config_->use_prob_match, false);
  getParameter("dynamic_cloud_remove", ptr_config_->dynamic_cloud_remove, false);
  getParameter("keyframe_fusion_size", ptr_config_->keyframe_fusion_size, 0);

  getParameter("angle_threshold", ptr_config_->angle_threshold, static_cast<float>(0.4));
  getParameter("trans_threshold", ptr_config_->trans_threshold, static_cast<float>(0.4));
  getParameter("time_threshold", ptr_config_->time_threshold, static_cast<float>(30.0));
  getParameter("imu_time_threshold", ptr_config_->imu_time_threshold, static_cast<float>(0.25));
  getParameter("angular_threshold", ptr_config_->angular_threshold, static_cast<float>(0.3));
  getParameter("max_match_dis", ptr_config_->max_match_dis, static_cast<float>(0.8));
  getParameter("min_success_match_error", ptr_config_->min_success_match_error,
               static_cast<float>(0.2));
  getParameter("min_match_overlap_ratio", ptr_config_->min_match_overlap_ratio,
               static_cast<float>(0.4));

  getParameter("view_map_min_prob", ptr_config_->view_map_min_prob, static_cast<float>(50.0));
  getParameter("view_map_max_height", ptr_config_->view_map_max_height, static_cast<float>(100.0));
  getParameter("view_map_min_height", ptr_config_->view_map_min_height, static_cast<float>(-100.0));

  getParameter("save_keyPose_to_file", ptr_config_->save_keyPose_to_file, false);
  getParameter("save_runinfo_to_file", ptr_config_->save_runinfo_to_file, false);

  getParameter("imu_freq", ptr_config_->imu_freq, 200);
  getParameter("imu_comp_gain", ptr_config_->imu_comp_gain, static_cast<float>(0.001));
  getParameter("imu_smooth_window", ptr_config_->imu_smooth_window, 1);
  getParameter("imu_q_smooth_ratio", ptr_config_->imu_q_smooth_ratio, static_cast<float>(0.001));
  getParameter("gravity_weight_ratio", ptr_config_->gravity_weight_ratio, static_cast<float>(10.0));
  getParameter("ground_weight_ratio", ptr_config_->ground_weight_ratio, static_cast<float>(10.0));
  getParameter("gyr_yaw_weight_ratio", ptr_config_->gyr_yaw_weight_ratio, static_cast<float>(0.2));
  getParameter("gyr_pitch_weight_ratio", ptr_config_->gyr_pitch_weight_ratio,
               static_cast<float>(0.2));
  getParameter("gyr_roll_weight_ratio", ptr_config_->gyr_roll_weight_ratio,
               static_cast<float>(0.2));
  getParameter("global_gravity_weight_ratio", ptr_config_->global_gravity_weight_ratio,
               static_cast<float>(0.05));
}

void LoadParams::loadOccMapConfig() {
  getParameter("occ_map_resolution", ptr_config_->occ_map_resolution, static_cast<float>(0.1));
  getParameter("low_resolution_ratio", ptr_config_->low_resolution_ratio, static_cast<float>(4.0));

  getParameter("occ_min_range", ptr_config_->occ_min_range, static_cast<float>(0.5));
  getParameter("occ_max_range", ptr_config_->occ_max_range, static_cast<float>(30.0));
  getParameter("occ_min_height", ptr_config_->occ_min_height, static_cast<float>(-0.5));
  getParameter("occ_max_height", ptr_config_->occ_max_height, static_cast<float>(-0.2));
  getParameter("near_back_remove", ptr_config_->near_back_remove, static_cast<int>(0));
  getParameter("use_gravity_for_occ", ptr_config_->use_gravity_for_occ, static_cast<bool>(false));
  getParameter("generate_occ_map", ptr_config_->generate_occ_map, static_cast<bool>(false));
  getParameter("generate_2d_cloud_map", ptr_config_->generate_2d_cloud_map,
               static_cast<bool>(false));
  getParameter("cloud_map_2d_reso", ptr_config_->cloud_map_2d_reso, static_cast<float>(0.1));
}

void LoadParams::loadLoopConfig() {
  getParameter("loop_match_error", ptr_config_->loop_match_error, static_cast<float>(0.2));
  getParameter("cloud_change_ratio", ptr_config_->cloud_change_ratio, static_cast<float>(0.1));
  getParameter("time_diff", ptr_config_->time_diff, 60);
  getParameter("max_loop_angle", ptr_config_->max_loop_angle, static_cast<float>(3.14));
  getParameter("height_diff", ptr_config_->height_diff, static_cast<float>(2.5));
  getParameter("loop_search_radius", ptr_config_->loop_search_radius, static_cast<float>(12.0));
  getParameter("loop_relative_pose", ptr_config_->loop_relative_pose, static_cast<float>(8.0));
  getParameter("drift_ratio", ptr_config_->drift_ratio, static_cast<float>(0.06));
  getParameter("last_keyframe_size", ptr_config_->last_keyframe_size, 10);
  getParameter("history_search_num", ptr_config_->history_search_num, 40);
  getParameter("loop_cloud_reso", ptr_config_->loop_cloud_reso, static_cast<float>(0.2));
  getParameter("loop_cov", ptr_config_->loop_cov, static_cast<float>(0.2));
  getParameter("relative_cov", ptr_config_->relative_cov, static_cast<float>(0.02));

  getParameter("fix_first_node", ptr_config_->fix_first_node, true);
  getParameter("new_node_size_to_opt", ptr_config_->new_node_size_to_opt, 1);
  getParameter("opt_windows_size", ptr_config_->opt_windows_size, 10);
  getParameter("opt_loop_count", ptr_config_->opt_loop_count, 1);

  getParameter("relocalization_icp_score", ptr_config_->relocalization_icp_score,
               static_cast<float>(0.2));
  getParameter("relocalization_search_radius", ptr_config_->relocalization_search_radius,
               static_cast<float>(12.0));
  getParameter("reloc_match_dis", ptr_config_->reloc_match_dis, static_cast<float>(8.0));
  getParameter("relocalization_box_size_x", ptr_config_->relocalization_box_size_x, 1);
  getParameter("relocalization_box_size_y", ptr_config_->relocalization_box_size_y, 1);
  getParameter("relocalization_box_size_yaw", ptr_config_->relocalization_box_size_yaw, 1);

  getParameter("loop_track_count", ptr_config_->loop_track_count, 8);
  getParameter("update_occ_map_time", ptr_config_->update_occ_map_time, static_cast<float>(20.0));
}

void LoadParams::loadScanContextConfig() {
  getParameter("use_scd", ptr_config_->use_scd, static_cast<bool>(true));
  getParameter("use_scd_augment", ptr_config_->use_scd_augment, static_cast<bool>(true));
  getParameter("use_scd_rkey_only", ptr_config_->use_scd_rkey_only, static_cast<bool>(true));
  getParameter("num_scd_candidates", ptr_config_->num_scd_candidates, static_cast<int>(5));
  getParameter("max_scd_mat_distance", ptr_config_->max_scd_mat_distance, static_cast<float>(0.3));
  getParameter("max_scd_rkey_distance", ptr_config_->max_scd_rkey_distance,
               static_cast<float>(80.0));
  getParameter("max_scd_range", ptr_config_->max_scd_range, static_cast<float>(60.0));
  getParameter("scd_angle_resolution", ptr_config_->scd_angle_resolution, static_cast<float>(6.0));
  getParameter("scd_range_resolution", ptr_config_->scd_range_resolution, static_cast<float>(3.0));
  getParameter("scd_max_z", ptr_config_->scd_max_z, static_cast<float>(999.9));
  getParameter("scd_min_z", ptr_config_->scd_min_z, static_cast<float>(-0.3));
}

void LoadParams::loadDenseMaptConfig() {
  getParameter("is_use_dense_map", ptr_config_->is_use_dense_map, static_cast<bool>(false));
  getParameter("is_dm_use_image", ptr_config_->is_dm_use_image, static_cast<bool>(false));
  getParameter("pub_dense_map", ptr_config_->pub_dense_map, static_cast<bool>(false));
  getParameter("show_lidar_in_image", ptr_config_->show_lidar_in_image, static_cast<bool>(false));
  getParameter("use_manual_calibration", ptr_config_->use_manual_calibration,
               static_cast<bool>(false));
  getParameter("image_time_offset", ptr_config_->image_time_offset, static_cast<float>(0.05));
  getParameter("dm_keyframe_nums", ptr_config_->dm_keyframe_nums, static_cast<int>(500));
  getParameter("dense_map_height", ptr_config_->dense_map_height, static_cast<float>(1.8));
  getParameter("dense_map_reso", ptr_config_->dense_map_reso, static_cast<float>(0.05));
  getParameter("width_crop", ptr_config_->width_crop, static_cast<int>(1));
  getParameter("height_crop", ptr_config_->height_crop, static_cast<int>(1));
  getParameter("camera_intrinsic_size", ptr_config_->camera_int_size, static_cast<int>(10));
  getParameter("camera_nums", ptr_config_->camera_nums, static_cast<int>(1));

  if (yaml_reader_["grslam"]["t_cl"]) {
    std::vector<float> t_cl = yaml_reader_["grslam"]["t_cl"].as<std::vector<float>>();
    ptr_config_->T_cl << t_cl[0], t_cl[1], t_cl[2], t_cl[3], t_cl[4], t_cl[5], t_cl[6], t_cl[7],
        t_cl[8], t_cl[9], t_cl[10], t_cl[11];
    LOG(INFO) << "T_cl: \n" << ptr_config_->T_cl;
  } else {
    LOG(ERROR) << "Get param failed! t_cl ! ";
  }

  if (yaml_reader_["grslam"]["camera_intrinsics_vecs"]) {
    ptr_config_->camera_intrinsics_vecs =
        yaml_reader_["grslam"]["camera_intrinsics_vecs"].as<std::vector<float>>();
  } else {
    LOG(ERROR) << "Get param failed! camera_intrinsics_vecs ! ";
  }
}

void LoadParams::loadGpsConfig() {
  getParameter("min_satellites_num", ptr_config_->min_satellites_num, static_cast<int>(20));
  getParameter("gps_gap_to_opt", ptr_config_->gps_gap_to_opt, static_cast<float>(20.0));
  getParameter("gps_local_error_ratio", ptr_config_->gps_local_error_ratio,
               static_cast<float>(0.3));
  getParameter("max_gps_ex_init_error", ptr_config_->max_gps_ex_init_error,
               static_cast<float>(2.0));
  getParameter("only_esti_gps_ex", ptr_config_->only_esti_gps_ex, static_cast<bool>(false));
  getParameter("gps_cov", ptr_config_->gps_cov, static_cast<float>(3.0));
  getParameter("muti_layer_opti_node_gap", ptr_config_->muti_layer_opti_node_gap,
               static_cast<int>(10));
  getParameter("use_gps_init_loc", ptr_config_->use_gps_init_loc, static_cast<bool>(true));
  getParameter("use_gps_odom", ptr_config_->use_gps_odom, static_cast<bool>(false));
  getParameter("min_gps_constrain_delta_trans", ptr_config_->min_gps_constrain_delta_trans,
               static_cast<float>(2.0));
  getParameter("g_max_slide_windows", ptr_config_->g_max_slide_windows, static_cast<int>(20));
  getParameter("gps_loc_cov", ptr_config_->gps_loc_cov, static_cast<float>(5.0));
  getParameter("odom_cov", ptr_config_->odom_cov, static_cast<float>(0.05));
  getParameter("g_max_optimize_dis", ptr_config_->g_max_optimize_dis, static_cast<float>(20.0));
  getParameter("g_optimize_error_threshold", ptr_config_->g_optimize_error_threshold,
               static_cast<float>(1.0));
  getParameter("gps_odom_smooth_ratio", ptr_config_->gps_odom_smooth_ratio,
               static_cast<float>(0.1));
}

void LoadParams::loadUwbOdomConfig() {
  getParameter("uwb_type", ptr_config_->uwb_type, static_cast<int>(2));
  getParameter("uwb_min_range", ptr_config_->uwb_min_range, static_cast<float>(2.0));
  getParameter("uwb_max_range", ptr_config_->uwb_max_range, static_cast<float>(200.0));
  getParameter("anchor_num", ptr_config_->anchor_num, static_cast<int>(4));

  getParameter("use_ransac_separated_filter", ptr_config_->use_ransac_separated_filter, true);
  getParameter("ransac_esti_algorithm", ptr_config_->ransac_esti_algorithm, static_cast<int>(2));
  getParameter("fix_z_in_ransac", ptr_config_->fix_z_in_ransac, true);
  getParameter("use_ransac_combined_filter", ptr_config_->use_ransac_combined_filter, false);
  getParameter("outlier_threshold", ptr_config_->outlier_threshold, static_cast<float>(0.5));
  getParameter("ransac_outlier_rate_threshold", ptr_config_->ransac_outlier_rate_threshold,
               static_cast<float>(1.0));
  getParameter("ransac_pick_num", ptr_config_->ransac_pick_num, static_cast<int>(5));
  getParameter("ransac_pick_times", ptr_config_->ransac_pick_times, static_cast<int>(200));
  getParameter("use_unfiltered_data", ptr_config_->use_unfiltered_data, false);
  getParameter("use_residual_filter", ptr_config_->use_residual_filter, true);
  getParameter("use_reprojection_filter", ptr_config_->use_reprojection_filter, false);

  getParameter("max_slide_windows", ptr_config_->max_slide_windows, static_cast<int>(30));
  getParameter("slide_windows_time_threshold", ptr_config_->slide_windows_time_threshold,
               static_cast<int>(180));
  getParameter("uwb_keyframe_delta_trans", ptr_config_->uwb_keyframe_delta_trans,
               static_cast<float>(0.6));
  getParameter("min_data_num_threshlod_ratio", ptr_config_->min_data_num_threshlod_ratio,
               static_cast<float>(0.5));
  getParameter("eigenvalue_scale_threshold", ptr_config_->eigenvalue_scale_threshold,
               static_cast<float>(0.0));
  getParameter("n_sets_largest_proportion_threshold",
               ptr_config_->n_sets_largest_proportion_threshold, static_cast<int>(0));
  getParameter("use_ransac_check", ptr_config_->use_ransac_check, false);
  getParameter("ransac_times", ptr_config_->ransac_times, static_cast<int>(3));
  getParameter("constrain_first_node_type", ptr_config_->constrain_first_node_type,
               static_cast<int>(1));
  getParameter("first_node_pos_cov", ptr_config_->first_node_pos_cov, static_cast<float>(10.0));
  getParameter("first_node_rotate_cov", ptr_config_->first_node_rotate_cov,
               static_cast<float>(5.0));
  getParameter("odom_t_cov", ptr_config_->odom_t_cov, static_cast<float>(0.03));
  getParameter("odom_q_cov", ptr_config_->odom_q_cov, static_cast<float>(0.03));
  getParameter("uwb_range_cov_ratio", ptr_config_->uwb_range_cov_ratio, static_cast<float>(1.0));
  getParameter("max_optimize_dis", ptr_config_->max_optimize_dis, static_cast<float>(10.0));
  getParameter("optimize_score_threshold", ptr_config_->optimize_score_threshold,
               static_cast<float>(0.7));
  getParameter("uwb_odom_smooth_ratio", ptr_config_->uwb_odom_smooth_ratio,
               static_cast<float>(1.0));

  getParameter("esti_unknown_anchor_pos_in_localization",
               ptr_config_->esti_unknown_anchor_pos_in_localization, false);
  getParameter("use_mark_outlier_data", ptr_config_->use_mark_outlier_data, false);
  getParameter("esti_anchor_pos_time_threshold", ptr_config_->esti_anchor_pos_time_threshold,
               static_cast<int>(0));
  getParameter("range_span_ratio_max", ptr_config_->range_span_ratio_max, static_cast<float>(1.0));
  getParameter("range_span_ratio_min", ptr_config_->range_span_ratio_min, static_cast<float>(1.0));
  getParameter("priori_anchor_height_max", ptr_config_->priori_anchor_height_max,
               static_cast<float>(2.0));
  getParameter("priori_anchor_height_min", ptr_config_->priori_anchor_height_min,
               static_cast<float>(1.0));

  getParameter("joint_optimize_when_saving_map", ptr_config_->joint_optimize_when_saving_map,
               false);
  getParameter("use_uwb_in_msf", ptr_config_->use_uwb_in_msf, false);
  getParameter("uwb_weight_in_msf", ptr_config_->uwb_weight_in_msf, static_cast<float>(0.01));

  getParameter("use_uwb_improve_loop_track", ptr_config_->use_uwb_improve_loop_track, false);
  getParameter("uwb_loop_search_radius", ptr_config_->uwb_loop_search_radius,
               static_cast<float>(5.0));
}

template <typename ParameterT>
void LoadParams::getParameter(const std::string &name, ParameterT &value,
                              const ParameterT &alternative_value) const {
  if (yaml_reader_["grslam"][name]) {
    value = yaml_reader_["grslam"][name].as<ParameterT>();
    LOG(INFO) << "Read param: " << name << " value:" << value;
  } else {
    value = alternative_value;
    LOG(ERROR) << "Get param failed! param: " << name << ", default value: " << value;
  }
}

}  // namespace GR_SLAM
