/************************************************************************
 * Software License Agreement (BSD License)

 ************************************************************************/

#include "map_track/map_track.hpp"

#include <glog/logging.h>

#include "common/pose_graph_error_term.hpp"
#include "common/math_base/slam_transform.hpp"
#include "common/data_struct/keyframe.hpp"
#include "common/config/system_config.hpp"
#include "common/debug_tools/tic_toc.h"

namespace GR_SLAM {
MapTrack::MapTrack(const std::shared_ptr<SystemConfig> config) : config_(config) {
  InitParameters();
}

MapTrack::~MapTrack() {}

void MapTrack::InitParameters() {
  downsize_filter_surf_.setLeafSize(config_->surf_size, config_->surf_size, config_->surf_size);
  Reset();
}

void MapTrack::Reset() {
  match_failed_count_ = 0;
  low_overlap_count_ = 0;
  localization_failed_count_ = 0;

  pose_cov_ = 10.;
  predict_error_ratio_ = 0.0;
  last_frame_time_ = -1.;

  last_frame_pose_ = Mathbox::Identity34();
  last_keyframe_pose_ = Mathbox::Identity34();
  last_frame_odom_pose_ = Mathbox::Identity34();
  last_keyframe_odom_pose_ = Mathbox::Identity34();
  map_to_odom_ = Mathbox::Identity34();
  last_map_to_odom_ = Mathbox::Identity34();

  cur_frame_pose_ = Mathbox::Identity34();
  keyframe_count_ = 0;
  is_first_keyframe_ = true;
  is_accept_keyframes_ = true;
  is_match_ok_ = true;
  is_last_match_ok_ = true;
  is_stopped_ = false;

  map_odom_update_flag_ = true;

  ptr_cur_keyframe_ = std::make_shared<KeyFrame>();
  ptr_last_keyframe_ = std::make_shared<KeyFrame>();

  surf_cloud_map_.reset(new laserCloud());
  ds_surf_cloud_map_.reset(new laserCloud());
  ds_curr_surf_cloud_.reset(new laserCloud());

  ptr_kdtree_surf_map_.reset(new pclKdTree());
  ptr_kdtree_new_surf_map_.reset(new pclKdTree());

  moved_distance_ = 0.0;
  static_duration_ = 0.0;

  ds_curr_surf_in_map_.reset(new laserCloud());

  ivox_options_.resolution_ = config_->vox_size;
  ivox_options_.capacity_ = config_->surround_map_size;
  ivox_options_.init_prob_ = config_->new_add_prob;
  ivox_options_.max_range_ = config_->laser_max_range;
  ivox_options_.near_num_ = config_->near_search_num;
  ivox_options_.nearby_type_ = config_->nearby_type;
  ivox_options_.use_hierachy_vox_ = config_->use_hierachy_vox;
  ivox_options_.high_ratio_ = config_->high_ratio;
  if (ptr_ivox_map_ != nullptr) {
    ptr_ivox_map_->Clear();
  }
  ptr_ivox_map_ = std::make_shared<IVoxType>(ivox_options_);

  v_nearest_points_.resize(config_->surround_map_size);

  LOG(WARNING) << "MapTrack::Reset !!!";
}

bool MapTrack::processNewKeyFrame(std::shared_ptr<KeyFrame> ptr_cur_keyframe, bool is_update_map) {
  if (nullptr == ptr_cur_keyframe) {
    LOG(ERROR) << "processNewKeyFrame: ptr_cur_keyframe == nullptr !!! ";
    return false;
  }

  is_last_match_ok_ = is_match_ok_;
  ptr_last_keyframe_ = ptr_cur_keyframe_;
  ptr_cur_keyframe_ = ptr_cur_keyframe;
  cur_frame_pose_ = ptr_cur_keyframe_->getPose();
  is_match_ok_ = true;

  last_imu_ = cur_imu_;
  cur_imu_ = ptr_cur_keyframe->cur_imu_;

  // 每帧点云处理流程：
  // 1. 利用预测的位姿，与地图匹配进行位姿求解；
  // 2. 更新局部地图；

  bool ds_flag = dowmSampleCurFrame();

  if (ds_flag == true) {
    static TicToc scanToLocalMap_cost;
    scanToLocalMap_cost.tic();
    scanToLocalMap();
    LOG(INFO) << "scanToLocalMap_time: " << scanToLocalMap_cost.toc()
              << ", ave_time: " << scanToLocalMap_cost.getAveTime();

    if (is_match_ok_) {
      updateTransform();
    } else {
      LOG(ERROR) << "MapTrack: map match failed ! ";
    }

    ds_curr_surf_in_map_->clear();
    ds_curr_surf_in_map_ =
        Transformbox::transformPointCloud(ptr_cur_keyframe_->getPose(), ds_curr_surf_cloud_);

    static TicToc updateLocalMap_cost;
    updateLocalMap_cost.tic();
    voxMapUpdate();
    LOG(INFO) << "updateLocalMap_time: " << updateLocalMap_cost.toc()
              << ", ave_time: " << updateLocalMap_cost.getAveTime();

    if (is_first_keyframe_) {
      is_first_keyframe_ = false;
      LOG(WARNING) << "MapTrack::processNewKeyFrame: Insert first keyframe !";
    }
  } else {
    is_match_ok_ = false;
    match_failed_count_++;
    LOG(ERROR) << "MapTrack: dowmSampleCurFrame failed ! match_failed_count: "
               << match_failed_count_;
  }

  Mat34d delta_key_pose = Mathbox::deltaPose34d(last_frame_pose_, ptr_cur_keyframe_->getPose());
  float delta_key_dis = delta_key_pose.block<3, 1>(0, 3).norm();
  moved_distance_ += delta_key_dis;
  ptr_cur_keyframe_->moved_distance_ = moved_distance_;

  ptr_keyframe_visu_ = ptr_cur_keyframe_;
  last_frame_odom_pose_ = ptr_cur_keyframe_->getLaserOdomPose();
  last_frame_pose_ = ptr_cur_keyframe_->getPose();

  /////////////////////////////////////////////////////////////////////////

  Vec3d cur_frame_pos = ptr_cur_keyframe_->getPose().block<3, 1>(0, 3);
  Vec3d cur_frame_rpy = Mathbox::rotation2rpy(ptr_cur_keyframe_->getPose().block<3, 3>(0, 0));
  cur_frame_rpy *= Rad2Deg;

  Vec3d cur_Lodom_pos = ptr_cur_keyframe_->getLaserOdomPose().block<3, 1>(0, 3);
  Vec3d cur_Lodom_rpy =
      Mathbox::rotation2rpy(ptr_cur_keyframe_->getLaserOdomPose().block<3, 3>(0, 0));
  cur_Lodom_rpy *= Rad2Deg;

  Vec3d cur_imu_rpy = Mathbox::rotation2rpy(ptr_cur_keyframe_->cur_imu_.Q_wi.toRotationMatrix());
  cur_imu_rpy *= Rad2Deg;

  LOG(WARNING) << "cur_frame_pose: x: " << cur_frame_pos.x() << ", y: " << cur_frame_pos.y()
               << ", z: " << cur_frame_pos.z() << ", roll: " << cur_frame_rpy.x()
               << ", pitch: " << cur_frame_rpy.y() << ", yaw: " << cur_frame_rpy.z()
               << "; imu -- roll: " << cur_imu_rpy.x() << ", pitch: " << cur_imu_rpy.y()
               << ", yaw: " << cur_imu_rpy.z();

  LOG(INFO) << "cur_laser_odom: x: " << cur_Lodom_pos.x() << ", y: " << cur_Lodom_pos.y()
            << ", z: " << cur_Lodom_pos.z() << ", roll: " << cur_Lodom_rpy.x()
            << ", pitch: " << cur_Lodom_rpy.y() << ", yaw: " << cur_Lodom_rpy.z();

  return is_match_ok_;
}

bool MapTrack::dowmSampleCurFrame() {
  float cloud_reso = 1.0 * config_->vox_size;
  static SpaceVoxel<PointType> ds_cloud(cloud_reso, cloud_reso, cloud_reso);

  static TicToc ds_curr_cloud_time;
  ds_curr_cloud_time.tic();

  laserCloud::Ptr cur_cloud(new laserCloud());
  if (config_->keyframe_fusion_size > 1) {
    cur_cloud = ptr_cur_keyframe_->fusion_cloud_;
  } else {
    cur_cloud = ptr_cur_keyframe_->surf_cloud_;
  }

  ds_curr_surf_cloud_->clear();
  ds_cloud.Clear();
  ds_cloud.InsertCloud(*cur_cloud);
  ds_cloud.getVoxelCloud(*ds_curr_surf_cloud_);

  // *ds_curr_surf_cloud_ = *(ptr_cur_keyframe_->surf_cloud_);

  if (ds_curr_surf_cloud_->size() < 8 * config_->min_feature_points) {
    LOG(INFO) << "MapTrack: Too few surf points: " << ds_curr_surf_cloud_->size();
    *ds_curr_surf_cloud_ = *cur_cloud;
  }

  LOG(INFO) << "MapTrack: ds_curr_cloud_time: " << ds_curr_cloud_time.toc()
            << ", ave_time: " << ds_curr_cloud_time.getAveTime()
            << ", curr_surf: " << ds_curr_surf_cloud_->size();

  ds_curr_surf_in_map_->clear();
  ds_curr_surf_in_map_ = Transformbox::transformPointCloud(cur_frame_pose_, ds_curr_surf_cloud_);

  if (ds_curr_surf_in_map_->size() < config_->min_feature_points) {
    LOG(WARNING) << "Too few points: " << ds_curr_surf_in_map_->size();
    return false;
  }

  return true;
}

void MapTrack::scanToLocalMap() {
  float min_success_match_error = config_->min_success_match_error;

  float error_ratio = 3.0;
  if (!config_->use_odom) {
    error_ratio = 100.0;
  }

  if (ds_surf_cloud_map_->size() < config_->min_feature_points) {
    LOG(ERROR) << "Too small map points: " << ds_surf_cloud_map_->size();
    is_match_ok_ = false;
    return;
  }

  int max_iteration = 10;
  max_match_dis_ = 0.5 * config_->max_match_dis;
  min_success_match_error = error_ratio * min_success_match_error;

  for (int iterCount = 0; iterCount < max_iteration; iterCount++) {
    Mat34d last_temp_pose = cur_frame_pose_;

    // 与地图点进行匹配，对位姿进行优化；
    CeresPPICPMatch();

    Mat34d delta_pose = Mathbox::deltaPose34d(last_temp_pose, cur_frame_pose_);
    float delta_rotation = Mathbox::rotation2rpy(delta_pose.block<3, 3>(0, 0)).norm();
    float delta_trans = delta_pose.block<3, 1>(0, 3).norm();
    LOG(INFO) << "MapTrack:iterCount: " << iterCount << "; delta_dis: " << delta_trans
              << ", delta_rot: " << delta_rotation;

    if (delta_rotation < 0.01 && delta_trans < 0.02) {
      if (curr_distance_error_ < min_success_match_error) {
        break;
      } else if (iterCount > 1) {
        is_match_ok_ = false;
        LOG(ERROR) << "MapTrack: big match error: " << curr_distance_error_;
        break;
      }
    } else if ((max_iteration - 1) == iterCount) {
      if (curr_distance_error_ > min_success_match_error) {
        is_match_ok_ = false;
        LOG(ERROR) << "MapTrack: reach max_iteration ! big match distance: "
                   << curr_distance_error_;
      }
    }
  }
  Mat34d delta_odom_pose =
      Mathbox::deltaPose34d(last_frame_odom_pose_, ptr_cur_keyframe_->getLaserOdomPose());

  Mat34d delta_match_pose = Mathbox::deltaPose34d(last_frame_pose_, cur_frame_pose_);

  float d_dis_odom = delta_odom_pose.block<3, 1>(0, 3).norm();
  float d_rot_odom = Mathbox::rotation2rpy(delta_odom_pose.block<3, 3>(0, 0)).norm();

  float d_dis_match = delta_match_pose.block<3, 1>(0, 3).norm();
  float d_rot_match = Mathbox::rotation2rpy(delta_match_pose.block<3, 3>(0, 0)).norm();

  // 通过两个运动增量之间的差值大小来判断是否打滑；
  float predict_dis_ratio = std::abs(d_dis_match - d_dis_odom) / config_->trans_threshold;
  float predict_rot_ratio = std::abs(d_rot_match - d_rot_odom) / config_->angle_threshold;

  predict_error_ratio_ = std::max(predict_dis_ratio, predict_rot_ratio);

  Mat34d delta_pose_accept =
      Mathbox::Interp_SE3(delta_odom_pose, delta_match_pose, config_->odom_match_ratio);

  if (map_odom_update_flag_ == true) {
    last_frame_pose_ = cur_frame_pose_;
    delta_pose_accept = Mathbox::Identity34();
    map_odom_update_flag_ = false;
    LOG(WARNING) << "update last_frame_pose_: \n" << last_frame_pose_;
  }
  cur_frame_pose_ = Mathbox::multiplePose34d(last_frame_pose_, delta_pose_accept);

  Eigen::Quaterniond q_tmp(cur_frame_pose_.block<3, 3>(0, 0));
  q_tmp.normalize();
  cur_frame_pose_.block<3, 3>(0, 0) = q_tmp.toRotationMatrix();

  if (config_->plane_mode == true) {
    Mathbox::ToPlaneMode(cur_frame_pose_);
  }

  if (!config_->use_imu && !config_->use_odom) {
    is_match_ok_ = true;
  }

  // if (predict_error_ratio_ > 0.5 || is_match_ok_ == false)
  if (predict_error_ratio_ > 0.5) {
    LOG(ERROR) << "MapTrack: big gap between Lodom and match !!! ";
    LOG(ERROR) << "predict_error_ratio: " << predict_error_ratio_;
    LOG(ERROR) << "map match distance : " << curr_distance_error_;
    LOG(ERROR) << "d_dis_odom : " << d_dis_odom << ", d_rot_odom: " << d_rot_odom;
    LOG(ERROR) << "d_dis_match: " << d_dis_match << ", d_rot_match: " << d_rot_match;
  } else {
    LOG(INFO) << "MapTrack: map match distance: " << curr_distance_error_;
  }

  if (curr_distance_error_ > min_success_match_error) {
    match_failed_count_++;
    LOG(ERROR) << "MapTrack: match_failed_count: " << match_failed_count_;
  }

  float map_match_failed_dis = 20.0 * config_->min_success_match_error;
  if (curr_distance_error_ > map_match_failed_dis && d_dis_odom > 0.1) {
    localization_failed_count_++;
    LOG(ERROR) << "MapTrack: localization_failed_count: " << localization_failed_count_;
  }

  float min_match_overlap_ratio = config_->min_match_overlap_ratio;
  if (overlap_ratio_ < min_match_overlap_ratio && d_dis_odom > 0.1) {
    low_overlap_count_++;
    LOG(ERROR) << "MapTrack: too low match overlap ! overlap_ratio: " << overlap_ratio_;
    LOG(ERROR) << "MapTrack: low_overlap_count: " << low_overlap_count_;
  }
}

void MapTrack::voxMapUpdate() {
  // 角速度较大时不更新地图；
  if (ptr_cur_keyframe_->angular > config_->angular_threshold && config_->dataset_mode != 1) {
    LOG(WARNING) << "MapTrack::voxMapUpdate - large angular: " << ptr_cur_keyframe_->angular;
    return;
  }

  laserCloud::Ptr cur_cloud;
  if (config_->dataset_mode == 2) {
    cur_cloud = ptr_cur_keyframe_->dense_cloud_;
  } else {
    cur_cloud = ptr_cur_keyframe_->surf_cloud_;
  }

  laserCloud::Ptr cloud_in_Map =
      Transformbox::transformPointCloud(ptr_cur_keyframe_->getPose(), cur_cloud);

  // 将当前帧的点更新到局部地图中；
  ptr_ivox_map_->AddPoints(cloud_in_Map->points, config_->use_prob_match);

  // 获取当前局部地图点云；
  std::unique_lock<std::mutex> lock(curr_surround_map_mutex_);
  ds_surf_cloud_map_->clear();
  ptr_ivox_map_->GetAllPoints(ds_surf_cloud_map_->points);
}

bool MapTrack::CeresPPICPMatch() {
  float new_add_prob = config_->new_add_prob;
  float surf_size = config_->surf_size;
  float max_match_range = config_->max_match_range;

  Eigen::Vector3d cur_t(cur_frame_pose_.block<3, 1>(0, 3));
  Eigen::Quaterniond cur_q(cur_frame_pose_.block<3, 3>(0, 0));
  cur_q.normalize();

  // ceres opt
  ceres::Problem *problem = nullptr;
  ceres::Problem::Options problem_options;
  ceres::LossFunction *huber_loss = nullptr;
  ceres::LossFunction *lidar_huber_loss = nullptr;
  huber_loss = new ceres::HuberLoss(5.0);
  lidar_huber_loss = new ceres::HuberLoss(0.1);
  problem = new ceres::Problem(problem_options);

  double para_q[4] = {cur_q.x(), cur_q.y(), cur_q.z(), cur_q.w()};
  double para_t[3] = {cur_t.x(), cur_t.y(), cur_t.z()};
  ceres::LocalParameterization *q_parameterization = new ceres::EigenQuaternionParameterization();
  problem->AddParameterBlock(para_q, 4, q_parameterization);
  problem->AddParameterBlock(para_t, 3);

  const int search_num = 5;
  static std::vector<int> Ind(search_num);
  static std::vector<float> SqDis(search_num);
  PointType p_w, p_raw;
  PointType PCLPoint[search_num];
  unsigned int valid_points = 0;
  int near_num = 0;
  float error_sum = 0.0;
  float distance = 1.0;
  float p_range = 0.0;
  float dis_weight = 1.0;
  float map_prob_weight = 1.0;
  float curr_prob_weight = 1.0;
  float prob_weight = 1.0;
  float range_weight = 1.0;
  float plane_weight = 1.0;
  float factor_weight = 1.0;
  Eigen::Matrix<float, search_num, 3> A;
  Eigen::Matrix<float, search_num, 1> b;
  Eigen::Vector3f norm_vec;
  Eigen::Vector4f pabcd;
  float norm = 0.0;
  float plane_error = 0.0;
  int un_plane_count = 0;
  std::vector<PointToPlaneFactor> v_match;
  bool is_fit_ok = false;
  float map_prob_ave = 0.0;
  float range_ratio = 0.5;
  static TicToc match_cost;
  match_cost.tic();

  float fit_error_scale = 0.3;
  float plane_error_th = fit_error_scale * config_->vox_size;
  if (plane_error_th < 0.1) {
    plane_error_th = 0.1;
  }

  if (ptr_cur_keyframe_->degenerate_flag_) {
    range_ratio = 0.2;
    is_match_ok_ = false;
    LOG(ERROR) << "cloud degenerate:\n" << ptr_cur_keyframe_->eigen_vec_;
    return false;
  }

  u_int32_t point_num = ds_curr_surf_in_map_->size();
  for (u_int32_t i = 0; i < point_num; i++) {
    v_nearest_points_[i].clear();
    p_w = ds_curr_surf_in_map_->points[i];
    p_raw = ds_curr_surf_cloud_->points[i];
    p_range = p_raw.normal_x;
    if (p_range > max_match_range) {
      continue;
    }

    if (config_->search_method == 1) {
      near_num = ptr_kdtree_surf_map_->nearestKSearch(p_w, search_num, Ind, SqDis);
      for (int j = 0; j < near_num; j++) { SqDis[j] = std::sqrt(SqDis[j]); }
    } else if (config_->search_method == 2) {
      is_fit_ok = ptr_ivox_map_->GetNearPlane(p_w, v_nearest_points_[i], pabcd, plane_error);
      SqDis[search_num - 1] = 0.0;
    } else {
      LOG(ERROR) << "search_method error !";
      continue;
    }

    if (is_fit_ok == true) {
      PCLPoint[0] = v_nearest_points_[i][0];

      distance = pabcd(0) * p_w.x + pabcd(1) * p_w.y + pabcd(2) * p_w.z + pabcd(3);
      distance = std::abs(distance);
      if (distance < 0.00001) {
        LOG(INFO) << "too small distance: " << distance;
        continue;
      }

      plane_weight = plane_error_th / plane_error;
      if (plane_weight > 1.0) {
        plane_weight = 1.0;
      }

      range_weight = range_ratio * max_match_range / p_range;
      if (range_weight > 1.0) {
        range_weight = 1.0;
      }

      if (config_->use_prob_match) {
        map_prob_ave = v_nearest_points_[i][0].intensity;
        prob_weight = 0.75 * map_prob_ave / new_add_prob;
        if (prob_weight > 1.0) {
          prob_weight = 1.0;
        }
      }

      factor_weight = prob_weight * range_weight * plane_weight;

      Vec3f pi(p_raw.x, p_raw.y, p_raw.z);
      PointToPlaneFactor pp_match_i = PointToPlaneFactor(pi, pabcd, factor_weight, distance);

      v_match.emplace_back(pp_match_i);

      bool use_point_factor = false;
      if (use_point_factor) {
        Vec3d p_i(p_raw.x, p_raw.y, p_raw.z);
        Vec3d p_world(p_w.x, p_w.y, p_w.z);
        Vec3d p_map(PCLPoint[0].x, PCLPoint[0].y, PCLPoint[0].z);

        float dis = (p_world - p_map).norm();
        if (dis > 0.001 && dis < 0.5 * surf_size) {
          float dis_weight = 0.2 * surf_size / dis;
          if (dis_weight > 1.0) {
            dis_weight = 1.0;
          }

          float point_weight = 0.2 * range_weight * dis_weight;

          ceres::CostFunction *p_factor = PointFactor::Create(p_i, p_map, point_weight);
          problem->AddResidualBlock(p_factor, lidar_huber_loss, para_q, para_t);
          valid_points++;
        }
      }

      error_sum += distance;
    }
  }

  std::sort(v_match.begin(), v_match.end(),
            [](PointToPlaneFactor a, PointToPlaneFactor b) { return (a.distance_ < b.distance_); });

  float dis_weight_ratio = 1.0;
  u_int32_t match_size = v_match.size();
  u_int32_t keep_num = config_->match_keep_ratio * match_size;
  for (u_int32_t i = 0; i < keep_num; i++) {
    PointToPlaneFactor pp_match_i = v_match[i];
    dis_weight_ratio = 1.0 - 0.4 * i / match_size;
    pp_match_i.scale_weight(dis_weight_ratio);
    ceres::CostFunction *pp_factor_i = pp_match_i.Create();

    problem->AddResidualBlock(pp_factor_i, lidar_huber_loss, para_q, para_t);
    valid_points++;
  }

  LOG(INFO) << "MapTrack::CeresPPICPMatch: valid_points: " << valid_points
            << ", localMap: " << ds_surf_cloud_map_->size();

  LOG(INFO) << "match_time: " << match_cost.toc()
            << ", ave_match_time: " << match_cost.getAveTime();

  Eigen::Vector3d imu_rpy, imu_smooth_rpy, cur_rpy;
  if (config_->use_imu) {
    imu_rpy = Mathbox::rotation2rpy(ptr_cur_keyframe_->cur_imu_.Q_wi.toRotationMatrix());
    imu_rpy *= Rad2Deg;
    imu_smooth_rpy =
        Mathbox::rotation2rpy(ptr_cur_keyframe_->cur_imu_.Q_wi_smooth.toRotationMatrix());
    imu_smooth_rpy *= Rad2Deg;
    cur_rpy = Mathbox::rotation2rpy(cur_frame_pose_.block<3, 3>(0, 0));
    cur_rpy *= Rad2Deg;

    imu_rpy(2) = 0.0;
    imu_smooth_rpy(2) = 0.0;
    cur_rpy(2) = 0.0;
  }

  if (config_->use_imu && config_->use_gravity_constraint) {
    float angle_diff = (imu_rpy - imu_smooth_rpy).norm();
    angle_diff -= 0.2;
    float angle_cov = angle_diff * 2.0;
    if (angle_cov < 1.0) {
      angle_cov = 1.0;
    }
    float angle_weight = 1.0 / (angle_cov * angle_cov * angle_cov);
    float gravity_weight_ratio = config_->gravity_weight_ratio;
    float gravity_weight = gravity_weight_ratio * angle_weight;

    ceres::CostFunction *gravity_factor =
        GravityFactor::Create(imu_smooth_rpy(0), imu_smooth_rpy(1), gravity_weight);
    problem->AddResidualBlock(gravity_factor, huber_loss, para_q);

    ptr_cur_keyframe_->cur_imu_.gravity_weight = angle_weight;
  }

  if (config_->use_imu && config_->use_local_ground_constraint) {
    float ground_weight_ratio = config_->ground_weight_ratio;

    float angle_diff = (imu_rpy - imu_smooth_rpy).norm();
    angle_diff -= 0.2;
    float angle_cov = angle_diff * 2.0;
    if (angle_cov < 1.0) {
      angle_cov = 1.0;
    }
    float angle_weight = 1.0 / (angle_cov * angle_cov * angle_cov);

    // float angle_norm = imu_rpy.norm();
    float angle_norm = imu_smooth_rpy.norm();
    angle_norm -= 0.2;
    if (angle_norm < 1.0) {
      angle_norm = 1.0;
    }
    float horizon_ratio = 40.0 / (angle_norm * angle_norm * angle_norm);

    float ground_weight = ground_weight_ratio * horizon_ratio * angle_weight;

    ceres::CostFunction *ground_factor =
        LocalGroundFactor::Create(ptr_last_keyframe_->getPose()(2, 3), ground_weight);
    problem->AddResidualBlock(ground_factor, huber_loss, para_t);
  }

  if (config_->use_imu && config_->use_gyr_constraint) {
    float weight_yaw = 1.0 * config_->gyr_yaw_weight_ratio;
    float weight_pitch = 1.0 * config_->gyr_pitch_weight_ratio;
    float weight_roll = 1.0 * config_->gyr_roll_weight_ratio;

    if (std::abs(ptr_last_keyframe_->time_stamp_ - ptr_cur_keyframe_->time_stamp_) < 5.0) {
      Eigen::Quaterniond q_last(ptr_last_keyframe_->getPose().block<3, 3>(0, 0));
      q_last.normalize();
      Eigen::Quaterniond dq = last_imu_.Q_wi_gyr.conjugate() * cur_imu_.Q_wi_gyr;
      dq.normalize();
      ceres::CostFunction *d_rot_factor =
          DeltaRFactor::Create(q_last, dq, weight_yaw, weight_pitch, weight_roll);
      problem->AddResidualBlock(d_rot_factor, huber_loss, para_q);
    }
  }

  if (config_->use_global_ground_constraint) {
    float horizon_ratio = 0.6;
    float ground_weight = config_->ground_weight_ratio * horizon_ratio;

    ceres::CostFunction *ground_factor = LocalGroundFactor::Create(0.0, ground_weight);
    problem->AddResidualBlock(ground_factor, huber_loss, para_t);

    float gravity_weight = config_->gravity_weight_ratio * horizon_ratio;

    ceres::CostFunction *gravity_factor = GravityFactor::Create(0.0, 0.0, gravity_weight);
    problem->AddResidualBlock(gravity_factor, huber_loss, para_q);
  }

  if (config_->use_odom && ptr_cur_keyframe_->degenerate_flag_) {
    Vec3d t_last = ptr_last_keyframe_->getLaserOdomPose().block<3, 1>(0, 3);
    Vec3d t_last_odom = ptr_last_keyframe_->getWheelPose().block<3, 1>(0, 3);
    Vec3d t_curr_odom = ptr_cur_keyframe_->getWheelPose().block<3, 1>(0, 3);
    float d_norm_odom = (t_curr_odom - t_last_odom).norm();

    ceres::CostFunction *norm_factor = DeltaDisFactor::Create(t_last, d_norm_odom, 5.0);
    problem->AddResidualBlock(norm_factor, huber_loss, para_t);

    Eigen::Quaterniond q_pre(ptr_cur_keyframe_->getLaserOdomPose().block<3, 3>(0, 0));
    q_pre.normalize();
    Vec3d t_pre = ptr_cur_keyframe_->getLaserOdomPose().block<3, 1>(0, 3);

    ceres::CostFunction *rt_factor = RTFactor::Create(q_pre, t_pre, 1.0);
    problem->AddResidualBlock(rt_factor, huber_loss, para_q, para_t);
  }

  if (valid_points < point_num / 2) {
    LOG(ERROR) << "MapTrack::CeresPPICPMatch: Too few valid_points: " << valid_points;
  } else {
    static TicToc opt_cost;
    opt_cost.tic();
    ceres::Solver::Options options;
    options.linear_solver_type = ceres::DENSE_QR;
    options.max_num_iterations = 4;
    options.minimizer_progress_to_stdout = false;
    options.check_gradients = false;
    options.gradient_check_relative_precision = 1e-6;
    ceres::Solver::Summary summary;
    ceres::Solve(options, problem, &summary);

    LOG(INFO) << "ceres_opt_time: " << opt_cost.toc()
              << ", ave_opt_time: " << opt_cost.getAveTime();
    // LOG(INFO) << summary.FullReport();
  }

  cur_q.x() = para_q[0];
  cur_q.y() = para_q[1];
  cur_q.z() = para_q[2];
  cur_q.w() = para_q[3];
  cur_q.normalize();
  cur_t.x() = para_t[0];
  cur_t.y() = para_t[1];
  cur_t.z() = para_t[2];

  cur_frame_pose_.block<3, 3>(0, 0) = cur_q.toRotationMatrix();
  cur_frame_pose_.block<3, 1>(0, 3) = cur_t;

  // Transform the input cloud using the final transformation
  ds_curr_surf_in_map_->clear();
  ds_curr_surf_in_map_ = Transformbox::transformPointCloud(cur_frame_pose_, ds_curr_surf_cloud_);

  size_t inlier_num = 0;
  float match_dis_i = 0.0;
  float match_dis_sum = 0.0;
  float match_dis_ave = 0.99;
  float max_match_dis = 12.0 * surf_size;
  float inlier_dis = 1.5 * surf_size;
  float zero_dis = 0.5 * surf_size;
  PointType ps, pt;
  size_t cloud_size = ds_curr_surf_in_map_->points.size();

  if (config_->search_method == 2) {
    for (size_t i = 0; i < cloud_size; ++i) {
      p_raw = ds_curr_surf_cloud_->points[i];
      if (p_raw.normal_x > max_match_range) {
        continue;
      }
      if (v_nearest_points_[i].size() == 0) {
        continue;
      }
      ps = ds_curr_surf_in_map_->points[i];
      pt = v_nearest_points_[i][0];
      match_dis_i = std::sqrt((ps.x - pt.x) * (ps.x - pt.x) + (ps.y - pt.y) * (ps.y - pt.y) +
                              (ps.z - pt.z) * (ps.z - pt.z));
      if (match_dis_i < max_match_dis) {
        if (match_dis_i < zero_dis) {
          match_dis_i = 0.0;
        }
        match_dis_sum += match_dis_i;
        if (match_dis_i < inlier_dis) {
          inlier_num++;
        }
      }
    }
  } else {
    for (size_t i = 0; i < cloud_size; ++i) {
      p_w = ds_curr_surf_in_map_->points[i];
      p_raw = ds_curr_surf_cloud_->points[i];
      if (p_raw.normal_x > max_match_range) {
        continue;
      }
      if (ptr_kdtree_surf_map_->nearestKSearch(p_w, 1, Ind, SqDis) > 0) {
        match_dis_i = std::sqrt(SqDis[0]);
        if (match_dis_i < max_match_dis) {
          if (match_dis_i < zero_dis) {
            match_dis_i = 0.0;
          }
          match_dis_sum += match_dis_i;
          if (match_dis_i < inlier_dis) {
            inlier_num++;
          }
        }
      }
    }
  }

  if (inlier_num > 10) {
    match_dis_ave = match_dis_sum / inlier_num;
  }

  overlap_ratio_ = 1.0 * inlier_num / cloud_size;
  if (overlap_ratio_ < 0.1) {
    overlap_ratio_ = 0.1;
  }

  float overlap_trans = 1.2 * overlap_ratio_;
  float overlap_score = overlap_trans * overlap_trans * overlap_trans;

  curr_distance_error_ = match_dis_ave / overlap_score;

  LOG(INFO) << "MapTrack::CeresPPICPMatch: match_dis_ave: " << match_dis_ave
            << ", overlap_ratio: " << overlap_ratio_ << ", curr_distance: " << curr_distance_error_;

  delete problem;

  return true;
}

pcl::PointCloud<PointType>::Ptr MapTrack::getCurrSurroundMap() {
  pcl::PointCloud<PointType>::Ptr surroundMapCloud(new pcl::PointCloud<PointType>());
  std::lock_guard<std::mutex> lock(curr_surround_map_mutex_);
  *surroundMapCloud = *ds_surf_cloud_map_;

  return surroundMapCloud;
}

Mat34d MapTrack::getMap2Odom() {
  std::lock_guard<std::mutex> lock(correct_pose_mutex_);
  return map_to_odom_;
}

void MapTrack::setMap2Odom(const Mat34d &map_to_odom, bool is_update_map) {
  std::lock_guard<std::mutex> lock(correct_pose_mutex_);
  map_to_odom_ = map_to_odom;
  last_map_to_odom_ = map_to_odom;
  map_odom_update_flag_ = true;
  // if (ptr_cur_keyframe_ != nullptr && is_update_map) {
  //   last_frame_pose_ = ptr_cur_keyframe_->getPose();
  //   LOG(INFO) << "setMap2Odom last_frame_pose_:" << last_frame_pose_;
  // }
}

bool MapTrack::isKeyFrame(bool use_rotation) {
  static Mat34d _last_keyframe_pose = Mathbox::Identity34();
  static Mat34d cur_frame_pose = Mathbox::Identity34();
  static Mat34d delta_pose = Mathbox::Identity34();
  static float delta_distance = 0.0, delta_angle = 0.0;
  float key_dis_threshold = 0.9 * config_->trans_threshold;
  float key_rot_threshold = 0.9 * config_->angle_threshold;

  if (config_->dataset_mode == 1) {
    return true;
  }
  if (is_first_keyframe_) {
    return true;
  }

  cur_frame_pose = ptr_cur_keyframe_->getPose();

  delta_pose = Mathbox::deltaPose34d(_last_keyframe_pose, cur_frame_pose);
  delta_distance = delta_pose.block<3, 1>(0, 3).norm();
  delta_angle = Mathbox::rotation2rpy(delta_pose.block<3, 3>(0, 0)).norm();

  if (delta_distance > key_dis_threshold || delta_angle > key_rot_threshold) {
    _last_keyframe_pose = cur_frame_pose;

    return true;
  } else {
    return false;
  }
}

void MapTrack::updateTransform() {
  ptr_cur_keyframe_->updatePose(cur_frame_pose_);

  // 利用地图匹配优化后的位姿和里程计位姿来计算最新的矫正值；
  map_to_odom_ = Mathbox::multiplePose34d(
      ptr_cur_keyframe_->getPose(), Mathbox::inversePose34d(ptr_cur_keyframe_->getLaserOdomPose()));

  Eigen::Quaterniond q_tmp(map_to_odom_.block<3, 3>(0, 0));
  q_tmp.normalize();
  map_to_odom_.block<3, 3>(0, 0) = q_tmp.toRotationMatrix();

  if (config_->plane_mode == true) {
    Mathbox::ToPlaneMode(map_to_odom_);
  }
  last_map_to_odom_ = map_to_odom_;
}

laserCloud::Ptr MapTrack::getSurfMapCloud() {
  laserCloud::Ptr cloud(new laserCloud());
  *cloud = *ds_surf_cloud_map_;
  return cloud;
}

laserCloud::Ptr MapTrack::getCurrSurfCloud() {
  laserCloud::Ptr curr_surf(new laserCloud());
  if (ptr_keyframe_visu_ != nullptr) {
    *curr_surf = *(ptr_keyframe_visu_->surf_cloud_);
  }
  return curr_surf;
}

float MapTrack::getGroundHeight(const laserCloud::Ptr &map_cloud) {
  float cloud_reso = config_->vox_size;
  float height_reso = 0.5 * cloud_reso;
  static int height_size = (config_->occ_max_height - config_->occ_min_height) / height_reso + 1;

  static SpaceVoxel<PointType> ds_cloud(cloud_reso, cloud_reso, cloud_reso);
  laserCloud::Ptr map_cloud_ds(new laserCloud());
  ds_cloud.Clear();
  ds_cloud.InsertCloud(*map_cloud);
  ds_cloud.getVoxelCloud(*map_cloud_ds);

  float cur_z = ptr_cur_keyframe_->getPose()(2, 3);
  float ph = 0.0;

  // 根据高度分辨率，计算点的高度区间，对每个高度区间内的点进行统计；
  PointType p;
  int height_id = 0;
  int point_num[height_size] = {0};
  for (u_int32_t i = 0; i < map_cloud_ds->size(); i++) {
    p = map_cloud_ds->points[i];
    ph = p.z - cur_z;
    if (ph < config_->occ_min_height || ph > config_->occ_max_height) {
      continue;
    }
    height_id = (ph - config_->occ_min_height) / height_reso;
    point_num[height_id]++;
  }

  // 计算平均点数；
  int ave_num = 0;
  int height_count = 0;
  for (int i = 0; i < height_size; i++) {
    if (point_num[i] > 10) {
      ave_num += point_num[i];
      height_count++;
    }
  }
  if (height_count > 0) {
    ave_num /= height_count;
  } else {
    ave_num = 100000;
  }

  int ground_th = 0.8 * ave_num;

  // 计算点数符合要求的最低高度，即地面高度；因为地面是最低的，通过点数可以滤除非地面噪声；
  float cur_height = 999.9;
  for (int i = 0; i < height_size; i++) {
    if (point_num[i] < ground_th) {
      continue;
    } else {
      cur_height = height_reso * i + config_->occ_min_height + cur_z;
      break;
    }
  }

  // 根据地面高度提取点云；计算平均地面高度；
  float min_height = cur_height - 1.5 * cloud_reso;
  float max_height = cur_height + 1.5 * cloud_reso;
  float ground_height = 0.0;
  height_count = 0;
  // laserCloud::Ptr ground_cloud(new laserCloud());
  for (u_int32_t i = 0; i < map_cloud_ds->size(); i++) {
    p = map_cloud_ds->points[i];
    if (p.z > min_height && p.z < max_height) {
      ground_height += p.z;
      height_count++;
      // ground_cloud->points.emplace_back(p);
    }
  }
  if (height_count > 0) {
    ground_height /= height_count;
  } else {
    ground_height = 999.9;
  }

  LOG(INFO) << "ave_num: " << ave_num << ", ground_points: " << height_count
            << ", ground_height: " << ground_height;

  return ground_height;
}

}  // namespace GR_SLAM
