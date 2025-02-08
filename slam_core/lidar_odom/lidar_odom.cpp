/************************************************************************
 * Software License Agreement (BSD License)
 *@author Yun Su(robosu12@gmail.com)
 *@version 0.8
 *@data 2022-04-27
 ************************************************************************/

#include "lidar_odom/lidar_odom.hpp"
#include <glog/logging.h>
#include "common/pose_graph_error_term.hpp"
#include "common/math_base/slam_transform.hpp"
#include "common/data_struct/keyframe.hpp"
#include "common/config/system_config.hpp"
#include "common/debug_tools/tic_toc.h"

namespace GR_SLAM {
LidarOdom::LidarOdom(const std::shared_ptr<SystemConfig> config) : config_(config) {
  InitParameters();
}

LidarOdom::~LidarOdom() {}

void LidarOdom::InitParameters() {
  float resolution_ratio = 1.0;  // 1.5
  float surf_size = config_->surf_size * resolution_ratio;
  downsize_filter_map_surf_.setLeafSize(surf_size, surf_size, surf_size);

  float key_pose_size = config_->trans_threshold * 4.0;
  downsize_filter_key_pose_.setLeafSize(key_pose_size, key_pose_size, key_pose_size);
  Reset();
}

void LidarOdom::Reset() {
  match_failed_count_ = 0;

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
  is_match_ok_ = true;
  is_last_match_ok_ = true;
  is_match_stable_ = true;
  is_wheel_stable_ = true;

  d_recent_lidar_cloud_in_map_.clear();

  ptr_cur_keyframe_ = std::make_shared<KeyFrame>();
  ptr_last_keyframe_ = std::make_shared<KeyFrame>();

  ptr_kdtree_surf_map_.reset(new pclKdTree());
  ptr_kdtree_key_pose_.reset(new pclKdTree());

  surf_cloud_map_.reset(new laserCloud());
  ds_surf_cloud_map_.reset(new laserCloud());
  ds_curr_surf_cloud_.reset(new laserCloud());
  key_pose_cloud_.reset(new laserCloud());

  ds_curr_surf_in_map_.reset(new laserCloud());

  ivox_options_.resolution_ = config_->vox_size_odom;
  ivox_options_.capacity_ = config_->surround_map_size_odom;
  ivox_options_.near_num_ = config_->near_search_num_odom;
  ivox_options_.nearby_type_ = config_->nearby_type_odom;
  ivox_options_.use_hierachy_vox_ = config_->use_hierachy_vox;
  ivox_options_.high_ratio_ = config_->high_ratio;
  ivox_options_.init_prob_ = config_->new_add_prob;
  ivox_options_.max_range_ = config_->laser_max_range;

  if (ptr_ivox_map_ != nullptr) {
    ptr_ivox_map_->Clear();
  }
  ptr_ivox_map_ = std::make_shared<IVoxType>(ivox_options_);

  v_nearest_points_.resize(config_->surround_map_size_odom);

  d_recent_keyframe_.clear();

  LOG(WARNING) << "LidarOdom::Reset !!!";
}

bool LidarOdom::processNewKeyFrame(std::shared_ptr<KeyFrame> ptr_cur_keyframe, bool is_update_map) {
  if (nullptr == ptr_cur_keyframe) {
    LOG(ERROR) << "LidarOdom: ptr_cur_keyframe is nullptr !!! ";
    return false;
  }

  is_last_match_ok_ = is_match_ok_;
  ptr_last_keyframe_ = ptr_cur_keyframe_;
  ptr_cur_keyframe_ = ptr_cur_keyframe;
  cur_frame_pose_ = ptr_cur_keyframe_->getLaserOdomPose();
  is_match_ok_ = true;

  last_imu_ = cur_imu_;
  cur_imu_ = ptr_cur_keyframe->cur_imu_;

  bool new_keyFrame_for_mapTrack = false;

  bool ds_flag = dowmSampleCurFrame();

  static TicToc scanToLocalMap_cost;
  scanToLocalMap_cost.tic();
  scanToLocalMap();
  LOG(INFO) << "scanToLocalMap_time: " << scanToLocalMap_cost.toc()
            << ", ave_time: " << scanToLocalMap_cost.getAveTime();

  if (is_match_ok_) {
    // 用匹配优化后的位姿对里程计结果进行更新；
    ptr_cur_keyframe_->updateLaserOdomPose(cur_frame_pose_);

    map_to_odom_ = Mathbox::multiplePose34d(
        cur_frame_pose_, Mathbox::inversePose34d(ptr_cur_keyframe_->getWheelPose()));

    Eigen::Quaterniond q_tmp(map_to_odom_.block<3, 3>(0, 0));
    q_tmp.normalize();
    map_to_odom_.block<3, 3>(0, 0) = q_tmp.toRotationMatrix();

    if (config_->plane_mode == true) {
      Mathbox::ToPlaneMode(map_to_odom_);
    }

    last_map_to_odom_ = map_to_odom_;
  } else {
    ds_curr_surf_in_map_->clear();
    ds_curr_surf_in_map_ = Transformbox::transformPointCloud(ptr_cur_keyframe_->getLaserOdomPose(),
                                                             ds_curr_surf_cloud_);
    match_failed_count_++;
    LOG(ERROR) << "lidar_odom match failed ! count: " << match_failed_count_;
  }

  last_frame_odom_pose_ = ptr_cur_keyframe_->getWheelPose();
  last_frame_pose_ = ptr_cur_keyframe_->getLaserOdomPose();

  bool match_stable = true;
  if (is_last_match_ok_ == true && is_match_ok_ == false) {
    match_stable = false;
    LOG(ERROR) << "LidarOdom: match unstable !!!";
  }
  if (config_->use_imu || config_->use_odom) {
    if (curr_distance_error_ > 5.0 * config_->min_success_match_error) {
      match_stable = false;
      LOG(ERROR) << "LidarOdom: match unstable: " << curr_distance_error_;
    } else {
      match_stable = true;
    }
  } else {
    match_stable = true;
  }

  if ((match_stable && isKeyFrame()) || ptr_cur_keyframe_->index_ < 5) {
    static TicToc updateLocalMap_cost;
    updateLocalMap_cost.tic();

    voxMapUpdate();

    LOG(INFO) << "updateLocalMap_time: " << updateLocalMap_cost.toc()
              << ", ave_time: " << updateLocalMap_cost.getAveTime();
  }

  if (isKeyFrameForMapTrack()) {
    // calculate keyframe probability
    if (config_->use_prob_match) {
      ptr_keyframe_maptrack_ = calculateKeyframeProbabilty();
    } else {
      ptr_keyframe_maptrack_ = ptr_cur_keyframe_;
    }

    // // undistortion; only support our lidar driver;
    // if (config_->lidar_undistortion == 3) {
    //   Mat34d delta_pose = Mathbox::deltaPose34d(ptr_last_keyframe_->getLaserOdomPose(),
    //                                             ptr_cur_keyframe_->getLaserOdomPose());
    //   adjustDistortion(delta_pose, ptr_cur_keyframe_->surf_cloud_);
    // }

    // recent keyframe fusion
    if (config_->keyframe_fusion_size > 1) {
      TicToc frameFusion_cost;
      LocalKeyFrameFusion(is_update_map);
      LOG(INFO) << "keyframe fusion time: " << frameFusion_cost.toc();
    }

    new_keyFrame_for_mapTrack = true;
  }

  if (is_first_keyframe_) {
    is_first_keyframe_ = false;
    LOG(WARNING) << "LidarOdom: Insert first keyframe !!!";
  }

  Vec3d cur_Lodom_pos = ptr_cur_keyframe_->getLaserOdomPose().block<3, 1>(0, 3);
  Vec3d cur_Lodom_rpy =
      Mathbox::rotation2rpy(ptr_cur_keyframe_->getLaserOdomPose().block<3, 3>(0, 0));
  cur_Lodom_rpy *= Rad2Deg;
  Vec3d cur_wheel_pos = ptr_cur_keyframe_->getWheelPose().block<3, 1>(0, 3);
  Vec3d cur_wheel_rpy = Mathbox::rotation2rpy(ptr_cur_keyframe_->getWheelPose().block<3, 3>(0, 0));
  cur_wheel_rpy *= Rad2Deg;

  LOG(INFO) << "cur_laser_odom: x: " << cur_Lodom_pos.x() << ", y: " << cur_Lodom_pos.y()
            << ", z: " << cur_Lodom_pos.z() << ", roll: " << cur_Lodom_rpy.x()
            << ", pitch: " << cur_Lodom_rpy.y() << ", yaw: " << cur_Lodom_rpy.z();
  LOG(INFO) << "cur_wheel_odom: x: " << cur_wheel_pos.x() << ", y: " << cur_wheel_pos.y()
            << ", z: " << cur_wheel_pos.z() << ", roll: " << cur_wheel_rpy.x()
            << ", pitch: " << cur_wheel_rpy.y() << ", yaw: " << cur_wheel_rpy.z();

  return new_keyFrame_for_mapTrack;
}

bool LidarOdom::dowmSampleCurFrame() {
  float cloud_reso = 1.0 * config_->vox_size_odom;
  static SpaceVoxel<PointType> ds_cloud(cloud_reso, cloud_reso, cloud_reso);

  static TicToc ds_curr_cloud_time;
  ds_curr_cloud_time.tic();

  ds_curr_surf_cloud_->clear();
  ds_cloud.Clear();
  ds_cloud.InsertCloud(*(ptr_cur_keyframe_->surf_cloud_));
  ds_cloud.getVoxelCloud(*ds_curr_surf_cloud_);

  // *ds_curr_surf_cloud_ = *(ptr_cur_keyframe_->surf_cloud_);

  if (ds_curr_surf_cloud_->size() < 8 * config_->min_feature_points) {
    LOG(INFO) << "LidarOdom: Too few surf points: " << ds_curr_surf_cloud_->size();
    *ds_curr_surf_cloud_ = *(ptr_cur_keyframe_->surf_cloud_);
  }

  LOG(INFO) << "LidarOdom: ds_curr_cloud_time: " << ds_curr_cloud_time.toc()
            << ", ave_time: " << ds_curr_cloud_time.getAveTime()
            << ", curr_surf: " << ds_curr_surf_cloud_->size();

  ds_curr_surf_in_map_->clear();
  ds_curr_surf_in_map_ = Transformbox::transformPointCloud(cur_frame_pose_, ds_curr_surf_cloud_);

  if (ds_curr_surf_in_map_->size() < config_->min_feature_points) {
    LOG(ERROR) << "Too few lidar points:  " << ds_curr_surf_in_map_->size();
    return false;
  }

  return true;
}

void LidarOdom::scanToLocalMap() {
  float match_dis_ratio = 3.0;
  if (!config_->use_odom) {
    match_dis_ratio = 100.0;
  }
  float max_map_match_distance = config_->min_success_match_error * match_dis_ratio;

  if (ds_curr_surf_cloud_->size() < config_->min_feature_points) {
    LOG(ERROR) << "too small cur points: " << ds_curr_surf_cloud_->size();
    is_match_ok_ = false;
    return;
  }

  if (is_first_keyframe_) {
    is_match_ok_ = false;
    return;
  }

  int max_iteration = 10;
  max_match_dis_ = 0.5 * config_->max_match_dis;

  for (int iterCount = 0; iterCount < max_iteration; iterCount++) {
    Mat34d last_temp_pose = cur_frame_pose_;

    // 与地图点进行匹配，对位姿进行优化；
    CeresPPICPMatch();

    Mat34d delta_pose = Mathbox::deltaPose34d(last_temp_pose, cur_frame_pose_);
    float delta_rotation = Mathbox::rotation2rpy(delta_pose.block<3, 3>(0, 0)).norm();
    float delta_trans = delta_pose.block<3, 1>(0, 3).norm();
    LOG(INFO) << "LidarOdom: iterCount: " << iterCount << "; delta_dis: " << delta_trans
              << ", delta_rot: " << delta_rotation;

    if (delta_rotation < 0.01 && delta_trans < 0.02) {
      if (curr_distance_error_ < max_map_match_distance) {
        break;
      } else if (iterCount > 1) {
        is_match_ok_ = false;
        LOG(ERROR) << "LidarOdom: big match distance: " << curr_distance_error_;
        break;
      }
    } else if ((max_iteration - 1) == iterCount) {
      if (curr_distance_error_ > max_map_match_distance) {
        is_match_ok_ = false;
        LOG(ERROR) << "LidarOdom: reach max_iteration ! big match distance: "
                   << curr_distance_error_;
      }
    }
  }

  Mat34d delta_odom_pose =
      Mathbox::deltaPose34d(last_frame_odom_pose_, ptr_cur_keyframe_->getWheelPose());
  Mat34d delta_match_pose = Mathbox::deltaPose34d(last_frame_pose_, cur_frame_pose_);

  float d_dis_odom = delta_odom_pose.block<3, 1>(0, 3).norm();
  float d_rot_odom = Mathbox::rotation2rpy(delta_odom_pose.block<3, 3>(0, 0)).norm();

  float d_dis_match = delta_match_pose.block<3, 1>(0, 3).norm();
  float d_rot_match = Mathbox::rotation2rpy(delta_match_pose.block<3, 3>(0, 0)).norm();

  float predict_dis_ratio = std::abs(d_dis_match - d_dis_odom) / (config_->trans_threshold * 1.0);
  float predict_rot_ratio = std::abs(d_rot_match - d_rot_odom) / (config_->angle_threshold * 1.0);

  is_match_stable_ = true;
  is_wheel_stable_ = true;
  if (predict_dis_ratio > 0.5) {
    if (d_dis_match > d_dis_odom) {
      is_match_stable_ = false;
    } else {
      is_wheel_stable_ = false;
    }
  }
  if (predict_rot_ratio > 0.5) {
    if (d_rot_match > d_rot_odom) {
      is_match_stable_ = false;
    } else {
      is_wheel_stable_ = false;
    }
  }

  if (curr_distance_error_ < max_map_match_distance) {
    is_match_stable_ = true;
  }
  if (!config_->use_imu && !config_->use_odom) {
    is_match_stable_ = true;
    is_match_ok_ = true;
  }

  Mat34d delta_pose_accept =
      Mathbox::Interp_SE3(delta_odom_pose, delta_match_pose, config_->odom_match_ratio);
  // if (config_->calculate_degenerate == 2 &&
  //     ptr_cur_keyframe_->eigen_vec_(1) < config_->degenerate_threshold) {
  //   is_match_ok_ = false;
  //   LOG(ERROR) << "cloud degenerate:\n" << ptr_cur_keyframe_->eigen_vec_;
  // }

  cur_frame_pose_ = Mathbox::multiplePose34d(last_frame_pose_, delta_pose_accept);

  Eigen::Quaterniond q_tmp(cur_frame_pose_.block<3, 3>(0, 0));
  q_tmp.normalize();
  cur_frame_pose_.block<3, 3>(0, 0) = q_tmp.toRotationMatrix();

  if (config_->plane_mode == true) {
    Mathbox::ToPlaneMode(cur_frame_pose_);
  }

  if (is_match_stable_ == false) {
    static int count = 0;
    is_match_ok_ = false;
    LOG(ERROR) << "LidarOdom: big gap between odom and match "
                  "!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!! count:  "
               << count++;
    LOG(ERROR) << "map match distance : " << curr_distance_error_;
    LOG(ERROR) << "predict_dis_ratio: " << predict_dis_ratio
               << ", predict_rot_ratio: " << predict_rot_ratio;
    LOG(ERROR) << "d_dis_odom : " << d_dis_odom << ", d_rot_odom: " << d_rot_odom;
    LOG(ERROR) << "d_dis_match: " << d_dis_match << ", d_rot_match: " << d_rot_match;
  } else {
    LOG(INFO) << "LidarOdom: map match distance: " << curr_distance_error_;
  }

  if (is_wheel_stable_ == false) {
    static int count = 0;
    LOG(ERROR) << "LidarOdom: wheel slipping "
                  "!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!! count: "
               << count++;
    LOG(ERROR) << "map match distance : " << curr_distance_error_;
    LOG(ERROR) << "predict_dis_ratio: " << predict_dis_ratio
               << ", predict_rot_ratio: " << predict_rot_ratio;
    LOG(ERROR) << "d_dis_odom : " << d_dis_odom << ", d_rot_odom: " << d_rot_odom;
    LOG(ERROR) << "d_dis_match: " << d_dis_match << ", d_rot_match: " << d_rot_match;
  }
}

void LidarOdom::voxMapUpdate() {
  laserCloud::Ptr cloud_in_Map = Transformbox::transformPointCloud(
      ptr_cur_keyframe_->getLaserOdomPose(), ptr_cur_keyframe_->surf_cloud_);

  // 将当前帧的点更新到局部地图中；
  ptr_ivox_map_->AddPoints(cloud_in_Map->points, config_->use_prob_match);

  // 获取当前局部地图点云；
  // ds_surf_cloud_map_->clear();
  // ptr_ivox_map_->GetAllPoints(ds_surf_cloud_map_->points);
}

bool LidarOdom::CeresPPICPMatch() {
  float surf_size = config_->surf_size_odom;
  float laser_max_range = config_->laser_max_range;
  float max_match_range = config_->max_match_range;
  float new_add_prob = config_->new_add_prob;

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
  PointType p_w, p_raw, p_near;
  PointType PCLPoint[search_num];
  Vec3d near_map_point[search_num];
  int valid_points = 0;
  int near_num = 0;
  float error_sum = 0.0;
  float distance = 1.0;
  float p_range = 0.0;
  float dis_weight = 1.0;
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
  float plane_error_ave = 0.0;
  std::vector<PointToPlaneFactor> v_match;
  bool is_fit_ok = false;
  float map_prob_ave = 0.0;
  float range_ratio = 0.5;

  static TicToc match_cost;
  match_cost.tic();

  float fit_error_scale = 0.3;
  float plane_error_th = fit_error_scale * config_->vox_size_odom;
  if (plane_error_th < 0.1) {
    plane_error_th = 0.1;
  }
  int up_plane_count = 0;

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
      // 根据近邻模型，返回的点可以近似看成从近到远排列；
      is_fit_ok = ptr_ivox_map_->GetNearPlane(p_w, v_nearest_points_[i], pabcd, plane_error);
      SqDis[search_num - 1] = 0.0;
    } else {
      LOG(ERROR) << "search method error ! ";
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

      if (plane_error > plane_error_th) {
        up_plane_count++;
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
      // ptr_cur_keyframe_->surf_cloud_->points[i].intensity +=
      //     factor_weight * 10.0;

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

      // if (ptr_cur_keyframe_->degenerate_flag_) {
      //   Vec3d p_i(p_raw.x, p_raw.y, p_raw.z);
      //   Vec3d p_map(PCLPoint[0].x, PCLPoint[0].y, PCLPoint[0].z);
      //   ceres::CostFunction *p_factor =
      //       PointFactor::Create(p_i, p_map, 0.5 * range_weight);
      //   problem->AddResidualBlock(p_factor, lidar_huber_loss, para_q,
      //   para_t); valid_points++;
      // }
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

  // plane_error_ave /= match_size;
  // LOG(WARNING) << "LidarOdom::CeresPPICPMatch: un_plane_count: "
  //              << un_plane_count << ", plane_error_ave: " << plane_error_ave;

  static int opt_count = 0;
  static float recall_sum = 0.0, recall_ave = 0.0;
  recall_sum += 1.0 * match_size / point_num;
  opt_count++;
  recall_ave = recall_sum / opt_count;

  static float up_plane_ratio = 0.0, up_plane_ave = 0.0;
  up_plane_ratio += 1.0 * up_plane_count / point_num;
  up_plane_ave = up_plane_ratio / opt_count;

  LOG(INFO) << "LidarOdom::CeresPPICPMatch: valid_points: " << valid_points
            << ", recall_ave: " << recall_ave << ", localMap: " << ds_surf_cloud_map_->size()
            << ", up_plane_count: " << up_plane_count << ", up_plane_ave: " << up_plane_ave;

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
        LocalGroundFactor::Create(ptr_last_keyframe_->getLaserOdomPose()(2, 3), ground_weight);
    problem->AddResidualBlock(ground_factor, huber_loss, para_t);
  }

  if (config_->use_imu && config_->use_gyr_constraint) {
    float weight_yaw = 1.0 * config_->gyr_yaw_weight_ratio;
    float weight_pitch = 1.0 * config_->gyr_pitch_weight_ratio;
    float weight_roll = 1.0 * config_->gyr_roll_weight_ratio;

    if (std::abs(ptr_last_keyframe_->time_stamp_ - ptr_cur_keyframe_->time_stamp_) < 5.0) {
      Eigen::Quaterniond q_last(ptr_last_keyframe_->getLaserOdomPose().block<3, 3>(0, 0));
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

  if (valid_points < config_->min_feature_points / 2) {
    LOG(ERROR) << "LidarOdom::CeresPPICPMatch: Too small valid_points: " << valid_points;
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

  if (config_->search_method == 1) {
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
  } else if (config_->search_method == 2) {
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
  }

  if (inlier_num > 10) {
    match_dis_ave = match_dis_sum / inlier_num;
  }

  float overlap_ratio = 1.0 * inlier_num / cloud_size;
  if (overlap_ratio < 0.1) {
    overlap_ratio = 0.1;
  }

  float overlap_trans = 1.2 * overlap_ratio;
  float overlap_score = overlap_trans * overlap_trans * overlap_trans;

  curr_distance_error_ = match_dis_ave / overlap_score;

  LOG(INFO) << "LidarOdom::CeresPPICPMatch: match_dis_ave: " << match_dis_ave
            << ", overlap_ratio: " << overlap_ratio << ", curr_distance: " << curr_distance_error_;

  delete problem;

  return true;
}

pcl::PointCloud<PointType>::Ptr LidarOdom::getCurrSurroundMap() {
  pcl::PointCloud<PointType>::Ptr surroundMapCloud(new pcl::PointCloud<PointType>());
  std::lock_guard<std::mutex> lock(curr_surround_map_mutex_);
  // 获取当前局部地图点云；
  ds_surf_cloud_map_->clear();
  ptr_ivox_map_->GetAllPoints(ds_surf_cloud_map_->points);
  *surroundMapCloud = *ds_surf_cloud_map_;

  return surroundMapCloud;
}

Mat34d LidarOdom::getMap2Odom() {
  std::lock_guard<std::mutex> lock(correct_pose_mutex_);
  return map_to_odom_;
}

void LidarOdom::setMap2Odom(const Mat34d &map_to_odom) {
  std::lock_guard<std::mutex> lock(correct_pose_mutex_);
  map_to_odom_ = map_to_odom;
  last_map_to_odom_ = map_to_odom;
}

bool LidarOdom::isKeyFrame(bool use_rotation) {
  static Mat34d _last_keyframe_pose = Mathbox::Identity34();
  static Mat34d _last_frame_pose = Mathbox::Identity34();
  static Mat34d cur_frame_pose = Mathbox::Identity34();
  static Mat34d delta_pose = Mathbox::Identity34();
  static float delta_distance = 0.0, delta_angle = 0.0;
  float key_dis_threshold = 1.0 * config_->trans_threshold;
  float key_rot_threshold = 1.0 * config_->angle_threshold;

  cur_frame_pose = ptr_cur_keyframe_->getLaserOdomPose();

  delta_pose = Mathbox::deltaPose34d(_last_frame_pose, cur_frame_pose);
  delta_distance = delta_pose.block<3, 1>(0, 3).norm();
  delta_angle = Mathbox::rotation2rpy(delta_pose.block<3, 3>(0, 0)).norm();
  _last_frame_pose = cur_frame_pose;

  // 仅使用激光时为连续帧匹配，帧间旋转等价于角速度；(10hz)
  if (config_->use_lidar && !config_->use_imu && !config_->use_odom) {
    ptr_cur_keyframe_->angular = delta_angle * 10.0;
  }

  if (config_->dataset_mode == 1) {
    return true;
  }
  if (is_first_keyframe_) {
    return true;
  }

  if (ptr_cur_keyframe_->angular > config_->angular_threshold) {
    LOG(WARNING) << "large rotation angular: " << ptr_cur_keyframe_->angular;
    return false;
  }

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

bool LidarOdom::isKeyFrameForMapTrack() {
  static Mat34d _last_keyframe_pose = Mathbox::Identity34();
  static Mat34d _last_stable_pose = Mathbox::Identity34();
  static Mat34d cur_frame_pose = Mathbox::Identity34();
  static Mat34d delta_pose = Mathbox::Identity34();
  static float delta_distance = 0.0, delta_angle = 0.0;
  float key_dis_threshold = 1.0 * config_->trans_threshold;
  float key_rot_threshold = 1.0 * config_->angle_threshold;

  if (config_->dataset_mode == 1) {
    return true;
  }

  if (is_first_keyframe_) {
    return true;
  }

  cur_frame_pose = ptr_cur_keyframe_->getLaserOdomPose();

  // stable key frame
  if (ptr_cur_keyframe_->angular < config_->angular_threshold) {
    delta_pose = Mathbox::deltaPose34d(_last_stable_pose, cur_frame_pose);
    delta_distance = delta_pose.block<3, 1>(0, 3).norm();
    delta_angle = Mathbox::rotation2rpy(delta_pose.block<3, 3>(0, 0)).norm();

    if (delta_distance > key_dis_threshold || delta_angle > key_rot_threshold) {
      _last_stable_pose = cur_frame_pose;
      _last_keyframe_pose = cur_frame_pose;
      return true;
    }
  }

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

bool LidarOdom::isJunkFrame(const double time, const Mat34d &cur_odom_pose) {
  static double _last_frame_time = time;
  float angle_threshold = 1.1 * config_->angle_threshold;
  float trans_threshold = 1.1 * config_->trans_threshold;
  float time_threshold = 1.0 * config_->time_threshold;
  float imu_time_threshold = 1.0 * config_->imu_time_threshold;
  if (config_->lidar_type == "2D") {
    angle_threshold *= 0.5;
    trans_threshold *= 0.5;
  }

  if (!config_->use_odom) {
    if (!config_->use_imu) {
      return false;
    } else {
      float delta_time = std::abs(time - _last_frame_time);
      if (delta_time > imu_time_threshold) {
        _last_frame_time = time;
        return false;
      } else {
        return true;
      }
    }
  }

  Mat34d delta_odom_pose = Mathbox::deltaPose34d(last_keyframe_odom_pose_, cur_odom_pose);
  float delta_angle = Mathbox::rotation2rpy(delta_odom_pose.block<3, 3>(0, 0)).norm();

  float delta_trans = delta_odom_pose.block<3, 1>(0, 3).norm();
  float delta_time = std::abs(time - _last_frame_time);
  float time_interval_limit = 2.0;
  float time_ratio_limit = 0.2;
  if (delta_time < time_interval_limit) {
    delta_time = time_interval_limit;
  }
  float time_ratio = time_interval_limit / delta_time;
  if (time_ratio < time_ratio_limit) {
    time_ratio = time_ratio_limit;
  }

  angle_threshold = angle_threshold * time_ratio;
  trans_threshold = trans_threshold * time_ratio;

  if (delta_angle > angle_threshold || delta_trans > trans_threshold ||
      delta_time > time_threshold) {
    last_keyframe_odom_pose_ = cur_odom_pose;
    _last_frame_time = time;
    return false;
  }

  return true;
}

std::shared_ptr<KeyFrame> LidarOdom::calculateKeyframeProbabilty() {
  static u_int32_t keep_size = 5;
  static std::deque<std::shared_ptr<KeyFrame>> _d_keyframe;

  if (config_->dynamic_cloud_remove) {
    keep_size = 5;
  } else {
    keep_size = 0;
  }

  if (is_first_keyframe_) {
    _d_keyframe.clear();
  }

  _d_keyframe.emplace_back(ptr_cur_keyframe_);
  std::shared_ptr<KeyFrame> ptr_pre_keyframe = _d_keyframe.front();
  if (_d_keyframe.size() > keep_size) {
    _d_keyframe.pop_front();
  }

  Mat34d frame_pose = ptr_pre_keyframe->getLaserOdomPose();
  laserCloud::Ptr cloud_in_map =
      Transformbox::transformPointCloud(frame_pose, ptr_pre_keyframe->surf_cloud_);

  float erase_prob = config_->new_add_prob - 1;
  int near_num = 0;
  int search_num = 1;
  PointType p_w;

  for (u_int32_t i = 0; i < ptr_pre_keyframe->surf_cloud_->size(); i++) {
    v_nearest_points_[i].clear();
    p_w = cloud_in_map->points[i];
    near_num = ptr_ivox_map_->GetNearPoints(p_w, v_nearest_points_[i], search_num, max_match_dis_);
    if (near_num == search_num) {
      ptr_pre_keyframe->surf_cloud_->points[i].intensity = v_nearest_points_[i][0].intensity;

      // // 可视化要删除的点云
      // if (ptr_pre_keyframe->surf_cloud_->points[i].intensity < erase_prob) {
      //   ptr_pre_keyframe->surf_cloud_->points[i].intensity = erase_prob - 1;
      // } else {
      //   ptr_pre_keyframe->surf_cloud_->points[i].intensity = erase_prob + 1;
      // }
    }
  }

  if (config_->dynamic_cloud_remove) {
    for (u_int32_t i = 0; i < ptr_pre_keyframe->surf_cloud_->size(); i++) {
      if (ptr_pre_keyframe->surf_cloud_->points[i].intensity < erase_prob) {
        ptr_pre_keyframe->surf_cloud_->erase(ptr_pre_keyframe->surf_cloud_->begin() + i);
        i--;
      }
    }
  }

  return ptr_pre_keyframe;
}

void LidarOdom::LocalKeyFrameFusion(const bool &map_match_flag) {
  if (is_match_ok_ == false || map_match_flag == false) {
    d_recent_keyframe_.clear();
  }

  d_recent_keyframe_.push_back(ptr_keyframe_maptrack_);
  if (d_recent_keyframe_.size() > config_->keyframe_fusion_size) {
    d_recent_keyframe_.pop_front();
  }

  ptr_keyframe_maptrack_->fusion_cloud_->clear();

  if (d_recent_keyframe_.size() < 2) {
    *(ptr_keyframe_maptrack_->fusion_cloud_) = *(ptr_keyframe_maptrack_->surf_cloud_);
    LOG(INFO) << "not enough recent_keyframe: " << d_recent_keyframe_.size();
    return;
  }

  Mat34d cur_pose = d_recent_keyframe_.back()->getLaserOdomPose();

  laserCloud::Ptr local_cloud(new laserCloud());
  laserCloud::Ptr ds_local_cloud(new laserCloud());

  for (u_int32_t i = 0; i < d_recent_keyframe_.size(); i++) {
    Mat34d delta_pose = Mathbox::deltaPose34d(cur_pose, d_recent_keyframe_[i]->getLaserOdomPose());

    laserCloud::Ptr cloud_i =
        Transformbox::transformPointCloud(delta_pose, d_recent_keyframe_[i]->surf_cloud_);

    *local_cloud += *cloud_i;
  }

  static SpaceVoxel<PointType> ds_cloud(config_->surf_size, config_->surf_size, config_->surf_size);
  ds_cloud.Clear();
  ds_cloud.InsertCloud(*local_cloud);
  ds_cloud.getVoxelCloud(*ds_local_cloud);

  *(ptr_keyframe_maptrack_->fusion_cloud_) = *ds_local_cloud;

  LOG(INFO) << "keyframe_fusion_cloud size: " << ds_local_cloud->size();
}

void LidarOdom::adjustDistortion(const Mat34d &delta_pose, laserCloud::Ptr cloud) {
  // LOG(WARNING) << "Undistort dis: " << delta_pose.block<3, 1>(0, 3).transpose()
  //              << ", rpy: " << Mathbox::rotation2rpy(delta_pose.block<3, 3>(0, 0)).transpose();

  PointType pi;
  // 小数部分为每个激光点相对于第一个点的测量时间；
  float time_span = 0.1;  // 10hz
  float inv_time_span = 1.0 / time_span;
  float ratio;
  Mat34d dT_0i;
  u_int32_t size = cloud->size();
  for (u_int32_t i = 0; i < size; ++i) {
    pi = cloud->points[i];
    ratio = (pi.intensity - (int) pi.intensity) * inv_time_span;
    dT_0i = Mathbox::Interp_SE3(Mathbox::Identity34(), delta_pose, ratio);
    Transformbox::pointAssociateToMap(&cloud->points[i], &pi, dT_0i);
    cloud->points[i] = pi;
  }
}

}  // namespace GR_SLAM
