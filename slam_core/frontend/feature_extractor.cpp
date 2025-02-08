#include "frontend/feature_extractor.hpp"
#include "common/data_struct/keyframe.hpp"
#include <glog/logging.h>
#include "common/debug_tools/tic_toc.h"

namespace GR_SLAM {
FeatureExtractor::FeatureExtractor() {
  config_ = std::make_shared<SystemConfig>();

  nan_point_.x = std::numeric_limits<float>::quiet_NaN();
  nan_point_.y = std::numeric_limits<float>::quiet_NaN();
  nan_point_.z = std::numeric_limits<float>::quiet_NaN();
  nan_point_.intensity = 0;

  allocateMemory();
  resetVariables();
}

FeatureExtractor::FeatureExtractor(const std::shared_ptr<SystemConfig> config) {
  config_ = config;

  nan_point_.x = std::numeric_limits<float>::quiet_NaN();
  nan_point_.y = std::numeric_limits<float>::quiet_NaN();
  nan_point_.z = std::numeric_limits<float>::quiet_NaN();
  nan_point_.intensity = 0;

  allocateMemory();
  resetVariables();
}

void FeatureExtractor::allocateMemory() {
  laser_cloud_raw_.reset(new laserCloud());
  laser_cloud_in_.reset(new laserCloud());
  laser_cloud_ds_.reset(new laserCloud());
  segmented_cloud_.reset(new laserCloud());
  outlier_cloud_.reset(new laserCloud());
  occ_cloud_ds_.reset(new laserCloud());

  corner_points_.reset(new laserCloud());
  surf_points_.reset(new laserCloud());
  raw_cloud_.reset(new laserCloud());
  occ_cloud_.reset(new laserCloud());

  ptr_kdtree_raw_cloud_.reset(new pclKdTree());

  downSize_filter_surf_.setLeafSize(config_->surf_size, config_->surf_size, config_->surf_size);

  downSize_narrow_cloud_.setLeafSize(config_->surf_size * 0.5, config_->surf_size * 0.5,
                                     config_->surf_size * 0.5);
  downSize_dense_cloud_.setLeafSize(config_->dense_map_reso, config_->dense_map_reso,
                                    config_->dense_map_reso);
  cloud_label_.resize(30000, 0);

  float surf_size = config_->surf_size;
  ptr_cloud_ds_surf_ = std::make_shared<SpaceVoxel<PointType>>(surf_size, surf_size, surf_size);

  p0_.x = 0.0;
  p0_.y = 0.0;
  p0_.z = 0.0;
}

void FeatureExtractor::resetVariables() {
  laser_cloud_raw_->clear();
  laser_cloud_in_->clear();
  laser_cloud_ds_->clear();
  segmented_cloud_->clear();
  outlier_cloud_->clear();
  occ_cloud_ds_->clear();

  corner_points_->clear();
  surf_points_->clear();
  raw_cloud_->clear();
  occ_cloud_->clear();

  segmented_range_.clear();

  label_count_ = 1;
}

bool FeatureExtractor::setInputCloud(const laserCloud::Ptr ptr_cloud) {
  if (ptr_cloud->points.size() < 10) {
    LOG(ERROR) << "too small cloud size: " << ptr_cloud->points.size();
    return false;
  }

  laser_cloud_raw_->clear();
  *laser_cloud_raw_ = *ptr_cloud;

  return true;
}

void FeatureExtractor::adjustDistortion(const Mat34d &delta_pose) {
  // LOG(WARNING) << "Undistort dis: " << delta_pose.block<3, 1>(0, 3).transpose()
  //              << ", rpy: " << Mathbox::rotation2rpy(delta_pose.block<3, 3>(0, 0)).transpose();

  PointType pi;
  // 小数部分为每个激光点相对于第一个点的测量时间；
  float time_span = 0.1;  // 10hz
  float inv_time_span = 1.0 / time_span;
  float ratio;
  Mat34d dT_0i;
  u_int32_t num = laser_cloud_raw_->size();
  for (u_int32_t i = 0; i < num; ++i) {
    pi = laser_cloud_raw_->points[i];
    ratio = (pi.intensity - (int) pi.intensity) * inv_time_span;
    dT_0i = Mathbox::Interp_SE3(Mathbox::Identity34(), delta_pose, ratio);
    Transformbox::pointAssociateToMap(&laser_cloud_raw_->points[i], &pi, dT_0i);
    laser_cloud_raw_->points[i] = pi;
  }
}

bool FeatureExtractor::cloudPreProcess() {
  float laser_min_range = config_->laser_min_range;
  float laser_max_range = config_->laser_max_range;
  float laser_min_height = config_->laser_min_height;
  float laser_max_height = config_->laser_max_height;

  laser_cloud_in_->clear();
  occ_cloud_->clear();
  cur_min_height_ = 999.9;
  cur_max_height_ = -999.9;

  float range = 0.0;
  PointType p_t;
  size_t cloudSize = laser_cloud_raw_->size();
  for (size_t i = 0; i < cloudSize; i++) {
    p_t = laser_cloud_raw_->points[i];

    if (std::isnan(p_t.x) || std::isnan(p_t.y) || std::isnan(p_t.z) || std::isnan(p_t.intensity)) {
      continue;
    }

    range = std::sqrt(p_t.x * p_t.x + p_t.y * p_t.y + p_t.z * p_t.z);
    if (range > laser_max_range || range < laser_min_range || p_t.z > laser_max_height ||
        p_t.z < laser_min_height) {
      continue;
    }

    if (config_->near_back_remove == 1) {
      // 去除后方行人:激光雷达朝前安装
      if (std::abs(p_t.y) < 5.0 && std::abs(p_t.z) < 5.0 && p_t.x < 0.0) {
        continue;
      }
    } else if (config_->near_back_remove == 2) {
      // 去除后方行人：激光雷达朝右安装
      if (std::abs(p_t.x) < 1.0 && p_t.y < 0.0 && p_t.y > -5.0 && p_t.z < 2.0) {
        continue;
      }
    }

    p_t.normal_x = range;
    laser_cloud_in_->points.emplace_back(p_t);
  }

  if (laser_cloud_in_->size() < config_->min_feature_points) {
    LOG(ERROR) << "too less laser_cloud_in_: " << laser_cloud_in_->size();
    return false;
  }

  return true;
}

void FeatureExtractor::clusterSegmentation3D() {
  float small_cluster_radius = config_->small_cluster_radius;
  int small_cluster_num = config_->small_cluster_num;
  float mid_range = 0.5 * config_->laser_max_range;

  // TicToc kdtree_cost;
  ptr_kdtree_raw_cloud_->setInputCloud(laser_cloud_ds_);
  // LOG(INFO) << "kdtree_cost: " << kdtree_cost.toc();

  segmented_cloud_->clear();
  std::fill(cloud_label_.begin(), cloud_label_.end(), 0);

  // TicToc search_cost;
  static std::vector<int> Ind(200);
  static std::vector<float> SqDis(200);
  PointType p;
  float range;
  float range_ratio;
  float search_radius = small_cluster_radius;
  int near_num = 0;
  int small_count = 0;
  u_int32_t size = laser_cloud_ds_->size();
  for (u_int32_t i = 0; i < size; i++) {
    if (cloud_label_[i] == 1) {
      continue;
    }
    p = laser_cloud_ds_->points[i];
    // range = std::sqrt(p.x * p.x + p.y * p.y);
    // range_ratio = range / mid_range;
    // if (range_ratio < 1.0) {
    //   range_ratio = 1.0;
    // }
    // search_radius = range_ratio * small_cluster_radius;
    near_num = ptr_kdtree_raw_cloud_->radiusSearch(p, search_radius, Ind, SqDis);
    if (near_num > small_cluster_num) {
      for (int j = 0; j < near_num; j++) {
        if (cloud_label_[Ind[j]] != 1) {
          segmented_cloud_->emplace_back(laser_cloud_ds_->points[Ind[j]]);
        }
        cloud_label_[Ind[j]] = 1;
      }
    } else {
      cloud_label_[i] = 0;
      small_count++;
    }
  }
  // LOG(INFO) << "search_cost: " << search_cost.toc();
  LOG(INFO) << "laser_cloud_ds: " << size << " , small_size: " << small_count;
}

laserCloud::Ptr FeatureExtractor::occCloudFilter(const laserCloud::Ptr occ_cloud) {
  laserCloud::Ptr slice_cloud(new laserCloud());

  float reso = config_->cloud_map_2d_reso;
  int height_size = (cur_max_height_ - cur_min_height_) / reso + 1;
  if (height_size < 3) {
    return slice_cloud;
  }

  // 根据高度分辨率，生成离散高度区间；
  int statistic[height_size - 1] = {0};
  float v_height[height_size];
  for (int i = 0; i < height_size; i++) { v_height[i] = cur_min_height_ + reso * i; }

  // 对每个高度区间内的点进行统计；
  PointType p;
  laserCloud::Ptr accu_cloud(new laserCloud());
  for (u_int32_t i = 0; i < occ_cloud->size(); i++) {
    p = occ_cloud->points[i];
    for (int j = 0; j < height_size - 1; j++) {
      if (p.z > v_height[j] && p.z < v_height[j + 1]) {
        statistic[j]++;
      }
    }
  }

  // 计算平均点数；
  int ave_num = 0;
  for (int i = 0; i < height_size - 1; i++) { ave_num += statistic[i]; }
  ave_num /= height_size - 1;
  int outlier_th = 0.8 * ave_num;

  // 计算点数符合要求的最低高度，即地面高度；因为地面是最低的，通过点数可以滤除非地面噪声；
  for (int i = 0; i < height_size - 1; i++) {
    if (statistic[i] < outlier_th) {
      continue;
    } else {
      cur_ground_height_ = v_height[i];
      break;
    }
  }

  if (cur_ground_height_ > config_->occ_max_height - 0.2) {
    return slice_cloud;
  }

  LOG(INFO) << "cur_min_height: " << cur_min_height_ << ",cur_max_height: " << cur_max_height_
            << ",cur_ground_height: " << cur_ground_height_;

  // 根据地面高度提取切片点云；
  float min_height = cur_ground_height_ + 0.2;
  float max_height = cur_ground_height_ + 0.4;
  for (u_int32_t i = 0; i < occ_cloud->size(); i++) {
    p = occ_cloud->points[i];
    if (p.z > min_height && p.z < max_height) {
      p.z = 0.0;
      slice_cloud->points.emplace_back(p);
    }
  }
  return slice_cloud;
}

bool FeatureExtractor::cloudExtractor(laserCloud::Ptr surf_cloud, laserCloud::Ptr occ_cloud,
                                      Vec3f &eigen_vec) {
  // static TicToc pre_cost;
  // pre_cost.tic();
  cloudPreProcess();

  // static TicToc ds_cost;
  // ds_cost.tic();
  // laser_cloud_ds_->clear();
  // downSize_filter_surf_.setInputCloud(laser_cloud_in_);
  // downSize_filter_surf_.filter(*laser_cloud_ds_);

  ptr_cloud_ds_surf_->Clear();
  ptr_cloud_ds_surf_->InsertCloud(*laser_cloud_in_);
  ptr_cloud_ds_surf_->getVoxelCloud(*laser_cloud_ds_);

  if (config_->seg_method == 1) {
    clusterSegmentation3D();

    *surf_cloud = *segmented_cloud_;
  } else {
    *surf_cloud = *laser_cloud_ds_;
  }

  if (surf_cloud->size() < 8 * config_->min_feature_points) {
    LOG(ERROR) << "too few surf points, use raw cloud: " << surf_cloud->size();
    *surf_cloud = *laser_cloud_in_;
  }

  if (surf_cloud->size() < 10) {
    LOG(ERROR) << "too few surf points: " << surf_cloud->size()
               << ", raw: " << laser_cloud_in_->size();
    surf_cloud->points.push_back(p0_);
    // return false;
  }

  if (config_->calculate_degenerate == 2) {
    calculateCloudEigenVec(occ_cloud, eigen_vec);
    LOG(INFO) << "cloud eigen_vec: " << eigen_vec.transpose();
  }

  return true;
}

void FeatureExtractor::calculateCloudEigenVec(const laserCloud::Ptr input_cloud, Vec3f &eigen_vec) {
  float near_radius = 5.0 * config_->surf_size;

  pclKdTree::Ptr ptr_kdtree_corner;
  ptr_kdtree_corner.reset(new pclKdTree());
  ptr_kdtree_corner->setInputCloud(input_cloud);

  u_int32_t point_size = input_cloud->size();
  Eigen::Matrix3f cov_mat = Eigen::Matrix3f::Zero();
  Eigen::Vector3f cov_vec;

  std::vector<int> Ind;
  std::vector<float> SqDis;
  for (u_int32_t i = 0; i < point_size; ++i) {
    PointType pointOri = input_cloud->points[i];
    if (ptr_kdtree_corner->nearestKSearch(pointOri, 5, Ind, SqDis) > 4) {
      SqDis[4] = std::sqrt(SqDis[4]);
      if (SqDis[4] < near_radius) {
        Eigen::Vector3f center(pointOri.x, pointOri.y, pointOri.z);
        for (int j = 0; j < 5; j++) {
          Eigen::Vector3f p_j(input_cloud->points[Ind[j]].x, input_cloud->points[Ind[j]].y,
                              input_cloud->points[Ind[j]].z);
          cov_vec = p_j - center;
          cov_mat = cov_mat + cov_vec * cov_vec.transpose();
        }
      }
    }
  }

  // Eigen library sort eigenvalues in increasing order
  Eigen::SelfAdjointEigenSolver<Eigen::Matrix3f> saes(cov_mat);
  eigen_vec = saes.eigenvalues();
}

laserCloud::Ptr FeatureExtractor::getDenseRawCloud() {
  laserCloud::Ptr dense_cloud(new laserCloud());
  // static SpaceVoxel<PointType> ds_cloud(0.1, 0.1, 0.1);
  // ds_cloud.Clear();
  // ds_cloud.InsertCloud(*laser_cloud_in_);
  // ds_cloud.getVoxelCloud(*dense_cloud);

  *dense_cloud = *laser_cloud_in_;
  return dense_cloud;
}
}  // namespace GR_SLAM