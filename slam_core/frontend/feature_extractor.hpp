/************************************************************************
 *@file feature_extractor.hpp
 *
 *@brief
 * 1.前端FeatureExtractor类
 *
 *@author Yun Su(robosu12@gmail.com)
 *@version 1.0
 *@data 2024-08-05
 ************************************************************************/
#ifndef FEATURE_EXTRACTOR_HPP
#define FEATURE_EXTRACTOR_HPP

#include "frontend/base_feature_extractor.hpp"
#include "common/data_struct/space_voxel.hpp"

namespace GR_SLAM {
class KeyFrame;

/**
 * CloudInfo
 * @brief 存储点云信息
 *
 **/
struct CloudInfo {
  std::vector<int> start_ring_index;
  std::vector<int> end_ring_index;

  float start_orientation;
  float end_orientation;
  float orientation_diff;

  std::vector<bool> segmented_cloud_ground_flag;      ///< true - ground point,
                                                      ///< false - other points
  std::vector<unsigned int> segmented_cloud_col_ind;  ///< point column index in range image
  std::vector<float> segmented_cloud_range;           ///< point range
};

struct smoothness_t {
  float value;
  size_t ind;
};

struct by_value {
  bool operator()(smoothness_t const &left, smoothness_t const &right) {
    return left.value < right.value;
  }
};

/**
 * FeatureExtractor
 * @brief 点云特征提取类
 * 1. 点云前处理
 * 2. 提取所需的特征点云
 **/
class FeatureExtractor final : public BaseFeatureExtractor {
 public:
  FeatureExtractor();
  FeatureExtractor(const std::shared_ptr<SystemConfig> config);
  virtual ~FeatureExtractor() {}

  /**
   *setInputCloud
   *@brief
   *获取输入点云数据
   *
   *@param[in] cloud_in-输入点云
   **/

  virtual bool setInputCloud(const laserCloud::Ptr ptr_cloud);

  /**
   *resetVariables
   *@brief
   *重置变量
   *
   **/
  virtual void resetVariables();

  /**
   *cloudExtractor
   *@brief
   *提取所需的特征点云
   *
   *@param[out] corner_cloud-角点
   *@param[out] surf_cloud-面点
   *@param[out] occ_cloud-移除地面点的点云
   **/
  virtual bool cloudExtractor(laserCloud::Ptr surf_cloud, laserCloud::Ptr occ_cloud,
                              Vec3f &eigen_vec);

  virtual void adjustDistortion(const Mat34d &delta_pose);

 private:
  /**
   *allocateMemory
   *@brief
   *提前分配内存
   *
   * */
  virtual void allocateMemory();

  bool cloudPreProcess();
  laserCloud::Ptr occCloudFilter(const laserCloud::Ptr occ_cloud);
  void clusterSegmentation3D();
  void calculateCloudEigenVec(const laserCloud::Ptr input_cloud, Vec3f &eigen_vec);

 public:
  virtual void setCurrIMU(const ImuMeasure &cur_imu) { cur_imu_ = cur_imu; }

  virtual laserCloud::Ptr getDenseRawCloud();

 private:
  laserCloud::Ptr laser_cloud_raw_;
  laserCloud::Ptr laser_cloud_in_;      ///< 输入点云
  laserCloud::Ptr laser_cloud_ds_;      ///< 输入点云
  laserCloud::Ptr segmented_cloud_;     ///< 分割好的点云，包括地面点
  laserCloud::Ptr outlier_cloud_;       ///< 异常点
  laserCloud::Ptr occ_cloud_ds_;        ///< 移除地面点的点云
  std::vector<float> segmented_range_;  ///< 曲率

  PointType nan_point_;  ///< fill in fullCloud at each iteration

  std::shared_ptr<SystemConfig> config_ = nullptr;

  int label_count_;  ///< 点云标签计数

  std::vector<float> cloud_curvature_;       ///< 曲率
  std::vector<float> cloud_curvature_norm_;  ///< 曲率
  std::vector<int> cloud_neighbor_picked_;   ///< 选择的邻域
  std::vector<char> cloud_label_;
  std::vector<smoothness_t> cloud_smoothness_;

  laserCloud::Ptr corner_points_;  ///< 角点
  laserCloud::Ptr surf_points_;    ///< 面点
  laserCloud::Ptr raw_cloud_;      ///< 面点
  laserCloud::Ptr occ_cloud_;      ///< 面点

  float range_array_[3600];
  char bad_point_marked_[3600];

  pcl::VoxelGrid<PointType> downSize_filter_surf_;   ///< 下采样滤波器
  pcl::VoxelGrid<PointType> downSize_filter_occp_;   ///< 下采样滤波器
  pcl::VoxelGrid<PointType> downSize_narrow_cloud_;  ///< 下采样滤波器
  pcl::VoxelGrid<PointType> downSize_dense_cloud_;

  pclKdTree::Ptr ptr_kdtree_raw_cloud_;

  std::shared_ptr<SpaceVoxel<PointType>> ptr_cloud_ds_surf_;
  std::shared_ptr<SpaceVoxel<PointType>> ptr_cloud_ds_occp_;

  ImuMeasure cur_imu_;

  float cur_min_height_ = 999.9;
  float cur_max_height_ = -999.9;
  float cur_ground_height_ = 999.9;

  PointType p0_;

};  // end of class

}  // namespace GR_SLAM

#endif