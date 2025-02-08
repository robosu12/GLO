/************************************************************************
 *@version V1.0
 ************************************************************************/
#ifndef BASE_FEATURE_EXTRACTOR_HPP
#define BASE_FEATURE_EXTRACTOR_HPP

#include <vector>
#include <mutex>

#include "common/data_struct/pc_base.hpp"
#include "common/config/system_config.hpp"
#include "common/math_base/slam_transform.hpp"
#include "common/data_struct/sensors_type.hpp"

namespace GR_SLAM {
/**
 * BaseFeatureExtractor
 * @brief 点云特征提取类
 * 1. 点云前处理
 * 2. 提取所需的特征点云
 **/
class BaseFeatureExtractor {
 public:
  BaseFeatureExtractor() = default;
  virtual ~BaseFeatureExtractor() = default;

  /**
   *setInputCloud
   *@brief
   *获取输入点云数据
   *
   *@param[in] cloud_in-输入点云
   **/

  virtual bool setInputCloud(const laserCloud::Ptr ptr_cloud) = 0;

  /**
   *resetVariables
   *@brief
   *重置变量
   *
   **/
  virtual void resetVariables() = 0;

  /**
   *adjustDistortion
   *@brief
   *将点云按匀速模型插补进行畸变矫正
   *
   **/
  virtual void adjustDistortion(const Mat34d &delta_pose) = 0;

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
                              Vec3f &eigen_vec) = 0;

  virtual void setCurrIMU(const ImuMeasure &cur_imu) = 0;

  virtual laserCloud::Ptr getDenseRawCloud() = 0;

 protected:
  std::mutex mutex_;  ///< 数据锁>

};  // end of class

}  // namespace GR_SLAM

#endif  // BASE_FEATURE_EXTRACTOR_HPP