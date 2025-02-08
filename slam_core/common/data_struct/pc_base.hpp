/************************************************************************
 * Software License Agreement (BSD License)
 ************************************************************************/
#ifndef PC_BASE_HPP_
#define PC_BASE_HPP_

#include <pcl/common/common.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/io/pcd_io.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/io/io.h>
#include <pcl/common/transforms.h>
#include <pcl/kdtree/kdtree_flann.h>
#include <pcl/registration/registration.h>
#include <pcl/registration/icp.h>

namespace GR_SLAM {
// struct PointXYZIRT
// {
//   PCL_ADD_POINT4D;
//   float intensity;
//   uint16_t ring = 0;
//   double timestamp = 0;
//   EIGEN_MAKE_ALIGNED_OPERATOR_NEW
// } EIGEN_ALIGN16;
// POINT_CLOUD_REGISTER_POINT_STRUCT(PointXYZIRT, (float, x, x)(float, y,
// y)(float, z, z)(uint8_t, intensity, intensity)(uint16_t, ring, ring)(double,
// timestamp, timestamp))

struct RsPointXYZIRT {
  PCL_ADD_POINT4D;
  uint8_t intensity;
  uint16_t ring = 0;
  double timestamp = 0;
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW
} EIGEN_ALIGN16;

typedef pcl::PointXYZI XYZIPointType;
typedef pcl::PointXYZINormal PointType;

typedef pcl::PointXYZINormal FramePosition;
typedef pcl::PointCloud<PointType> laserCloud;
typedef pcl::KdTreeFLANN<PointType> pclKdTree;
typedef pcl::VoxelGrid<PointType> pclDownsampler;

typedef pcl::PointXYZRGBNormal PointRGBType;
typedef pcl::PointCloud<PointRGBType> RGBCloud;
typedef pcl::VoxelGrid<PointRGBType> pclRGBDownsampler;

typedef pcl::PointXYZ depthPointType;
typedef pcl::PointCloud<depthPointType> depthCloud;

using PointVector = std::vector<PointType, Eigen::aligned_allocator<PointType>>;

}  // namespace GR_SLAM

POINT_CLOUD_REGISTER_POINT_STRUCT(GR_SLAM::RsPointXYZIRT,
                                  (float, x, x)(float, y, y)(float, z, z)(uint8_t, intensity,
                                                                          intensity)(
                                      uint16_t, ring, ring)(double, timestamp, timestamp))

#endif  // PC_BASE_HPP_
