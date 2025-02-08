/************************************************************************
 * Software License Agreement (BSD License)
 ************************************************************************/
#ifndef SLAM_TRANSFORM_HPP_
#define SLAM_TRANSFORM_HPP_

#include "slam_math.hpp"
#include "common/data_struct/pc_base.hpp"
#include <glog/logging.h>

namespace GR_SLAM {
/**
 * Transformbox
 * @brief 点云坐标变换类
 *
 **/
class Transformbox {
 public:
  /**
   *pointToStart
   *@brief
   *将点变换到起始坐标系
   *
   *@param[in] pi-输入点
   *@param[in] pose-变换pose
   *@param[out] po-输出点
   **/
  // static void pointToStart(PointType const *const pi, PointType *const po,
  //                          const Mat34d &pose) {
  //   // float s = 10 * (pi->intensity - int(pi->intensity));
  //   po->intensity = pi->intensity;
  // }

  /**
   *pointToEnd
   *@brief
   *将点变换到结束坐标系
   *
   *@param[in] pi-输入点
   *@param[in] pose-变换pose
   *@param[out] po-输出点
   **/
  // static void pointToEnd(PointType const *const pi, PointType *const po,
  //                        const Mat34d &pose) {
  //   // float s = 10 * (pi->intensity - int(pi->intensity));
  //   po->intensity = int(pi->intensity);
  // }

  /**
   *pointAssociateToMap
   *@brief
   *将点变换到地图坐标系
   *
   *@param[in] pi-输入点
   *@param[in] pose-变换pose
   *@param[out] po-输出点
   **/
  static void pointAssociateToMap(PointType const *const pi, PointType *const po,
                                  const Mat34d &pose) {
    *po = *pi;
    Vec3d point_curr(pi->x, pi->y, pi->z);
    Vec3d point_w = Mathbox::multiplePoint(pose, point_curr);
    po->x = point_w.x();
    po->y = point_w.y();
    po->z = point_w.z();
  }

  static void pointXYZAssociateToMap(depthPointType const *const pi, depthPointType *const po,
                                     const Mat34d &pose) {
    Vec3d point_curr(pi->x, pi->y, pi->z);
    Vec3d point_w = Mathbox::multiplePoint(pose, point_curr);
    po->x = point_w.x();
    po->y = point_w.y();
    po->z = point_w.z();
  }

  /**
   *transformPointCloud
   *@brief
   *将点云变换到地图坐标系
   *
   *@param[in] cloudIn-输入点云
   *@param[in] pose-变换pose
   *@return laserCloud::Ptr
   **/
  static laserCloud::Ptr transformPointCloud(const Mat34d &pose, laserCloud::Ptr cloudIn) {
    laserCloud::Ptr cloudOut(new laserCloud());
    PointType pointTo;
    if (nullptr == cloudIn) {
      LOG(ERROR) << " cloudIn == nullptr, point cloud transfrom error ";
      return nullptr;
    }

    int cloudSize = cloudIn->points.size();
    cloudOut->resize(cloudSize);

    for (int i = 0; i < cloudSize; ++i) {
      pointAssociateToMap(&cloudIn->points[i], &pointTo, pose);
      cloudOut->points[i] = pointTo;
    }
    return cloudOut;
  }

  static depthCloud::Ptr transformDepthCloud(const Mat34d &pose, depthCloud::Ptr cloudIn) {
    depthCloud::Ptr cloudOut(new depthCloud());
    depthPointType pointTo;

    int cloudSize = cloudIn->points.size();
    cloudOut->resize(cloudSize);

    for (int i = 0; i < cloudSize; ++i) {
      pointXYZAssociateToMap(&cloudIn->points[i], &pointTo, pose);
      cloudOut->points[i] = pointTo;
    }
    return cloudOut;
  }

  static RGBCloud::Ptr transformColorPointCloud(const Mat34d &pose, RGBCloud::Ptr cloudIn) {
    RGBCloud::Ptr cloudOut(new RGBCloud());
    PointRGBType pointTo;
    if (nullptr == cloudIn) {
      LOG(ERROR) << " cloudIn == nullptr, point cloud transfrom error ";
      return nullptr;
    }

    int cloudSize = cloudIn->points.size();
    cloudOut->resize(cloudSize);

    for (int i = 0; i < cloudSize; ++i) {
      cloudOut->points[i] = cloudIn->points[i];

      Vec3d point_curr(cloudIn->points[i].x, cloudIn->points[i].y, cloudIn->points[i].z);
      Vec3d point_w = Mathbox::multiplePoint(pose, point_curr);

      cloudOut->points[i].x = point_w.x();
      cloudOut->points[i].y = point_w.y();
      cloudOut->points[i].z = point_w.z();
    }
    return cloudOut;
  }

};  // end of class

}  // namespace GR_SLAM

#endif  // SLAM_TRANSFORM_HPP_