/************************************************************************
 * Software License Agreement (BSD License)

 *@author Yun Su(robosu12@gmail.com)
 *@version 1.0
 *@data 2024-02-20
 ************************************************************************/
#ifndef SPACE_VOXEL_HPP_
#define SPACE_VOXEL_HPP_

#include <glog/logging.h>
#include <unordered_map>
#include <vector>

#include "common/debug_tools/debug_color.h"
#include "common/math_base/slam_math.hpp"
#include "common/data_struct/pc_base.hpp"

namespace GR_SLAM {

struct hash_vec3i {
  size_t operator()(const Eigen::Matrix<int, 3, 1> &v) const {
    return size_t(((v[0]) * 73856093) ^ ((v[1]) * 471943) ^ ((v[2]) * 83492791)) % 10000000;
  };
};
template <typename point_type = PointType>
class SpaceVoxel {
 public:
  using KeyType = Eigen::Matrix<int, 3, 1>;
  using cloud_type = pcl::PointCloud<point_type>;

  SpaceVoxel(float vox_size_x, float vox_size_y, float vox_size_z) {
    size_x_ = vox_size_x;
    size_y_ = vox_size_y;
    size_z_ = vox_size_z;
    inv_size_x_ = 1.0 / size_x_;
    inv_size_y_ = 1.0 / size_y_;
    inv_size_z_ = 1.0 / size_z_;

    LOG(INFO) << "SpaceVoxel: size_x:" << size_x_ << " , size_y:" << size_y_
              << " , size_z:" << size_z_;
    voxel_map_.clear();
  }

  ~SpaceVoxel() { voxel_map_.clear(); }

  void Clear() { voxel_map_.clear(); }

  void setLeafSize(const float x, const float y, const float z) {
    size_x_ = x;
    size_y_ = y;
    size_z_ = z;
    inv_size_x_ = 1.0 / size_x_;
    inv_size_y_ = 1.0 / size_y_;
    inv_size_z_ = 1.0 / size_z_;

    LOG(WARNING) << "SpaceVoxel: size_x:" << size_x_ << " , size_y:" << size_y_
                 << " , size_z:" << size_z_;
  }

  void InsertPoint(const point_type &point) {
    KeyType cur_key;
    // cur_key[0] = std::round(point.x * inv_size_x_);
    // cur_key[1] = std::round(point.y * inv_size_y_);
    // cur_key[2] = std::round(point.z * inv_size_z_);

    // 采用以下四舍五入方法效率更高，效率提升20%；
    if (point.x > 0.0) {
      cur_key[0] = point.x * inv_size_x_ + 0.5;
    } else {
      cur_key[0] = point.x * inv_size_x_ - 0.5;
    }
    if (point.y > 0.0) {
      cur_key[1] = point.y * inv_size_y_ + 0.5;
    } else {
      cur_key[1] = point.y * inv_size_y_ - 0.5;
    }
    if (point.z > 0.0) {
      cur_key[2] = point.z * inv_size_z_ + 0.5;
    } else {
      cur_key[2] = point.z * inv_size_z_ - 0.5;
    }

    if (voxel_map_.find(cur_key) == voxel_map_.end()) {
      voxel_map_.insert({cur_key, point});
    }
  }

  void InsertCloud(const cloud_type &cloud_in) {
    unsigned int frame_number = cloud_in.size();
    if (0 == frame_number)
      return;
    for (unsigned int i = 0; i < frame_number; ++i) { InsertPoint(cloud_in.points[i]); }
  }

  bool getVoxelCloud(cloud_type &cloud_out) {
    if (0 == voxel_map_.size()) {
      return false;
    }
    cloud_out.clear();
    for (const auto &it : voxel_map_) { cloud_out.points.emplace_back(it.second); }
    return true;
  }

 private:
  std::unordered_map<KeyType, point_type, hash_vec3i> voxel_map_;
  float size_x_ = 0.5, inv_size_x_ = 2.0;
  float size_y_ = 0.5, inv_size_y_ = 2.0;
  float size_z_ = 0.5, inv_size_z_ = 2.0;
};

}  // namespace GR_SLAM

#endif