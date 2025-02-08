//
// Created by xiang on 2021/9/16.
//

#ifndef FASTER_LIO_IVOX3D_H
#define FASTER_LIO_IVOX3D_H

#include <glog/logging.h>
#include <execution>
#include <list>
#include <thread>
#include <unordered_set>

#include "eigen_types.h"
#include "iPvox_node.hpp"
#include "common/debug_tools/tic_toc.h"

namespace GR_SLAM {

enum class IVoxNodeType {
  DEFAULT,  // linear ivox
  PHC,      // phc ivox
};

/// traits for NodeType
template <IVoxNodeType node_type, typename PointT, int dim>
struct IVoxNodeTypeTraits {};

template <typename PointT, int dim>
struct IVoxNodeTypeTraits<IVoxNodeType::DEFAULT, PointT, dim> {
  using NodeType = IVoxNode<PointT, dim>;
};

template <int dim = 3, IVoxNodeType node_type = IVoxNodeType::DEFAULT,
          typename PointType = pcl::PointXYZ>
class IVox {
 public:
  using KeyType = Eigen::Matrix<int, dim, 1>;
  using PtType = Eigen::Matrix<float, dim, 1>;
  using NodeType = typename IVoxNodeTypeTraits<node_type, PointType, dim>::NodeType;
  using PointVector = std::vector<PointType, Eigen::aligned_allocator<PointType>>;
  using DistPoint = typename NodeType::DistPoint;

  enum class NearbyType {
    CENTER = 0,  // center only
    NEARBY7 = 1,
    NEARBY19 = 2,
    NEARBY27 = 3,
    NEARBY81 = 4,
    NEARBY125 = 5
  };

  struct Options {
    float resolution_ = 0.4;        // ivox resolution
    float inv_resolution_ = 2.5;    // inverse resolution
    unsigned int nearby_type_ = 3;  // nearby range
    std::size_t capacity_ = 50000;  // capacity
    char init_prob_ = 30;
    float max_range_ = 30.0;
    float half_max_range_ = 15.0;
    unsigned int near_num_ = 5;
    unsigned int max_point_num_ = 10;

    bool use_hierachy_vox_ = true;
    float high_ratio_ = 5.0;
    float high_resolution_ = 0.1;
    float inv_high_reso_ = 10.0;
  };

  /**
   * constructor
   * @param options  ivox options
   */
  explicit IVox(Options options) : options_(options) {
    options_.inv_resolution_ = 1.0 / options_.resolution_;
    options_.half_max_range_ = 0.5 * options_.max_range_;

    options_.high_resolution_ = options_.resolution_ / options_.high_ratio_;
    options_.inv_high_reso_ = 1.0 / options_.high_resolution_;

    options_.max_point_num_ = 2 * options_.near_num_;
    // options_.max_point_num_ = options_.near_num_;

    LOG(INFO) << "ivox::options::resolution: " << options_.resolution_;
    LOG(INFO) << "ivox::options::nearby_type: " << options_.nearby_type_;
    LOG(INFO) << "ivox::options::near_num: " << options_.near_num_;
    LOG(INFO) << "ivox::options::max_point_num: " << options_.max_point_num_;
    LOG(INFO) << "ivox::options::capacity: " << options_.capacity_;
    LOG(INFO) << "ivox::options::use_hierachy_vox: " << options_.use_hierachy_vox_;

    GenerateNearbyGrids();
  }

  /**
   * add points
   * @param points_to_add
   */
  void AddPoints(const PointVector &points_to_add, const bool &prob_update);

  /// get nn
  bool GetClosestPoint(const PointType &pt, PointType &closest_pt);

  /// get nn with condition
  bool GetClosestPoint(const PointType &pt, PointVector &closest_pt, int max_num = 5,
                       double max_range = 5.0);

  /// get nn in cloud
  bool GetClosestPoint(const PointVector &cloud, PointVector &closest_cloud);

  /// get near points
  int GetNearPoints(const PointType &pt, PointVector &near_pts, int max_num, float max_range);

  /// get near points
  int GetNearPoints(const PointType &pt, PointVector &near_pts);

  int GetNearGrid(
      const KeyType &key, const int &max_num,
      std::vector<typename std::list<std::pair<KeyType, NodeType>>::iterator> &near_grid);

  void UpdateNearbyGrid(const typename std::list<std::pair<KeyType, NodeType>>::iterator &new_grid);

  bool GetNearPlane(const PointType &pt, PointVector &near_pts, Eigen::Vector4f &pabcd,
                    float &fit_error);

  /// get number of points
  size_t NumPoints() const;

  /// get number of valid grids
  size_t NumValidGrids() const;

  /// get statistics of the points
  std::vector<float> StatGridPoints() const;

  /// get all of the points
  void GetAllPoints(PointVector &all_points);

  /// reset
  void Clear();

 private:
  /// generate the nearby grids according to the given options
  void GenerateNearbyGrids();

  /// position to grid
  KeyType Pos2Grid(const PtType &pt) const;

  KeyType Pos2HighGrid(const PtType &pt) const;

  Options options_;
  std::unordered_map<KeyType, typename std::list<std::pair<KeyType, NodeType>>::iterator,
                     hash_vec<dim>>
      grids_map_;                                        // voxel hash map
  std::list<std::pair<KeyType, NodeType>> grids_cache_;  // voxel cache
  std::vector<KeyType> nearby_grids_;                    // nearbys
  std::vector<KeyType> nearby26_grids_;                  // nearbys
  bool prob_update_;
};

template <int dim, IVoxNodeType node_type, typename PointType>
bool IVox<dim, node_type, PointType>::GetClosestPoint(const PointType &pt, PointType &closest_pt) {
  std::vector<DistPoint> candidates;
  auto key = Pos2Grid(ToEigen<float, dim>(pt));
  std::for_each(nearby_grids_.begin(), nearby_grids_.end(),
                [&key, &candidates, &pt, this](const KeyType &delta) {
                  auto dkey = key + delta;
                  auto iter = grids_map_.find(dkey);
                  if (iter != grids_map_.end()) {
                    DistPoint dist_point;
                    bool found = iter->second->second.NNPoint(pt, dist_point);
                    if (found) {
                      candidates.emplace_back(dist_point);
                    }
                  }
                });

  if (candidates.empty()) {
    return false;
  }

  auto iter = std::min_element(candidates.begin(), candidates.end());
  closest_pt = iter->Get();
  return true;
}

template <int dim, IVoxNodeType node_type, typename PointType>
bool IVox<dim, node_type, PointType>::GetClosestPoint(const PointType &pt, PointVector &closest_pt,
                                                      int max_num, double max_range) {
  std::vector<DistPoint> candidates;
  candidates.reserve(max_num * nearby_grids_.size());

  auto key = Pos2Grid(ToEigen<float, dim>(pt));

  for (const KeyType &delta : nearby_grids_) {
    auto dkey = key + delta;
    auto iter = grids_map_.find(dkey);
    if (iter != grids_map_.end()) {
      auto tmp = iter->second->second.KNNPointByCondition(candidates, pt, max_num, max_range);
    }
  }

  if (candidates.empty()) {
    return false;
  }

  if (candidates.size() <= max_num) {
  } else {
    std::nth_element(candidates.begin(), candidates.begin() + max_num - 1, candidates.end());
    candidates.resize(max_num);
  }
  std::nth_element(candidates.begin(), candidates.begin(), candidates.end());

  closest_pt.clear();
  for (auto &it : candidates) { closest_pt.emplace_back(it.Get()); }
  return closest_pt.empty() == false;
}

template <int dim, IVoxNodeType node_type, typename PointType>
int IVox<dim, node_type, PointType>::GetNearPoints(const PointType &pt, PointVector &near_pts,
                                                   int max_num, float max_range) {
  near_pts.clear();  // 根据近邻模型，返回的点可以近似看成从近到远排列；
  auto key = Pos2Grid(ToEigen<float, dim>(pt));
  for (const KeyType &delta : nearby_grids_) {
    auto dkey = key + delta;
    auto iter = grids_map_.find(dkey);
    if (iter != grids_map_.end()) {
      near_pts.emplace_back(iter->second->second.GetPoint());
      if (near_pts.size() >= max_num) {
        break;
      }
    }
  }

  return near_pts.size();
}

template <int dim, IVoxNodeType node_type, typename PointType>
bool IVox<dim, node_type, PointType>::GetNearPlane(const PointType &pt, PointVector &near_pts,
                                                   Eigen::Vector4f &pabcd, float &fit_error) {
  Eigen::Vector3f norm_vec;
  float norm = 0.0;
  unsigned int points_num = 0;
  PointType p;
  KeyType key, dkey, nkey;
  PointVector v_near_p;

  key = Pos2Grid(ToEigen<float, dim>(pt));
  for (const KeyType &delta : nearby_grids_) {
    dkey = key + delta;
    auto iter = grids_map_.find(dkey);
    if (iter == grids_map_.end()) {
      continue;
    }

    points_num = iter->second->second.m_points_.size();

    // 如果体素内部已经完成拟合且内部点没有更新，则直接使用体素内部点拟合的结构参数；
    if (iter->second->second.is_high_fit_ok_ == true && iter->second->second.update_flag == false) {
      near_pts.emplace_back(iter->second->second.GetPoint());
      pabcd = iter->second->second.high_pabcd_;
      fit_error = iter->second->second.high_fit_error_;

      return true;
    }  // 如果体素内部点有更新，或者还未完成拟合且点数满足，则利用内部点进行拟合；
    else if (points_num >= options_.near_num_) {
      Eigen::MatrixXf AA(points_num, 3);
      Eigen::MatrixXf bb(points_num, 1);
      AA.setZero();
      bb.setOnes();
      bb *= -1.0f;

      int row = 0;
      for (auto miter = iter->second->second.m_points_.begin();
           miter != iter->second->second.m_points_.end(); miter++) {
        p = miter->second;
        AA(row, 0) = p.x;
        AA(row, 1) = p.y;
        AA(row, 2) = p.z;
        row++;
      }
      // 线性最小二乘求解平面方程: ax+by+cz+d=0
      norm_vec = AA.colPivHouseholderQr().solve(bb);
      norm = norm_vec.norm();

      pabcd(0) = norm_vec(0) / norm;
      pabcd(1) = norm_vec(1) / norm;
      pabcd(2) = norm_vec(2) / norm;
      pabcd(3) = 1.0 / norm;

      fit_error = 0.0;
      for (int j = 0; j < points_num; j++) {
        fit_error +=
            std::abs(pabcd(0) * AA(j, 0) + pabcd(1) * AA(j, 1) + pabcd(2) * AA(j, 2) + pabcd(3));
      }
      iter->second->second.high_pabcd_ = pabcd;
      iter->second->second.high_fit_error_ = fit_error;
      iter->second->second.is_high_fit_ok_ = true;
      iter->second->second.update_flag = false;
      near_pts.emplace_back(iter->second->second.GetPoint());

      return true;
    }  // 体素内部点数不足，则读取体素近邻点拟合好的结构参数；
    else if (iter->second->second.is_fit_ok_ == true) {
      near_pts.emplace_back(iter->second->second.GetPoint());
      pabcd = iter->second->second.pabcd_;
      fit_error = iter->second->second.fit_error_;

      return true;
    }  // 如果体素外部还未拟合，且近邻点数满足，则进行外部拟合；
    else {
      if (options_.use_hierachy_vox_) {
        // 将体素内所有的点拿出来；
        for (auto miter = iter->second->second.m_points_.begin();
             miter != iter->second->second.m_points_.end(); miter++) {
          v_near_p.emplace_back(miter->second);
        }
      } else {
        v_near_p.emplace_back(iter->second->second.GetPoint());
      }

      if (v_near_p.size() >= options_.near_num_) {
        Eigen::MatrixXf A(options_.near_num_, 3);
        Eigen::MatrixXf b(options_.near_num_, 1);

        A.setZero();
        b.setOnes();
        b *= -1.0f;

        for (int j = 0; j < options_.near_num_; j++) {
          A(j, 0) = v_near_p[j].x;
          A(j, 1) = v_near_p[j].y;
          A(j, 2) = v_near_p[j].z;
        }
        // 线性最小二乘求解平面方程: ax+by+cz+d=0
        norm_vec = A.colPivHouseholderQr().solve(b);
        norm = norm_vec.norm();

        pabcd(0) = norm_vec(0) / norm;
        pabcd(1) = norm_vec(1) / norm;
        pabcd(2) = norm_vec(2) / norm;
        pabcd(3) = 1.0 / norm;

        fit_error = 0.0;
        for (int j = 0; j < options_.near_num_; j++) {
          fit_error +=
              std::abs(pabcd(0) * A(j, 0) + pabcd(1) * A(j, 1) + pabcd(2) * A(j, 2) + pabcd(3));
        }
        fit_error = fit_error * options_.high_ratio_ + options_.resolution_;

        iter->second->second.pabcd_ = pabcd;
        iter->second->second.fit_error_ = fit_error;
        iter->second->second.is_fit_ok_ = true;
        near_pts.emplace_back(v_near_p.front());

        return true;
      }
    }
  }

  return false;
}

template <int dim, IVoxNodeType node_type, typename PointType>
int IVox<dim, node_type, PointType>::GetNearGrid(
    const KeyType &key, const int &max_num,
    std::vector<typename std::list<std::pair<KeyType, NodeType>>::iterator> &near_grid) {
  near_grid.clear();
  for (const KeyType &delta : nearby_grids_) {
    auto dkey = key + delta;
    auto iter = grids_map_.find(dkey);
    if (iter != grids_map_.end()) {
      near_grid.emplace_back(iter->second);
      if (near_grid.size() >= max_num) {
        break;
      }
    }
  }
  return near_grid.size();
}

template <int dim, IVoxNodeType node_type, typename PointType>
void IVox<dim, node_type, PointType>::AddPoints(const PointVector &points_to_add,
                                                const bool &prob_update) {
  // static float update_near_time_sum = 0.0;
  // static float update_near_time_ave = 0.0;
  // static int update_count = 0;
  PointType pt;
  KeyType key, high_key;
  std::unordered_set<KeyType, hash_vec<dim>> inserted_vox;

  for (int i = 0; i < points_to_add.size(); ++i) {
    pt = points_to_add[i];
    key = Pos2Grid(ToEigen<float, dim>(pt));

    auto iter = grids_map_.find(key);
    if (iter == grids_map_.end()) {
      grids_cache_.push_front({key, NodeType(pt)});
      grids_map_.emplace(key, grids_cache_.begin());

      if (prob_update) {
        grids_cache_.front().second.SetOccpValue(options_.init_prob_);
        inserted_vox.emplace(key);
      }

      if (options_.use_hierachy_vox_) {
        high_key = Pos2HighGrid(ToEigen<float, dim>(pt));
        grids_cache_.front().second.m_points_.emplace(high_key, pt);
      }

      if (grids_map_.size() > options_.capacity_) {
        grids_map_.erase(grids_cache_.back().first);
        grids_cache_.pop_back();
      }

    } else {
      if (options_.use_hierachy_vox_) {
        if (iter->second->second.m_points_.size() < options_.max_point_num_) {
          high_key = Pos2HighGrid(ToEigen<float, dim>(pt));
          auto pair = iter->second->second.m_points_.try_emplace(high_key, pt);
          if (pair.second) {
            iter->second->second.update_flag = true;
          }
        }
      }

      // 添加新点的体素都会被移动到链表的最前面（重复观测过的体素）；
      // 长时间未添加点的体素会逐渐移动到链表后面（旧观测），最后超过容量的将被删除；
      grids_cache_.splice(grids_cache_.begin(), grids_cache_, iter->second);
      grids_map_[key] = grids_cache_.begin();
      if (prob_update) {
        if (inserted_vox.find(key) == inserted_vox.end()) {
          grids_cache_.begin()->second.OccpIncrease();
          inserted_vox.emplace(key);
        }
      }
    }
  }

  // LOG(WARNING) << "update_near_time_sum: " << update_near_time_sum;

  // 对体素地图进行概率衰减；
  if (prob_update) {
    for (auto iter = grids_cache_.begin(); iter != grids_cache_.end(); iter++) {
      iter->second.OccpDecrease();
    }
  }
}

template <int dim, IVoxNodeType node_type, typename PointType>
size_t IVox<dim, node_type, PointType>::NumValidGrids() const {
  return grids_map_.size();
}

template <int dim, IVoxNodeType node_type, typename PointType>
void IVox<dim, node_type, PointType>::GenerateNearbyGrids() {
  if (options_.nearby_type_ == 0) {  // NearbyType::CENTER
    nearby_grids_.emplace_back(KeyType::Zero());
  } else if (options_.nearby_type_ == 1) {  // NearbyType::NEARBY7
    nearby_grids_ = {KeyType(0, 0, 0),  KeyType(-1, 0, 0), KeyType(1, 0, 0), KeyType(0, 1, 0),
                     KeyType(0, -1, 0), KeyType(0, 0, -1), KeyType(0, 0, 1)};
  } else if (options_.nearby_type_ == 2) {  // NearbyType::NEARBY19
    nearby_grids_ = {KeyType(0, 0, 0),  KeyType(-1, 0, 0), KeyType(1, 0, 0),   KeyType(0, 1, 0),
                     KeyType(0, -1, 0), KeyType(0, 0, -1), KeyType(0, 0, 1),   KeyType(1, 1, 0),
                     KeyType(-1, 1, 0), KeyType(1, -1, 0), KeyType(-1, -1, 0), KeyType(1, 0, 1),
                     KeyType(-1, 0, 1), KeyType(1, 0, -1), KeyType(-1, 0, -1), KeyType(0, 1, 1),
                     KeyType(0, -1, 1), KeyType(0, 1, -1), KeyType(0, -1, -1)};
  } else if (options_.nearby_type_ == 3) {  // NearbyType::NEARBY27
    nearby_grids_ = {KeyType(0, 0, 0),   KeyType(-1, 0, 0),  KeyType(1, 0, 0),   KeyType(0, 1, 0),
                     KeyType(0, -1, 0),  KeyType(0, 0, -1),  KeyType(0, 0, 1),   KeyType(1, 1, 0),
                     KeyType(-1, 1, 0),  KeyType(1, -1, 0),  KeyType(-1, -1, 0), KeyType(1, 0, 1),
                     KeyType(-1, 0, 1),  KeyType(1, 0, -1),  KeyType(-1, 0, -1), KeyType(0, 1, 1),
                     KeyType(0, -1, 1),  KeyType(0, 1, -1),  KeyType(0, -1, -1), KeyType(1, 1, 1),
                     KeyType(-1, 1, 1),  KeyType(1, -1, 1),  KeyType(1, 1, -1),  KeyType(-1, -1, 1),
                     KeyType(-1, 1, -1), KeyType(1, -1, -1), KeyType(-1, -1, -1)};
  } else if (options_.nearby_type_ == 4) {  // NearbyType::NEARBY81 27 + 9*6
    nearby_grids_ = {
        KeyType(0, 0, 0),    KeyType(-1, 0, 0),  KeyType(1, 0, 0),    KeyType(0, 1, 0),
        KeyType(0, -1, 0),   KeyType(0, 0, -1),  KeyType(0, 0, 1),    KeyType(1, 1, 0),
        KeyType(-1, 1, 0),   KeyType(1, -1, 0),  KeyType(-1, -1, 0),  KeyType(1, 0, 1),
        KeyType(-1, 0, 1),   KeyType(1, 0, -1),  KeyType(-1, 0, -1),  KeyType(0, 1, 1),
        KeyType(0, -1, 1),   KeyType(0, 1, -1),  KeyType(0, -1, -1),  KeyType(1, 1, 1),
        KeyType(-1, 1, 1),   KeyType(1, -1, 1),  KeyType(1, 1, -1),   KeyType(-1, -1, 1),
        KeyType(-1, 1, -1),  KeyType(1, -1, -1), KeyType(-1, -1, -1), KeyType(2, 0, 0),
        KeyType(-2, 0, 0),   KeyType(0, 2, 0),   KeyType(0, -2, 0),   KeyType(0, 0, 2),
        KeyType(0, 0, -2),   KeyType(2, 0, 1),   KeyType(2, 0, -1),   KeyType(2, 1, 0),
        KeyType(2, -1, 0),   KeyType(-2, 0, 1),  KeyType(-2, 0, -1),  KeyType(-2, 1, 0),
        KeyType(-2, -1, 0),  KeyType(1, 2, 0),   KeyType(-1, 2, 0),   KeyType(0, 2, 1),
        KeyType(0, 2, -1),   KeyType(1, -2, 0),  KeyType(-1, -2, 0),  KeyType(0, -2, 1),
        KeyType(0, -2, -1),  KeyType(1, 0, 2),   KeyType(-1, 0, 2),   KeyType(0, 1, 2),
        KeyType(0, -1, 2),   KeyType(1, 0, -2),  KeyType(-1, 0, -2),  KeyType(0, 1, -2),
        KeyType(0, -1, -2),  KeyType(2, 1, 1),   KeyType(2, -1, 1),   KeyType(2, 1, -1),
        KeyType(2, -1, -1),  KeyType(-2, 1, 1),  KeyType(-2, -1, 1),  KeyType(-2, 1, -1),
        KeyType(-2, -1, -1), KeyType(1, 2, 1),   KeyType(-1, 2, 1),   KeyType(1, 2, -1),
        KeyType(-1, 2, -1),  KeyType(1, -2, 1),  KeyType(-1, -2, 1),  KeyType(1, -2, -1),
        KeyType(-1, -2, -1), KeyType(1, 1, 2),   KeyType(1, -1, 2),   KeyType(-1, 1, 2),
        KeyType(-1, -1, 2),  KeyType(1, 1, -2),  KeyType(1, -1, -2),  KeyType(-1, 1, -2),
        KeyType(-1, -1, -2)};
  } else if (options_.nearby_type_ == 5) {  // NearbyType::NEARBY125 5*5*5
    nearby_grids_ = {
        KeyType(0, 0, 0),    KeyType(-1, 0, 0),  KeyType(1, 0, 0),    KeyType(0, 1, 0),
        KeyType(0, -1, 0),   KeyType(0, 0, -1),  KeyType(0, 0, 1),    KeyType(1, 1, 0),
        KeyType(-1, 1, 0),   KeyType(1, -1, 0),  KeyType(-1, -1, 0),  KeyType(1, 0, 1),
        KeyType(-1, 0, 1),   KeyType(1, 0, -1),  KeyType(-1, 0, -1),  KeyType(0, 1, 1),
        KeyType(0, -1, 1),   KeyType(0, 1, -1),  KeyType(0, -1, -1),  KeyType(1, 1, 1),
        KeyType(-1, 1, 1),   KeyType(1, -1, 1),  KeyType(1, 1, -1),   KeyType(-1, -1, 1),
        KeyType(-1, 1, -1),  KeyType(1, -1, -1), KeyType(-1, -1, -1), KeyType(2, 0, 0),
        KeyType(-2, 0, 0),   KeyType(0, 2, 0),   KeyType(0, -2, 0),   KeyType(0, 0, 2),
        KeyType(0, 0, -2),   KeyType(2, 0, 1),   KeyType(2, 0, -1),   KeyType(2, 1, 0),
        KeyType(2, -1, 0),   KeyType(-2, 0, 1),  KeyType(-2, 0, -1),  KeyType(-2, 1, 0),
        KeyType(-2, -1, 0),  KeyType(1, 2, 0),   KeyType(-1, 2, 0),   KeyType(0, 2, 1),
        KeyType(0, 2, -1),   KeyType(1, -2, 0),  KeyType(-1, -2, 0),  KeyType(0, -2, 1),
        KeyType(0, -2, -1),  KeyType(1, 0, 2),   KeyType(-1, 0, 2),   KeyType(0, 1, 2),
        KeyType(0, -1, 2),   KeyType(1, 0, -2),  KeyType(-1, 0, -2),  KeyType(0, 1, -2),
        KeyType(0, -1, -2),  KeyType(2, 1, 1),   KeyType(2, -1, 1),   KeyType(2, 1, -1),
        KeyType(2, -1, -1),  KeyType(-2, 1, 1),  KeyType(-2, -1, 1),  KeyType(-2, 1, -1),
        KeyType(-2, -1, -1), KeyType(1, 2, 1),   KeyType(-1, 2, 1),   KeyType(1, 2, -1),
        KeyType(-1, 2, -1),  KeyType(1, -2, 1),  KeyType(-1, -2, 1),  KeyType(1, -2, -1),
        KeyType(-1, -2, -1), KeyType(1, 1, 2),   KeyType(1, -1, 2),   KeyType(-1, 1, 2),
        KeyType(-1, -1, 2),  KeyType(1, 1, -2),  KeyType(1, -1, -2),  KeyType(-1, 1, -2),
        KeyType(-1, -1, -2), KeyType(2, 2, 0),   KeyType(-2, 2, 0),   KeyType(2, -2, 0),
        KeyType(-2, -2, 0),  KeyType(2, 0, 2),   KeyType(-2, 0, 2),   KeyType(2, 0, -2),
        KeyType(-2, 0, -2),  KeyType(0, 2, 2),   KeyType(0, -2, 2),   KeyType(0, 2, -2),
        KeyType(0, -2, -2),  KeyType(2, 2, 1),   KeyType(2, -2, 1),   KeyType(2, 2, -1),
        KeyType(2, -2, -1),  KeyType(-2, 2, 1),  KeyType(-2, -2, 1),  KeyType(-2, 2, -1),
        KeyType(-2, -2, -1), KeyType(1, 2, 2),   KeyType(-1, 2, 2),   KeyType(1, 2, -2),
        KeyType(-1, 2, -2),  KeyType(1, -2, 2),  KeyType(-1, -2, 2),  KeyType(1, -2, -2),
        KeyType(-1, -2, -2), KeyType(2, 1, 2),   KeyType(2, -1, 2),   KeyType(-2, 1, 2),
        KeyType(-2, -1, 2),  KeyType(2, 1, -2),  KeyType(2, -1, -2),  KeyType(-2, 1, -2),
        KeyType(-2, -1, -2), KeyType(2, 2, 2),   KeyType(-2, 2, 2),   KeyType(2, -2, 2),
        KeyType(2, 2, -2),   KeyType(-2, -2, 2), KeyType(-2, 2, -2),  KeyType(2, -2, -2),
        KeyType(-2, -2, -2)};
  } else {
    LOG(ERROR) << "Unknown nearby_type!";
  }

  nearby26_grids_ = {KeyType(-1, 0, 0),  KeyType(1, 0, 0),   KeyType(0, 1, 0),   KeyType(0, -1, 0),
                     KeyType(0, 0, -1),  KeyType(0, 0, 1),   KeyType(1, 1, 0),   KeyType(-1, 1, 0),
                     KeyType(1, -1, 0),  KeyType(-1, -1, 0), KeyType(1, 0, 1),   KeyType(-1, 0, 1),
                     KeyType(1, 0, -1),  KeyType(-1, 0, -1), KeyType(0, 1, 1),   KeyType(0, -1, 1),
                     KeyType(0, 1, -1),  KeyType(0, -1, -1), KeyType(1, 1, 1),   KeyType(-1, 1, 1),
                     KeyType(1, -1, 1),  KeyType(1, 1, -1),  KeyType(-1, -1, 1), KeyType(-1, 1, -1),
                     KeyType(1, -1, -1), KeyType(-1, -1, -1)};
}

template <int dim, IVoxNodeType node_type, typename PointType>
bool IVox<dim, node_type, PointType>::GetClosestPoint(const PointVector &cloud,
                                                      PointVector &closest_cloud) {
  std::vector<size_t> index(cloud.size());
  for (int i = 0; i < cloud.size(); ++i) { index[i] = i; }
  closest_cloud.resize(cloud.size());

  std::for_each(std::execution::par_unseq, index.begin(), index.end(),
                [&cloud, &closest_cloud, this](size_t idx) {
                  PointType pt;
                  if (GetClosestPoint(cloud[idx], pt)) {
                    closest_cloud[idx] = pt;
                  } else {
                    closest_cloud[idx] = PointType();
                  }
                });
  return true;
}

template <int dim, IVoxNodeType node_type, typename PointType>
Eigen::Matrix<int, dim, 1> IVox<dim, node_type, PointType>::Pos2Grid(const IVox::PtType &pt) const {
  return (pt * options_.inv_resolution_).array().round().template cast<int>();
}

template <int dim, IVoxNodeType node_type, typename PointType>
Eigen::Matrix<int, dim, 1> IVox<dim, node_type, PointType>::Pos2HighGrid(
    const IVox::PtType &pt) const {
  return (pt * options_.inv_high_reso_).array().round().template cast<int>();
}

template <int dim, IVoxNodeType node_type, typename PointType>
std::vector<float> IVox<dim, node_type, PointType>::StatGridPoints() const {
  int num = grids_cache_.size(), valid_num = 0, max = 0, min = 100000000;
  int sum = 0, sum_square = 0;
  for (auto &it : grids_cache_) {
    int s = it.second.Size();
    valid_num += s > 0;
    max = s > max ? s : max;
    min = s < min ? s : min;
    sum += s;
    sum_square += s * s;
  }
  float ave = float(sum) / num;
  float stddev = num > 1 ? sqrt((float(sum_square) - num * ave * ave) / (num - 1)) : 0;
  return std::vector<float>{valid_num, ave, max, min, stddev};
}

template <int dim, IVoxNodeType node_type, typename PointType>
void IVox<dim, node_type, PointType>::GetAllPoints(PointVector &all_points) {
  // int num = 0;
  // for (auto &it : grids_cache_) {
  //   num = it.second.Size();
  //   for (int j = 0; j < num; j++) {
  //     all_points.emplace_back(it.second.GetPoint(j));
  //   }
  // }
  for (auto &it : grids_cache_) { all_points.emplace_back(it.second.GetPoint()); }
}

template <int dim, IVoxNodeType node_type, typename PointType>
void IVox<dim, node_type, PointType>::Clear() {
  grids_cache_.clear();
  grids_map_.clear();
}

}  // namespace GR_SLAM

#endif
