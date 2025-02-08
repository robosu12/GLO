#include <pcl/common/centroid.h>
#include <algorithm>
#include <cmath>
#include <list>
#include <vector>

namespace GR_SLAM {

// squared distance of two pcl points
template <typename PointT>
inline double distance2(const PointT &pt1, const PointT &pt2) {
  Eigen::Vector3f d = pt1.getVector3fMap() - pt2.getVector3fMap();
  return d.squaredNorm();
}

// convert from pcl point to eigen
template <typename T, int dim, typename PointType>
inline Eigen::Matrix<T, dim, 1> ToEigen(const PointType &pt) {
  return Eigen::Matrix<T, dim, 1>(pt.x, pt.y, pt.z);
}

template <>
inline Eigen::Matrix<float, 3, 1> ToEigen<float, 3, pcl::PointXYZ>(const pcl::PointXYZ &pt) {
  return pt.getVector3fMap();
}

template <>
inline Eigen::Matrix<float, 3, 1> ToEigen<float, 3, pcl::PointXYZI>(const pcl::PointXYZI &pt) {
  return pt.getVector3fMap();
}

template <>
inline Eigen::Matrix<float, 3, 1> ToEigen<float, 3, pcl::PointXYZINormal>(
    const pcl::PointXYZINormal &pt) {
  return pt.getVector3fMap();
}

template <typename PointT, int dim = 3>
class IVoxNode {
 public:
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW;

  using KeyType = Eigen::Matrix<int, dim, 1>;

  struct DistPoint;

  IVoxNode() = default;
  IVoxNode(const PointT &pt) { point_ = pt; }

  void InsertPoint(const PointT &pt);

  void SetPoint(const PointT &pt);

  inline bool Empty() const;

  inline std::size_t Size() const;

  inline PointT GetPoint(const std::size_t idx) const;

  inline PointT GetPoint() const;

  int KNNPointByCondition(std::vector<DistPoint> &dis_points, const PointT &point, const int &K,
                          const double &max_range);

  void SetOccpValue(const char &p) {
    prob_ = p;
    point_.intensity = prob_;
    update_count_ = 15;
  }
  char GetOccpValue() { return prob_; }
  void OccpIncrease() {
    if (update_count_ > 0 && prob_ < 45) {
      prob_ += 3;
      point_.intensity = prob_;
    }
  }
  void OccpDecrease() {
    if (update_count_ > 0 && prob_ > 20) {
      prob_ -= 1;
      point_.intensity = prob_;

      update_count_--;
    }
  }

  bool is_fit_ok_ = false;
  Eigen::Vector4f pabcd_;
  float fit_error_ = 0.0;

  bool is_high_fit_ok_ = false;
  Eigen::Vector4f high_pabcd_;
  float high_fit_error_ = 0.0;
  std::unordered_map<KeyType, PointT, hash_vec<dim>> m_points_;
  bool update_flag = true;

 private:
  // std::vector<PointT> points_;
  PointT point_;
  unsigned char prob_ = 0;
  unsigned char update_count_ = 0;
};

template <typename PointT, int dim>
struct IVoxNode<PointT, dim>::DistPoint {
  double dist = 0;
  IVoxNode *node = nullptr;
  int idx = 0;

  DistPoint() = default;
  DistPoint(const double d, IVoxNode *n, const int i) : dist(d), node(n), idx(i) {}

  PointT Get() { return node->GetPoint(idx); }

  inline bool operator()(const DistPoint &p1, const DistPoint &p2) { return p1.dist < p2.dist; }

  inline bool operator<(const DistPoint &rhs) { return dist < rhs.dist; }
};

template <typename PointT, int dim>
void IVoxNode<PointT, dim>::InsertPoint(const PointT &pt) {}

template <typename PointT, int dim>
void IVoxNode<PointT, dim>::SetPoint(const PointT &pt) {
  point_ = pt;
}

template <typename PointT, int dim>
PointT IVoxNode<PointT, dim>::GetPoint() const {
  return point_;
}

}  // namespace GR_SLAM
