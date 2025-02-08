#ifndef _NEW_LIDAR_SLAM_ROS2_
#define _NEW_LIDAR_SLAM_ROS2_

// ROS2
#include <tf2/convert.h>
#include <tf2/time.h>
#include <tf2/utils.h>
#include <tf2_ros/buffer.h>
#include <tf2_ros/transform_broadcaster.h>
#include <tf2_ros/transform_listener.h>
#include <tf2_sensor_msgs/tf2_sensor_msgs.h>
#include <time.h>
#include <fstream>

#include <geometry_msgs/msg/pose_array.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <geometry_msgs/msg/twist_stamped.hpp>
#include <geometry_msgs/msg/pose_with_covariance_stamped.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <nav_msgs/msg/path.hpp>
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/laser_scan.hpp>
#include <sensor_msgs/msg/nav_sat_fix.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <sensor_msgs/msg/point_cloud.hpp>
#include <sensor_msgs/point_cloud2_iterator.hpp>
#include <sensor_msgs/msg/imu.hpp>
#include <sensor_msgs/msg/range.hpp>
#include <visualization_msgs/msg/marker.hpp>
#include <visualization_msgs/msg/marker_array.hpp>
#include <std_msgs/msg/int64_multi_array.hpp>
#include <nav_msgs/msg/occupancy_grid.hpp>
#include <sensor_msgs/msg/range.hpp>

#include <pcl_conversions/pcl_conversions.h>

#include <glog/logging.h>
#include <pwd.h>
#include <sys/stat.h>
#include <sys/types.h>

#include <dirent.h>
#include <unistd.h>

#include "common/kitti.hpp"
#include "common/math_base/slam_math.hpp"
#include "system/system.hpp"
#include "load_param_ros2.hpp"

namespace GR_SLAM {
class GRSLAM {
 public:
  GRSLAM(rclcpp::Node::SharedPtr gr_slam_node);
  GRSLAM() = delete;
  ~GRSLAM();
  GRSLAM(const GRSLAM &obj) = delete;
  GRSLAM &operator=(const GRSLAM &obj) = delete;
  void spin();
  void registerDataPublisher();
  void registerDataCallback();
  void resetDataCallback();

 private:
  void init();
  void updateParameter();
  void clear();
  void laserScanDataCallback(const sensor_msgs::msg::LaserScan::SharedPtr laserScanMsg);
  void odomDataCallback(const nav_msgs::msg::Odometry::SharedPtr odom_msg);
  void uwbMutiDataCallback(const nav_msgs::msg::Odometry::SharedPtr uwb_msg);
  void uwbSingleDataCallback(const sensor_msgs::msg::Range::SharedPtr uwb_msg);
  void gpsDataCallback(const sensor_msgs::msg::NavSatFix::SharedPtr gps_msg);
  void gps2DataCallback(const sensor_msgs::msg::NavSatFix::SharedPtr gps_msg);

  void IMUDataCallback(const sensor_msgs::msg::Imu::SharedPtr ImuMsg);
  void laserCloudDataCallback(const sensor_msgs::msg::PointCloud2::SharedPtr cloud_msg);
  void imageDataCallback(const sensor_msgs::msg::Image::SharedPtr ImageMsg, const int &flag);
  void readDataThread();
  void publishThread();

  void publishPath(double time, const Mat34d &slam_pose, const Mat34d &lidar_odom_pose,
                   const Mat34d &wheel_pose);
  void publishTf(const Mat34d &pose, double time);
  bool publishLocalizationResult(const Mat34d &pose, double time, const double cov);

  laserCloud::Ptr removeNanAndNoisePoints(const laserCloud::Ptr cloud_in);

  static inline Mat34d fromGeometryMsg(const geometry_msgs::msg::Pose &msg) {
    Mat34d pose;
    Vec3d translation(msg.position.x, msg.position.y, msg.position.z);
    Eigen::Quaterniond q(msg.orientation.w, msg.orientation.x, msg.orientation.y,
                         msg.orientation.z);
    q.normalize();
    pose.block<3, 3>(0, 0) = q.toRotationMatrix();
    pose.block<3, 1>(0, 3) = translation;
    return pose;
  }

  static inline Mat34d fromOdometryMsg(const nav_msgs::msg::Odometry::SharedPtr odom_msg) {
    geometry_msgs::msg::Pose msg;
    msg.orientation.x = odom_msg->pose.pose.orientation.x;
    msg.orientation.y = odom_msg->pose.pose.orientation.y;
    msg.orientation.z = odom_msg->pose.pose.orientation.z;
    msg.orientation.w = odom_msg->pose.pose.orientation.w;
    msg.position.x = odom_msg->pose.pose.position.x;
    msg.position.y = odom_msg->pose.pose.position.y;
    msg.position.z = odom_msg->pose.pose.position.z;
    Mat34d pose;
    pose = fromGeometryMsg(msg);
    return pose;
  }

  static inline geometry_msgs::msg::Pose toGeometryMsg(const Mat34d &in) {
    geometry_msgs::msg::Pose msg;
    msg.position.x = in(0, 3);
    msg.position.y = in(1, 3);
    msg.position.z = in(2, 3);
    Eigen::Quaterniond q(in.block<3, 3>(0, 0));
    msg.orientation.x = q.x();
    msg.orientation.y = q.y();
    msg.orientation.z = q.z();
    msg.orientation.w = q.w();
    return msg;
  }

  static inline nav_msgs::msg::Odometry toOdometryMsg(const Mat34d &in) {
    geometry_msgs::msg::Pose msg;
    msg = toGeometryMsg(in);
    nav_msgs::msg::Odometry odom_msg;
    odom_msg.pose.pose.orientation.x = msg.orientation.x;
    odom_msg.pose.pose.orientation.y = msg.orientation.y;
    odom_msg.pose.pose.orientation.z = msg.orientation.z;
    odom_msg.pose.pose.orientation.w = msg.orientation.w;
    odom_msg.pose.pose.position.x = msg.position.x;
    odom_msg.pose.pose.position.y = msg.position.y;
    odom_msg.pose.pose.position.z = msg.position.z;
    return odom_msg;
  }

  inline Mat34d transformOdom(const Mat34d &in) {
    // 里程计坐标系下转换到世界坐标系下
    static Mat34d T_ol = Mathbox::inversePose34d(ptr_config_->T_lo);

    Mat34d T_o_0w = Mathbox::inversePose34d(T_wo_0_);
    Mat34d To_0_i = Mathbox::multiplePose34d(T_o_0w, in);

    Mat34d out =
        Mathbox::multiplePose34d(Mathbox::multiplePose34d(ptr_config_->T_lo, To_0_i), T_ol);
    return out;
  }

 private:
  rclcpp::Node::SharedPtr gr_slam_node_;

  rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr sub_laser_cloud_;
  rclcpp::Subscription<sensor_msgs::msg::LaserScan>::SharedPtr sub_laser_scan_;
  rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr sub_odom_;
  rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr sub_uwb_muti_;
  rclcpp::Subscription<sensor_msgs::msg::Range>::SharedPtr sub_uwb_single_;
  rclcpp::Subscription<sensor_msgs::msg::NavSatFix>::SharedPtr sub_gps_;
  rclcpp::Subscription<sensor_msgs::msg::NavSatFix>::SharedPtr sub_second_gps_;
  rclcpp::Subscription<sensor_msgs::msg::Imu>::SharedPtr sub_imu_;

  rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr pub_surroud_cloud_;
  rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr pub_global_cloud_;
  rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr pub_surf_cloud_;

  rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr pub_pose_cloud_;
  rclcpp::Publisher<geometry_msgs::msg::PoseWithCovarianceStamped>::SharedPtr pub_pose_;

  rclcpp::Publisher<nav_msgs::msg::Path>::SharedPtr pub_key_pose_path_;
  rclcpp::Publisher<nav_msgs::msg::Path>::SharedPtr pub_lidar_odom_path_;
  rclcpp::Publisher<nav_msgs::msg::Path>::SharedPtr pub_wheel_odom_path_;

  rclcpp::Publisher<nav_msgs::msg::Odometry>::SharedPtr pub_debug_data_;

  std::shared_ptr<tf2_ros::TransformBroadcaster> ptr_tfb_ = nullptr;
  std::shared_ptr<tf2_ros::TransformBroadcaster> ptr_tfl_ = nullptr;
  std::shared_ptr<tf2_ros::TransformBroadcaster> ptr_tf_camerainit_map_ = nullptr;
  std::shared_ptr<tf2_ros::TransformBroadcaster> ptr_tf_baselink_camera_ = nullptr;
  std::shared_ptr<tf2_ros::TransformBroadcaster> ptr_tf_laser_rslidar_ = nullptr;

  nav_msgs::msg::Path key_pose_path_;
  nav_msgs::msg::Path wheel_odom_path_;
  nav_msgs::msg::Path lidar_odom_path_;
  nav_msgs::msg::Path smooth_uwb_odom_path_;
  nav_msgs::msg::Path raw_uwb_odom_path_;
  nav_msgs::msg::Path smooth_gps_odom_path_;
  nav_msgs::msg::Path raw_gps_odom_path_;
  nav_msgs::msg::Path raw_gps_path_;
  nav_msgs::msg::Path aligned_gps_path_;
  nav_msgs::msg::Path raw_gps2_path_;
  nav_msgs::msg::Path aligned_gps2_path_;

  double last_keyframe_time_ = 0.;

  Mat34d T_wo_0_;  ///< 第一个odom数据
  Mat34d T_wo_;    ///< odom数据

  Mat34d last_raw_odom_pose_;    ///< 记录上一次odom的位置
  Mat34d last_odom_pose_;        ///< 记录上一次odom的位置
  Mat34d last_delta_odom_pose_;  ///< 记录上一次odom的位置

  Mat34d init_map_to_odom_;

  bool is_first_cloud_ = true;
  bool is_first_uwb_ = true;
  bool is_first_gps_ = true;
  bool is_first_odom_ = true;
  bool is_first_imu_ = true;
  bool is_debug_;
  bool print_debug_;
  bool publish_tf_;
  bool publish_cloud_;
  bool publish_cloud_loc_;
  bool publish_occ_map_;
  int publish_occ_map_period_ = 2000;
  bool publish_global_cloud_;
  bool publish_global_cloud_loc_;
  bool publish_path_;
  bool publish_path_loc_;
  bool publish_uwbodom_;
  bool publish_gpsodom_;
  size_t odom_count_;   ///<>
  size_t uwb_count_;    ///<>
  size_t gps_count_;    ///<>
  size_t lidar_count_;  ///<>
  size_t imu_count_;    ///<>
  double first_odom_time_;
  double curr_odom_time_;
  double curr_lidar_time_;
  double curr_uwb_time_;
  double curr_gps_time_;

  std::unique_ptr<LoadParamsRos2> slam_params_ptr_ = nullptr;
  std::shared_ptr<SystemConfig> ptr_config_ = nullptr;
  System *ptr_slam_system_ = nullptr;  ///< 当前帧指针

  std::thread pub_cloud_thread_;
  std::thread read_data_thread_;
};
}  // namespace GR_SLAM
#endif