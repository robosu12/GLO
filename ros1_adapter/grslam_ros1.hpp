#ifndef _NEW_GR_SLAM_ROS1_
#define _NEW_GR_SLAM_ROS1_

// ROS1
#include <ros/ros.h>
#include <std_msgs/Header.h>
#include <std_msgs/Float32.h>
#include <std_msgs/Bool.h>
#include <sensor_msgs/Imu.h>
#include <sensor_msgs/PointCloud.h>
#include <sensor_msgs/PointCloud2.h>
#include <sensor_msgs/LaserScan.h>
#include <sensor_msgs/NavSatFix.h>
#include <nav_msgs/Path.h>
#include <nav_msgs/Odometry.h>
#include <geometry_msgs/PoseStamped.h>
#include <tf/transform_datatypes.h>
#include <tf/transform_broadcaster.h>
#include <time.h>
#include <fstream>

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

namespace GR_SLAM {
class GRSLAM {
 public:
  GRSLAM(ros::NodeHandle &ros_node, const std::string &config_path);
  GRSLAM() = delete;
  ~GRSLAM();
  GRSLAM(const GRSLAM &obj) = delete;
  GRSLAM &operator=(const GRSLAM &obj) = delete;
  void spin();
  void init();
  void registerDataPublisher();
  void registerDataCallback();
  void resetDataCallback();

 private:
  void updateParameter();
  void clear();

  void laserCloudDataCallback(const sensor_msgs::PointCloud2ConstPtr &cloud_msg);
  void laserScanDataCallback(const sensor_msgs::LaserScanConstPtr &laserScanMsg);
  void odomDataCallback(const nav_msgs::OdometryConstPtr &odom_msg);
  void IMUDataCallback(const sensor_msgs::ImuConstPtr &ImuMsg);

  void readDataThread();
  void publishThread();

  void publishPath(double time, const Mat34d &slam_pose, const Mat34d &lidar_odom_pose,
                   const Mat34d &wheel_pose);
  void publishTf(const Mat34d &pose, double time);
  bool publishLocalizationResult(const Mat34d &pose, double time, const double cov);

  laserCloud::Ptr removeNanAndNoisePoints(const laserCloud::Ptr cloud_in);

  static inline Mat34d fromGeometryMsg(const geometry_msgs::Pose &msg) {
    Mat34d pose;
    Vec3d translation(msg.position.x, msg.position.y, msg.position.z);
    Eigen::Quaterniond q(msg.orientation.w, msg.orientation.x, msg.orientation.y,
                         msg.orientation.z);
    q.normalize();
    pose.block<3, 3>(0, 0) = q.toRotationMatrix();
    pose.block<3, 1>(0, 3) = translation;
    return pose;
  }

  static inline Mat34d fromOdometryMsg(const nav_msgs::OdometryConstPtr odom_msg) {
    geometry_msgs::Pose msg;
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

  static inline geometry_msgs::Pose toGeometryMsg(const Mat34d &in) {
    geometry_msgs::Pose msg;
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

  static inline nav_msgs::Odometry toOdometryMsg(const Mat34d &in) {
    geometry_msgs::Pose msg = toGeometryMsg(in);
    nav_msgs::Odometry odom_msg;
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
  ros::NodeHandle ros_node_;

  ros::Subscriber sub_laser_cloud_;
  ros::Subscriber sub_laser_scan_;
  ros::Subscriber sub_odom_;
  ros::Subscriber sub_imu_;
  ros::Subscriber sub_uwb_muti_;
  ros::Subscriber sub_gps_;

  ros::Publisher pub_surroud_cloud_;
  ros::Publisher pub_global_cloud_;
  ros::Publisher pub_surf_cloud_;
  ros::Publisher pub_pose_cloud_;
  ros::Publisher pub_lidar_pose_;

  ros::Publisher pub_pose_;

  ros::Publisher pub_key_pose_path_;
  ros::Publisher pub_lidar_odom_path_;
  ros::Publisher pub_wheel_odom_path_;

  ros::Publisher pub_debug_data_;

  nav_msgs::Path key_pose_path_;
  nav_msgs::Path wheel_odom_path_;
  nav_msgs::Path lidar_odom_path_;

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

  std::shared_ptr<SystemConfig> ptr_config_ = nullptr;
  System *ptr_slam_system_ = nullptr;  ///< 当前帧指针

  std::thread pub_cloud_thread_;
  std::thread read_data_thread_;

  std::string config_path_ = "src/slam_3d/params/3DSLAM_RS16.yaml";
};
}  // namespace GR_SLAM
#endif