#include "grslam_ros2.hpp"

namespace GR_SLAM {

GRSLAM::GRSLAM(rclcpp::Node::SharedPtr gr_slam_node) {
  gr_slam_node_ = gr_slam_node;

  is_first_odom_ = true;
  is_first_uwb_ = true;
  is_first_gps_ = true;
  is_first_cloud_ = true;
  odom_count_ = 0;
  uwb_count_ = 0;
  gps_count_ = 0;
  lidar_count_ = 0;
  imu_count_ = 0;
  slam_params_ptr_ = std::make_unique<LoadParamsRos2>(gr_slam_node_);
  ptr_slam_system_ = System::getInstance();

  init();
}

GRSLAM::~GRSLAM() {
  LOG(ERROR) << "SENSOR COUNT " << odom_count_ << ' ' << lidar_count_;
  ptr_slam_system_->shutdown();

  LOG(ERROR) << "system shutdown !!!";
}

void GRSLAM::init() {
  slam_params_ptr_->loadSystemConfig();

  ptr_config_ = slam_params_ptr_->ptr_config_;

  ptr_slam_system_->setConfig(ptr_config_);

  ptr_slam_system_->Reset();

  registerDataPublisher();

  resetDataCallback();

  registerDataCallback();
}

void GRSLAM::registerDataPublisher() {
  if (ptr_config_->publish_cloud) {
    pub_surf_cloud_ = gr_slam_node_->create_publisher<sensor_msgs::msg::PointCloud2>(
        "/surf_cloud", rclcpp::QoS(1).best_effort());
    pub_surroud_cloud_ = gr_slam_node_->create_publisher<sensor_msgs::msg::PointCloud2>(
        "/surround_map_cloud", rclcpp::QoS(1).best_effort());
    pub_global_cloud_ = gr_slam_node_->create_publisher<sensor_msgs::msg::PointCloud2>(
        "/global_map_cloud", rclcpp::QoS(1).best_effort());

    pub_pose_cloud_ = gr_slam_node_->create_publisher<sensor_msgs::msg::PointCloud2>(
        "/frame_pose_cloud", rclcpp::QoS(1).best_effort());
  }

  if (ptr_config_->publish_path) {
    pub_key_pose_path_ = gr_slam_node_->create_publisher<nav_msgs::msg::Path>(
        "/key_pose_path", rclcpp::QoS(1).best_effort());
    pub_lidar_odom_path_ = gr_slam_node_->create_publisher<nav_msgs::msg::Path>(
        "/lidar_odom_path", rclcpp::QoS(1).best_effort());
    pub_wheel_odom_path_ = gr_slam_node_->create_publisher<nav_msgs::msg::Path>(
        "/wheel_odom_path", rclcpp::QoS(1).best_effort());
  }

  if (ptr_config_->publish_tf) {
    ptr_tfl_ = std::make_shared<tf2_ros::TransformBroadcaster>(gr_slam_node_);
    ptr_tfb_ = std::make_shared<tf2_ros::TransformBroadcaster>(gr_slam_node_);
  }

  if (ptr_config_->is_debug) {
    pub_debug_data_ = gr_slam_node_->create_publisher<nav_msgs::msg::Odometry>(
        "/debug_data", rclcpp::QoS(1).best_effort());
  }

  pub_pose_ = gr_slam_node_->create_publisher<geometry_msgs::msg::PoseWithCovarianceStamped>(
      "/fusion_pose", rclcpp::QoS(2).best_effort());

  if (ptr_config_->is_debug) {
    pub_cloud_thread_ = std::thread(std::bind(&GRSLAM::publishThread, this));
  }
}

void GRSLAM::resetDataCallback() {
  sub_laser_cloud_.reset();
  sub_laser_scan_.reset();
  sub_odom_.reset();
  sub_imu_.reset();
  sub_uwb_muti_.reset();
  sub_uwb_single_.reset();
  sub_gps_.reset();

  is_first_odom_ = true;
  is_first_uwb_ = true;
  is_first_gps_ = true;
  is_first_cloud_ = true;
  odom_count_ = 0;
  uwb_count_ = 0;
  gps_count_ = 0;
  lidar_count_ = 0;
  imu_count_ = 0;
  LOG(INFO) << "reset all data callback.";
}

void GRSLAM::registerDataCallback() {
  if ((ptr_config_->use_imu && nullptr == sub_imu_) ||
      (ptr_config_->use_odom && nullptr == sub_odom_) ||
      (ptr_config_->use_lidar && nullptr == sub_laser_cloud_)) {
    LOG(INFO) << "...............register data call_back..................";
    is_first_odom_ = true;
    is_first_cloud_ = true;
    is_first_imu_ = true;
    odom_count_ = 0;
    lidar_count_ = 0;
    imu_count_ = 0;

    if (ptr_config_->use_lidar) {
      sub_laser_cloud_ = gr_slam_node_->create_subscription<sensor_msgs::msg::PointCloud2>(
          ptr_config_->cloud_topic_name, rclcpp::QoS(10).best_effort(),
          std::bind(&GRSLAM::laserCloudDataCallback, this, std::placeholders::_1));
      sub_laser_scan_ = gr_slam_node_->create_subscription<sensor_msgs::msg::LaserScan>(
          ptr_config_->scan_topic_name, rclcpp::QoS(10).best_effort(),
          std::bind(&GRSLAM::laserScanDataCallback, this, std::placeholders::_1));
    }

    if (ptr_config_->use_odom) {
      sub_odom_ = gr_slam_node_->create_subscription<nav_msgs::msg::Odometry>(
          ptr_config_->wheel_odom_topic_name, rclcpp::QoS(50).best_effort(),
          std::bind(&GRSLAM::odomDataCallback, this, std::placeholders::_1));
    }

    if (ptr_config_->use_imu) {
      sub_imu_ = gr_slam_node_->create_subscription<sensor_msgs::msg::Imu>(
          ptr_config_->imu_topic_name, rclcpp::QoS(200).best_effort(),
          std::bind(&GRSLAM::IMUDataCallback, this, std::placeholders::_1));
    }
  }

  // 如果配置数据集模式，则读取数据集，并将数据添加到系统数据队列；
  if (ptr_config_->dataset_mode == 1) {
    read_data_thread_ = std::thread(std::bind(&GRSLAM::readDataThread, this));
  }
}

void GRSLAM::laserCloudDataCallback(sensor_msgs::msg::PointCloud2::SharedPtr cloud_msg) {
  float laser_min_range = ptr_config_->laser_min_range;
  float laser_max_range = ptr_config_->laser_max_range;
  float laser_min_height = ptr_config_->laser_min_height;
  float laser_max_height = ptr_config_->laser_max_height;

  lidar_count_++;

  double time_stamp_now =
      double(cloud_msg->header.stamp.sec) + double(cloud_msg->header.stamp.nanosec) * (1.0 * 1e-9);

  pcl::PointCloud<XYZIPointType>::Ptr lidar_cloud_raw(new pcl::PointCloud<XYZIPointType>());
  pcl::fromROSMsg(*cloud_msg, *lidar_cloud_raw);

  XYZIPointType p_raw;
  PointType p_new;
  pcl::PointCloud<PointType>::Ptr new_cloud(new pcl::PointCloud<PointType>());

  int size = lidar_cloud_raw->size();
  for (int i = 0; i < size; i++) {
    p_raw = lidar_cloud_raw->points[i];
    p_new.x = p_raw.x;
    p_new.y = p_raw.y;
    p_new.z = p_raw.z;
    p_new.intensity = p_raw.intensity;
    new_cloud->push_back(p_new);
  }

  ptr_slam_system_->addCloudFrame(new_cloud, time_stamp_now);
}

void GRSLAM::laserScanDataCallback(const sensor_msgs::msg::LaserScan::SharedPtr laserScanMsg) {
  float laser_min_range = ptr_config_->laser_min_range;
  float laser_max_range = ptr_config_->laser_max_range;

  lidar_count_++;

  double time_stamp_now = double(laserScanMsg->header.stamp.sec) +
                          double(laserScanMsg->header.stamp.nanosec) * (1.0 * 1e-9);

  PointType newPoint;
  newPoint.z = 0.0;
  double newPointAngle = 0.0;

  pcl::PointCloud<PointType>::Ptr lidar_cloud_raw(new pcl::PointCloud<PointType>());

  int beamNum = laserScanMsg->ranges.size();
  for (int i = 0; i < beamNum; i++) {
    if (std::isnan(laserScanMsg->ranges[i])) {
      continue;
    }
    if (laserScanMsg->ranges[i] < laser_min_range || laserScanMsg->ranges[i] > laser_max_range) {
      continue;
    }
    newPointAngle = laserScanMsg->angle_min + laserScanMsg->angle_increment * i;
    newPoint.x = laserScanMsg->ranges[i] * std::cos(newPointAngle);
    newPoint.y = laserScanMsg->ranges[i] * std::sin(newPointAngle);
    newPoint.intensity = laserScanMsg->intensities[i];
    newPoint.normal_x = laserScanMsg->ranges[i];
    lidar_cloud_raw->push_back(newPoint);
  }

  ptr_slam_system_->addCloudFrame(lidar_cloud_raw, time_stamp_now);
}

void GRSLAM::odomDataCallback(const nav_msgs::msg::Odometry::SharedPtr odom_msg) {
  static double last_odom_time = -1.0;
  static double _last_delta_time = 0.0;

  odom_count_++;

  Mat34d curr_raw_odom_pose = fromOdometryMsg(odom_msg);

  if (ptr_config_->plane_mode == true) {
    Mathbox::ToPlaneMode(curr_raw_odom_pose);
  }

  float linear = odom_msg->twist.twist.linear.x;
  float angular = odom_msg->twist.twist.angular.z;

  double time_stamp = odom_msg->header.stamp.sec + odom_msg->header.stamp.nanosec * (1.0 * 1e-9);

  if (is_first_odom_) {
    if (odom_count_ < 50) {
      T_wo_0_ = curr_raw_odom_pose;

      last_raw_odom_pose_ = curr_raw_odom_pose;
      last_odom_pose_ = curr_raw_odom_pose;
      first_odom_time_ = time_stamp;
      last_odom_time = time_stamp;
      is_first_odom_ = false;
    }
  }

  // 判断里程计是否出现异常值
  bool time_gap_flag = false;
  double delta_time = std::abs(time_stamp - last_odom_time);
  if (delta_time < 0.02) {
    delta_time = 0.02;  // 50hz=0.02s;
  }
  if (delta_time > 0.1) {
    time_gap_flag = true;
    LOG(ERROR) << "big delta time ocurr for odom : d_t: " << delta_time << " s";
  }

  Mat34d delta_odom = Mathbox::deltaPose34d(last_raw_odom_pose_, curr_raw_odom_pose);

  float delta_rot = Mathbox::rotation2rpy(delta_odom.block<3, 3>(0, 0)).norm();
  float delta_dis = delta_odom.block<3, 1>(0, 3).norm();
  if (delta_dis > 0.06 || delta_rot > 0.06) {
    LOG(ERROR) << "big delta pose ocurr for odom: d_dis: " << delta_dis << ", d_rot: " << delta_rot;

    bool wheel_reset_flag = false;
    float x_tmp = curr_raw_odom_pose(0, 3);
    float y_tmp = curr_raw_odom_pose(1, 3);
    if (std::abs(x_tmp) < 0.1 && std::abs(y_tmp) < 0.1) {
      wheel_reset_flag = true;
      LOG(ERROR) << "wheel odom reset !!! curr_raw_odom_pose: ";
      LOG(ERROR) << curr_raw_odom_pose;
    }

    float linear_temp = delta_dis / delta_time;
    float angular_temp = delta_rot / delta_time;
    // 只要瞬时速度异常，或者运动增量太大，则进行异常值剔除;
    if (delta_dis > 0.2 || delta_rot > 0.2 || wheel_reset_flag) {
      if (linear_temp > 2.0 || angular_temp > 2.0 || wheel_reset_flag) {
        float time_ratio = delta_time / _last_delta_time;
        Mat34d temp_delta_odom =
            Mathbox::Interp_SE3(Mathbox::Identity34(), last_delta_odom_pose_, time_ratio);
        delta_odom = temp_delta_odom;
        LOG(ERROR) << "linear: " << linear_temp << " ,angular: " << angular_temp;
        LOG(ERROR) << "delta_dis: " << delta_dis << " ,delta_rot: " << delta_rot;
        LOG(ERROR) << "delta odom abnormal, using constant motion modle !!!";
        LOG(ERROR) << "time_ratio: " << time_ratio;
        LOG(ERROR) << "last_delta_odom: " << std::endl << last_delta_odom_pose_;
        LOG(ERROR) << "temp_delta_odom: " << std::endl << temp_delta_odom;
      }
    }
  }

  Mat34d new_odom_pose = Mathbox::multiplePose34d(last_odom_pose_, delta_odom);

  Eigen::Quaterniond q_tmp(new_odom_pose.block<3, 3>(0, 0));
  q_tmp.normalize();
  new_odom_pose.block<3, 3>(0, 0) = q_tmp.toRotationMatrix();

  if (ptr_config_->plane_mode == true) {
    Mathbox::ToPlaneMode(new_odom_pose);
  }

  curr_odom_time_ = time_stamp;

  // 将里程计坐标系下的里程计结果通过 外参 转化到激光雷达坐标系；
  // 地图坐标系以激光雷达为原点
  T_wo_ = transformOdom(new_odom_pose);

  // 添加当前帧里程计结果，并利用map_to_odom_和最新的里程计结果对机器人当前状态进行预测；
  ptr_slam_system_->addOdom(OdomMeasure(T_wo_, linear, angular, time_stamp));

  last_odom_pose_ = new_odom_pose;
  last_raw_odom_pose_ = curr_raw_odom_pose;
  last_delta_odom_pose_ = delta_odom;
  last_odom_time = time_stamp;
  _last_delta_time = delta_time;

  RobotState robot_state = ptr_slam_system_->getRobotState();
  Mat34d predict_cur_frame_pose = robot_state.pose;
  float cov = ptr_slam_system_->getPoseCov();

  publishLocalizationResult(robot_state.pose, time_stamp, cov);
}

void GRSLAM::IMUDataCallback(const sensor_msgs::msg::Imu::SharedPtr ImuMsg) {
  static double G_inv = 1.0 / 9.81;
  static Eigen::Vector3d _acc;
  static Eigen::Vector3d _gyr;
  static TicToc imu_receive_time;
  Eigen::Matrix3d R_li = ptr_config_->T_li.block<3, 3>(0, 0);

  float delta_receive_time = imu_receive_time.toc();
  if (delta_receive_time > 100.0) {
    LOG(WARNING) << "big delta time of imu: " << delta_receive_time << " ms";
  }
  imu_receive_time.tic();

  imu_count_++;

  double time_stamp = ImuMsg->header.stamp.sec + ImuMsg->header.stamp.nanosec * (1.0 * 1e-9);
  _acc = Eigen::Vector3d(ImuMsg->linear_acceleration.x, ImuMsg->linear_acceleration.y,
                         ImuMsg->linear_acceleration.z);
  _gyr = Eigen::Vector3d(ImuMsg->angular_velocity.x, ImuMsg->angular_velocity.y,
                         ImuMsg->angular_velocity.z);

  if (std::abs(_acc.z()) > 2.0) {
    _acc *= G_inv;
  }

  // convert imu to lidar coordinate
  _acc = R_li * _acc;
  _gyr = R_li * _gyr;

  ptr_slam_system_->addIMU(ImuMeasure(_acc, _gyr, time_stamp));

  if (ptr_config_->is_debug) {
    ImuMeasure cur_imu = ptr_slam_system_->getCurIMU();
    Eigen::Vector3d acc = cur_imu.acc_smooth;
    Eigen::Vector3d gyr = cur_imu.gyr_smooth;
    Eigen::Quaterniond Q = cur_imu.Q_wi;

    Vec3d rpy = Mathbox::rotation2rpy(cur_imu.Q_wi_smooth.toRotationMatrix());
    // Vec3d rpy = Mathbox::rotation2rpy(cur_imu.Q_wi_gyr.toRotationMatrix());
    rpy *= Rad2Deg;
    nav_msgs::msg::Odometry debug_msg;
    debug_msg.header = ImuMsg->header;
    debug_msg.pose.pose.position.x = rpy.x();
    debug_msg.pose.pose.position.y = rpy.y();
    debug_msg.pose.pose.position.z = rpy.z();
    debug_msg.pose.pose.orientation.x = Q.x();
    debug_msg.pose.pose.orientation.y = Q.y();
    debug_msg.pose.pose.orientation.z = Q.z();
    debug_msg.pose.pose.orientation.w = Q.w();
    debug_msg.twist.twist.linear.x = acc.x();
    debug_msg.twist.twist.linear.y = acc.y();
    debug_msg.twist.twist.linear.z = acc.z();
    debug_msg.twist.twist.angular.x = gyr.x();
    debug_msg.twist.twist.angular.y = gyr.y();
    debug_msg.twist.twist.angular.z = gyr.z();

    pub_debug_data_->publish(debug_msg);
  }
}

void GRSLAM::readDataThread() {
  while (rclcpp::ok()) {
    // 如果配置数据集模式，则读取数据集，并将数据添加到系统数据队列；
    if (ptr_config_->dataset_mode == 1) {
      std::pair<float, laserCloud::Ptr> lidar_frame =
          readKittiSequence(ptr_config_->dataset_path, ptr_config_->kitti_sequence);

      if (lidar_frame.second->size() > 0) {
        ptr_slam_system_->addCloudFrame(lidar_frame.second, lidar_frame.first);
        lidar_count_++;

        // 处理完再读取下一帧；
        while (ptr_slam_system_->getCurKeyFrameTimeStamp() < lidar_frame.first - 0.001) {
          std::this_thread::sleep_for(std::chrono::milliseconds(5));
        }
      } else {
        LOG(ERROR) << "read kitti dataset file failed !!!";
        std::this_thread::sleep_for(std::chrono::milliseconds(1000));
      }
    }

    std::this_thread::sleep_for(std::chrono::milliseconds(5));
  }
}

void GRSLAM::publishThread() {
  static Mat34d init_map_to_odom;
  static double _last_key_time = 0.0;

  while (rclcpp::ok()) {
    if (ptr_slam_system_ == nullptr) {
      std::this_thread::sleep_for(std::chrono::milliseconds(100));
      continue;
    }

    if (!ptr_config_->is_debug) {
      std::this_thread::sleep_for(std::chrono::milliseconds(20));
      continue;
    }

    double curr_key_time = ptr_slam_system_->getCurKeyFrameTimeStamp();
    if (curr_key_time - _last_key_time < 0.001) {
      std::this_thread::sleep_for(std::chrono::milliseconds(20));
      continue;
    }
    _last_key_time = curr_key_time;

    rclcpp::Time now_time(curr_key_time * 1e9);

    Mat34d curr_key_pose = ptr_slam_system_->getKeyFramePose();
    Mat34d curr_lidar_odom_pose = ptr_slam_system_->getKeyFrameLidarOdomPose();

    if (ptr_config_->publish_path) {
      Mat34d T_wo = ptr_slam_system_->getKeyFrameWheelOdomPose();
      publishPath(curr_key_time, curr_key_pose, curr_lidar_odom_pose, T_wo);
    }

    if (ptr_config_->publish_tf) {
      publishTf(curr_key_pose, curr_key_time);
    }

    if (ptr_config_->publish_cloud) {
      sensor_msgs::msg::PointCloud2 cornerMsg, localMapMsg, keyPoseMsg;

      laserCloud::Ptr curr_surf = ptr_slam_system_->getCurKeyFrameSurf();
      if (curr_surf->size() > 0) {
        pcl::toROSMsg(*curr_surf, cornerMsg);
        cornerMsg.header.stamp = now_time;
        cornerMsg.header.frame_id = ptr_config_->lidar_type;
        pub_surf_cloud_->publish(cornerMsg);
      }

      laserCloud::Ptr key_pose_cloud = ptr_slam_system_->getKeyPoseCloud();
      if (key_pose_cloud->size() > 0) {
        pcl::toROSMsg(*key_pose_cloud, keyPoseMsg);
        keyPoseMsg.header.stamp = now_time;
        keyPoseMsg.header.frame_id = "/map";
        pub_pose_cloud_->publish(keyPoseMsg);
      }

      static double last_localmap_pub_time = 0.0;
      if (std::abs(curr_key_time - last_localmap_pub_time) > 1.0) {
        laserCloud::Ptr surroud_cloud = ptr_slam_system_->getCurrSurroundCloud();
        pcl::toROSMsg(*surroud_cloud, localMapMsg);
        localMapMsg.header.stamp = now_time;
        localMapMsg.header.frame_id = "/map";
        pub_surroud_cloud_->publish(localMapMsg);

        last_localmap_pub_time = curr_key_time;
      }
    }

    std::this_thread::sleep_for(std::chrono::milliseconds(20));
  }
}

void GRSLAM::publishPath(double time, const Mat34d &slam_pose, const Mat34d &lidar_odom_pose,
                         const Mat34d &wheel_pose) {
  rclcpp::Time now_time(time * 1e9);

  geometry_msgs::msg::PoseStamped slamPose;
  slamPose.pose = toGeometryMsg(slam_pose);
  key_pose_path_.header.stamp = now_time;
  key_pose_path_.header.frame_id = "map";
  key_pose_path_.poses.push_back(slamPose);
  pub_key_pose_path_->publish(key_pose_path_);

  geometry_msgs::msg::PoseStamped lidarOdomPose;
  lidarOdomPose.pose = toGeometryMsg(lidar_odom_pose);
  lidar_odom_path_.header.stamp = now_time;
  lidar_odom_path_.header.frame_id = "map";
  lidar_odom_path_.poses.push_back(lidarOdomPose);
  pub_lidar_odom_path_->publish(lidar_odom_path_);

  if (ptr_config_->use_odom) {
    geometry_msgs::msg::PoseStamped odomPose;
    odomPose.pose = toGeometryMsg(wheel_pose);
    wheel_odom_path_.header.stamp = now_time;
    wheel_odom_path_.header.frame_id = "map";
    wheel_odom_path_.poses.push_back(odomPose);
    pub_wheel_odom_path_->publish(wheel_odom_path_);
  }
}

void GRSLAM::publishTf(const Mat34d &pose, double time) {
  rclcpp::Time now_time(time * 1e9);

  geometry_msgs::msg::TransformStamped tmp_tf_stamped;
  Eigen::Quaterniond geoQuat(pose.block<3, 3>(0, 0));
  tf2::Transform tf_map_laser;
  tf2::Quaternion q(geoQuat.x(), geoQuat.y(), geoQuat.z(), geoQuat.w());
  tf_map_laser = tf2::Transform(q, tf2::Vector3(pose(0, 3), pose(1, 3), pose(2, 3)));
  tmp_tf_stamped.header.frame_id = "map";
  tmp_tf_stamped.child_frame_id = ptr_config_->lidar_type;
  tmp_tf_stamped.header.stamp = now_time;
  tmp_tf_stamped.transform = tf2::toMsg(tf_map_laser);
  ptr_tfl_->sendTransform(tmp_tf_stamped);

  static Eigen::Quaterniond q_lo(ptr_config_->T_lo.block<3, 3>(0, 0));
  static tf2::Quaternion q_lo_tf(q_lo.x(), q_lo.y(), q_lo.z(), q_lo.w());
  static tf2::Transform tf_laser_baselink(
      q_lo_tf,
      tf2::Vector3(ptr_config_->T_lo(0, 3), ptr_config_->T_lo(1, 3), ptr_config_->T_lo(2, 3)));
  tmp_tf_stamped.header.frame_id = ptr_config_->lidar_type;
  tmp_tf_stamped.child_frame_id = "base_link";
  tmp_tf_stamped.header.stamp = now_time;
  tmp_tf_stamped.transform = tf2::toMsg(tf_laser_baselink);
  ptr_tfb_->sendTransform(tmp_tf_stamped);
}

bool GRSLAM::publishLocalizationResult(const Mat34d &pose, double time, const double cov) {
  Mat34d transform_in_base = Mathbox::multiplePose34d(pose, ptr_config_->T_lo);
  geometry_msgs::msg::Pose base_pose = toGeometryMsg(transform_in_base);
  geometry_msgs::msg::PoseWithCovarianceStamped pose_msgs;

  rclcpp::Time now_time(time * 1e9);
  pose_msgs.header.stamp = now_time;
  pose_msgs.header.frame_id = "map";
  pose_msgs.pose.pose.position.x = base_pose.position.x;
  pose_msgs.pose.pose.position.y = base_pose.position.y;
  pose_msgs.pose.pose.position.z = base_pose.position.z;
  pose_msgs.pose.pose.orientation.x = base_pose.orientation.x;
  pose_msgs.pose.pose.orientation.y = base_pose.orientation.y;
  pose_msgs.pose.pose.orientation.z = base_pose.orientation.z;
  pose_msgs.pose.pose.orientation.w = base_pose.orientation.w;
  pose_msgs.pose.covariance[0] = cov;

  pub_pose_->publish(pose_msgs);

  return true;
}

laserCloud::Ptr GRSLAM::removeNanAndNoisePoints(const laserCloud::Ptr cloud_in) {
  float laser_min_range = ptr_config_->laser_min_range;
  float laser_max_range = ptr_config_->laser_max_range;
  float laser_min_height = ptr_config_->laser_min_height;
  float laser_max_height = ptr_config_->laser_max_height;

  PointType p_r, p_new;
  float range = 0.0;
  laserCloud::Ptr cloud_out(new laserCloud());

  int size = cloud_in->size();
  for (int i = 0; i < size; i++) {
    p_r = cloud_in->points[i];

    if (std::isnan(p_r.x) || std::isnan(p_r.y) || std::isnan(p_r.z) || std::isnan(p_r.intensity)) {
      continue;
    }

    range = std::sqrt(p_r.x * p_r.x + p_r.y * p_r.y + p_r.z * p_r.z);
    if (range > laser_max_range || range < laser_min_range || p_r.z > laser_max_height ||
        p_r.z < laser_min_height) {
      continue;
    }

    p_new.x = p_r.x;
    p_new.y = p_r.y;
    p_new.z = p_r.z;
    p_new.intensity = p_r.intensity;
    p_new.normal_x = range;

    cloud_out->push_back(p_new);
  }
  return cloud_out;
}

}  // namespace GR_SLAM

void initGoogleLog(const std::string &module, const std::string &log_level) {
  if (log_level.empty()) {
    std::cout << "please set log level.(error/warn/info)" << std::endl;
    return;
  }
  FLAGS_colorlogtostderr = true;

  google::InitGoogleLogging(module.c_str());
  google::InstallFailureSignalHandler();

  std::cout << "glog level: " << log_level << std::endl;
  if (!log_level.compare("error")) {
    google::SetStderrLogging(google::ERROR);
  } else if (!log_level.compare("info")) {
    google::SetStderrLogging(google::INFO);
  } else if (!log_level.compare("warn")) {
    google::SetStderrLogging(google::WARNING);
  } else if (!log_level.compare("fatal")) {
    google::SetStderrLogging(google::FATAL);
  } else {
    std::cout << "error log mode. (error/warn/info/fatal)" << std::endl;
  }

  std::string LogHomeDir = "log_info/";

  char current_path[500] = {'\0'};
  if (getcwd(current_path, 500) == NULL) {
    LOG(FATAL) << "get pwd fail!!";
  }

  const char *tmp = getenv("USERDATA_PATH");
  std::string bash_path = "/userdata/";
  if (access(bash_path.c_str(), F_OK) == 0) {
    // std::string path = tmp;
    LogHomeDir = bash_path + LogHomeDir;
  } else if (strncmp(current_path, "/tmp", 4) == 0) {  // appimage部署方式
    struct passwd *pw;
    pw = getpwuid(getuid());
    std::string data_folder_path =
        std::string("/home/") + std::string(pw->pw_name) + std::string("/Development/");

    if (access(data_folder_path.c_str(), F_OK) != 0) {
      if (mkdir(data_folder_path.c_str(), S_IRWXU) != 0) {
        LOG(FATAL) << "mkdir " << data_folder_path << " fail!!";
      }
    }

    LogHomeDir = data_folder_path + LogHomeDir;
  }

  if (access(LogHomeDir.c_str(), F_OK)) {
    std::cerr << "loghomedir not exist, will create!" << std::endl;
    mkdir(LogHomeDir.c_str(), S_IRWXU | S_IRWXG | S_IROTH | S_IXOTH);
  }

  const std::string LogDir = LogHomeDir + module + "/";
  if (access(LogDir.c_str(), F_OK)) {
    std::cerr << "logdir not exist, will create!" << std::endl;
    mkdir(LogDir.c_str(), S_IRWXU | S_IRWXG | S_IROTH | S_IXOTH);
  }

  const std::string infoLogDir = LogHomeDir + module + "/info/";
  if (access(infoLogDir.c_str(), F_OK)) {
    std::cerr << "infologdir not exist, will create!" << std::endl;
    mkdir(infoLogDir.c_str(), S_IRWXU | S_IRWXG | S_IROTH | S_IXOTH);
  }

  const std::string warnLogDir = LogHomeDir + module + "/warn/";
  if (access(warnLogDir.c_str(), F_OK)) {
    std::cerr << "warnlogdir not exist, will create!" << std::endl;
    mkdir(warnLogDir.c_str(), S_IRWXU | S_IRWXG | S_IROTH | S_IXOTH);
  }

  const std::string errorLogDir = LogHomeDir + module + "/error/";
  if (access(errorLogDir.c_str(), F_OK)) {
    std::cerr << "errorlogdir not exist, will create!" << std::endl;
    mkdir(errorLogDir.c_str(), S_IRWXU | S_IRWXG | S_IROTH | S_IXOTH);
  }

  const std::string fatalLogDir = LogHomeDir + module + "/fatal/";
  if (access(fatalLogDir.c_str(), F_OK)) {
    std::cerr << "fatallogdir not exist, will create!" << std::endl;
    mkdir(fatalLogDir.c_str(), S_IRWXU | S_IRWXG | S_IROTH | S_IXOTH);
  }

  google::SetLogDestination(google::GLOG_INFO, infoLogDir.c_str());
  google::SetLogDestination(google::GLOG_WARNING, warnLogDir.c_str());
  google::SetLogDestination(google::GLOG_ERROR, errorLogDir.c_str());
  google::SetLogDestination(google::GLOG_FATAL, fatalLogDir.c_str());

  FLAGS_max_log_size = 50;  // 单个日志文件大小上限（MB）, 如果设置为0将默认为1
  FLAGS_logbufsecs = 0;  //设置可以缓冲日志的最大秒数,glog默认30秒,0指实时输出
}

int main(int argc, char **argv) {
  using namespace GR_SLAM;
  rclcpp::init(argc, argv);
  initGoogleLog("grslam", "warn");

  rclcpp::NodeOptions options;
  options.automatically_declare_parameters_from_overrides(true);

  rclcpp::Node::SharedPtr grslam_ros_node = std::make_shared<rclcpp::Node>("grslam", options);

  GRSLAM grslam(grslam_ros_node);

  rclcpp::executors::MultiThreadedExecutor exec;
  exec.add_node(grslam_ros_node);
  exec.spin();
  exec.remove_node(grslam_ros_node);

  rclcpp::shutdown();
  return 0;
}