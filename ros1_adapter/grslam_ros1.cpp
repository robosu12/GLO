#include "grslam_ros1.hpp"

namespace GR_SLAM {

GRSLAM::GRSLAM(ros::NodeHandle &ros_node, const std::string &config_path) {
  ros_node_ = ros_node;
  config_path_ = config_path;

  is_first_odom_ = true;
  is_first_uwb_ = true;
  is_first_gps_ = true;
  is_first_cloud_ = true;
  odom_count_ = 0;
  uwb_count_ = 0;
  gps_count_ = 0;
  lidar_count_ = 0;
  imu_count_ = 0;

  ptr_slam_system_ = System::getInstance();
}

GRSLAM::~GRSLAM() {
  LOG(ERROR) << "SENSOR COUNT " << odom_count_ << ' ' << lidar_count_;
  ptr_slam_system_->shutdown();

  LOG(ERROR) << "system shutdown !!!";
}

void GRSLAM::init() {
  LOG(ERROR) << "GRSLAM begin init ...";

  ptr_slam_system_->Init(config_path_);

  ptr_config_ = ptr_slam_system_->getConfig();

  registerDataPublisher();

  registerDataCallback();

  LOG(ERROR) << "GRSLAM finish init !!!";

  LOG(WARNING) << "waiting for sensors data ......";
}

void GRSLAM::registerDataPublisher() {
  if (ptr_config_->publish_cloud) {
    pub_surf_cloud_ = ros_node_.advertise<sensor_msgs::PointCloud2>("/surf_cloud", 1);
    pub_surroud_cloud_ = ros_node_.advertise<sensor_msgs::PointCloud2>("/surround_map_cloud", 1);
    pub_global_cloud_ = ros_node_.advertise<sensor_msgs::PointCloud2>("/global_map_cloud", 1);
    pub_pose_cloud_ = ros_node_.advertise<sensor_msgs::PointCloud2>("/frame_pose_cloud", 1);
    pub_lidar_pose_ = ros_node_.advertise<geometry_msgs::PoseStamped>("/lidar_pose", 1);
  }

  if (ptr_config_->publish_path) {
    pub_key_pose_path_ = ros_node_.advertise<nav_msgs::Path>("/key_pose_path", 1);
    pub_lidar_odom_path_ = ros_node_.advertise<nav_msgs::Path>("/lidar_odom_path", 1);
    pub_wheel_odom_path_ = ros_node_.advertise<nav_msgs::Path>("/wheel_odom_path", 1);
  }

  if (ptr_config_->is_debug) {
    pub_debug_data_ = ros_node_.advertise<nav_msgs::Odometry>("/debug_data", 1);
  }

  if (ptr_config_->is_debug) {
    pub_cloud_thread_ = std::thread(std::bind(&GRSLAM::publishThread, this));
  }
}

void GRSLAM::registerDataCallback() {
  LOG(INFO) << "...............register data call_back..................";
  is_first_odom_ = true;
  is_first_cloud_ = true;
  is_first_imu_ = true;
  odom_count_ = 0;
  lidar_count_ = 0;
  imu_count_ = 0;

  if (ptr_config_->use_lidar) {
    sub_laser_cloud_ = ros_node_.subscribe<sensor_msgs::PointCloud2>(
        ptr_config_->cloud_topic_name, 100, &GRSLAM::laserCloudDataCallback, this);
  }

  // 如果配置数据集模式，则读取数据集，并将数据添加到系统数据队列；
  if (ptr_config_->dataset_mode == 1) {
    read_data_thread_ = std::thread(std::bind(&GRSLAM::readDataThread, this));
  }
}

void GRSLAM::laserCloudDataCallback(const sensor_msgs::PointCloud2ConstPtr &cloud_msg) {
  lidar_count_++;

  double time_stamp_now = cloud_msg->header.stamp.toSec();

  pcl::PointCloud<XYZIPointType>::Ptr lidar_cloud_raw(new pcl::PointCloud<XYZIPointType>());
  pcl::fromROSMsg(*cloud_msg, *lidar_cloud_raw);

  XYZIPointType p_raw;
  PointType p_new;
  pcl::PointCloud<PointType>::Ptr new_cloud(new pcl::PointCloud<PointType>());

  u_int32_t size = lidar_cloud_raw->size();
  for (u_int32_t i = 0; i < size; i++) {
    p_raw = lidar_cloud_raw->points[i];
    p_new.x = p_raw.x;
    p_new.y = p_raw.y;
    p_new.z = p_raw.z;
    p_new.intensity = p_raw.intensity;
    new_cloud->push_back(p_new);
  }

  ptr_slam_system_->addCloudFrame(new_cloud, time_stamp_now);
}

void GRSLAM::readDataThread() {
  while (ros::ok()) {
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
        LOG(ERROR) << "read kitti dataset file failed, check your dataset_path in config file !!!";
        std::this_thread::sleep_for(std::chrono::milliseconds(1000));
      }
    }

    std::this_thread::sleep_for(std::chrono::milliseconds(5));
  }
}

void GRSLAM::publishThread() {
  static Mat34d init_map_to_odom;
  static double _last_key_time = 0.0;

  while (ros::ok()) {
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

    ros::Time now_time(curr_key_time);

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
      sensor_msgs::PointCloud2 cornerMsg, localMapMsg, keyPoseMsg;

      laserCloud::Ptr curr_surf = ptr_slam_system_->getCurKeyFrameSurf();
      if (curr_surf->size() > 0) {
        pcl::toROSMsg(*curr_surf, cornerMsg);
        cornerMsg.header.stamp = now_time;
        cornerMsg.header.frame_id = ptr_config_->lidar_type;
        pub_surf_cloud_.publish(cornerMsg);
      }

      laserCloud::Ptr key_pose_cloud = ptr_slam_system_->getKeyPoseCloud();
      if (key_pose_cloud->size() > 0) {
        pcl::toROSMsg(*key_pose_cloud, keyPoseMsg);
        keyPoseMsg.header.stamp = now_time;
        keyPoseMsg.header.frame_id = "map";
        pub_pose_cloud_.publish(keyPoseMsg);
      }

      static double last_localmap_pub_time = 0.0;
      if (std::abs(curr_key_time - last_localmap_pub_time) > 1.0) {
        laserCloud::Ptr surroud_cloud = ptr_slam_system_->getCurrSurroundCloud();
        pcl::toROSMsg(*surroud_cloud, localMapMsg);
        localMapMsg.header.stamp = now_time;
        localMapMsg.header.frame_id = "map";
        pub_surroud_cloud_.publish(localMapMsg);

        last_localmap_pub_time = curr_key_time;
      }
    }

    std::this_thread::sleep_for(std::chrono::milliseconds(20));
  }
}

void GRSLAM::publishPath(double time, const Mat34d &slam_pose, const Mat34d &lidar_odom_pose,
                         const Mat34d &wheel_pose) {
  ros::Time now_time(time);

  geometry_msgs::PoseStamped slamPose;
  slamPose.pose = toGeometryMsg(slam_pose);
  key_pose_path_.header.stamp = now_time;
  key_pose_path_.header.frame_id = "map";
  key_pose_path_.poses.push_back(slamPose);
  pub_key_pose_path_.publish(key_pose_path_);

  geometry_msgs::PoseStamped lidarOdomPose;
  lidarOdomPose.pose = toGeometryMsg(lidar_odom_pose);
  lidar_odom_path_.header.stamp = now_time;
  lidar_odom_path_.header.frame_id = "map";
  lidar_odom_path_.poses.push_back(lidarOdomPose);
  pub_lidar_odom_path_.publish(lidar_odom_path_);

  if (ptr_config_->use_odom) {
    geometry_msgs::PoseStamped odomPose;
    odomPose.pose = toGeometryMsg(wheel_pose);
    wheel_odom_path_.header.stamp = now_time;
    wheel_odom_path_.header.frame_id = "map";
    wheel_odom_path_.poses.push_back(odomPose);
    pub_wheel_odom_path_.publish(wheel_odom_path_);
  }

  // for NTU VIRAL dataset
  Mat34d T_lidar_leica = Mathbox::Identity34();
  T_lidar_leica.block<3, 1>(0, 3) = Eigen::Vector3d(-0.24, -0.01, -0.33);
  Mat34d T_w_leica = Mathbox::multiplePose34d(slam_pose, T_lidar_leica);
  slamPose.pose = toGeometryMsg(T_w_leica);
  slamPose.header.stamp = now_time;
  slamPose.header.frame_id = "map";
  pub_lidar_pose_.publish(slamPose);
}

void GRSLAM::publishTf(const Mat34d &pose, double time) {
  ros::Time now_time(time);

  static tf::TransformBroadcaster br;
  tf::Transform transform;
  transform.setOrigin(tf::Vector3(pose(0, 3), pose(1, 3), pose(2, 3)));
  Eigen::Quaterniond q_eigen(pose.block<3, 3>(0, 0));
  tf::Quaternion q;
  q.setW(q_eigen.w());
  q.setX(q_eigen.x());
  q.setY(q_eigen.y());
  q.setZ(q_eigen.z());
  transform.setRotation(q);
  br.sendTransform(tf::StampedTransform(transform, now_time, "map", ptr_config_->lidar_type));

  // // for NTU VIRAL dataset
  // static tf::Transform T_imu_leica, T_lidar_imu;
  // T_lidar_imu.setOrigin(tf::Vector3(0.050, 0.000, -0.055));
  // q.setW(1.0);
  // q.setX(0.0);
  // q.setY(0.0);
  // q.setZ(0.0);
  // T_lidar_imu.setRotation(q);
  // br.sendTransform(tf::StampedTransform(T_lidar_imu, now_time, ptr_config_->lidar_type, "imu"));

  // T_imu_leica.setOrigin(tf::Vector3(-0.293656, -0.012288, -0.273095));
  // q.setW(1.0);
  // q.setX(0.0);
  // q.setY(0.0);
  // q.setZ(0.0);
  // T_imu_leica.setRotation(q);
  // br.sendTransform(tf::StampedTransform(T_imu_leica, now_time, "imu", "/leica"));
}

laserCloud::Ptr GRSLAM::removeNanAndNoisePoints(const laserCloud::Ptr cloud_in) {
  float laser_min_range = ptr_config_->laser_min_range;
  float laser_max_range = ptr_config_->laser_max_range;
  float laser_min_height = ptr_config_->laser_min_height;
  float laser_max_height = ptr_config_->laser_max_height;

  PointType p_r, p_new;
  float range = 0.0;
  laserCloud::Ptr cloud_out(new laserCloud());

  u_int32_t size = cloud_in->size();
  for (u_int32_t i = 0; i < size; i++) {
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
  if (argc < 2) {
    LOG(ERROR) << "Usage: " << argv[0] << " config_path ";
    LOG(ERROR) << "please input config file path ! ";
    return 1;
  }

  using namespace GR_SLAM;

  ros::init(argc, argv, "grslam");

  initGoogleLog("grslam", "warn");

  ros::NodeHandle ros1_node;

  const std::string config_file_path = std::string(argv[1]);

  GRSLAM grslam(ros1_node, config_file_path);

  grslam.init();

  ros::spin();

  return 0;
}