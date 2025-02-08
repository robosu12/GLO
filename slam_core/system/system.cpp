
#include "system.hpp"
#include <algorithm>
#include "common/debug_tools/debug_color.h"
#include "frontend/feature_extract_factory.hpp"
#include "malloc.h"
#include "common/kitti.hpp"

namespace GR_SLAM {

System::System() {
  LOG(WARNING) << "GRODOM version: V1.0 - 2024.08.05 !!!";
}

System::~System() {}

System *System::getInstance() {
  static System lidar_slam_system;
  return &lidar_slam_system;
}

void System::Init(const std::string &config_path) {
  ptr_load_params_ = std::make_shared<LoadParams>();
  ptr_load_params_->loadSystemConfig(config_path);
  ptr_config_ = ptr_load_params_->ptr_config_;

  is_stopped_ = false;
  is_first_cloud_ = true;
  is_first_odom_ = true;
  is_first_keyframe_ = true;
  new_keyframe_flag_ = false;
  id_ = 0;

  d_odom_measures_.clear();
  d_imu_measures_.clear();
  d_cloud_measures_.clear();
  odom_count_ = 0;
  imu_count_ = 0;
  lidar_count_ = 0;
  init_map_to_odom_ = Mathbox::Identity34();
  last_map_to_odom_ = Mathbox::Identity34();
  cur_map_to_odom_ = Mathbox::Identity34();
  lidar_odom_to_odom_ = Mathbox::Identity34();
  init_imu_pose_ = Mathbox::Identity34();

  curr_correspond_odom_pose_ = Mathbox::Identity34();
  last_correspond_odom_pose_ = Mathbox::Identity34();
  curr_lidar_odom_pose_ = Mathbox::Identity34();
  last_lidar_odom_pose_ = Mathbox::Identity34();
  cur_robot_state_.Reset();

  surf_cloud_.reset(new laserCloud());
  occ_cloud_.reset(new laserCloud());
  key_pose_cloud_.reset(new laserCloud());

  ptr_feature_extractor_ = FeatureExtractorFactory::creatExtractor(ptr_config_);
  ptr_lidar_odom_ = std::make_shared<LidarOdom>(ptr_config_);
  ptr_map_track_ = std::make_shared<MapTrack>(ptr_config_);

  ptr_lidarOdom_thread_ = new std::thread(&System::lidarOdomThread, this);
  ptr_maptrack_thread_ = new std::thread(&System::mapTrackThread, this);

  if (ptr_config_->save_keyPose_to_file || ptr_config_->save_runinfo_to_file) {
    size_t pos = config_path.find("src");
    if (pos != std::string::npos) {
      result_path_ = config_path.substr(0, pos);
    } else {
      LOG(ERROR) << ">>>>>>>> wrong config file path !!!";
      return;
    }

    if (ptr_config_->save_keyPose_to_file) {
      lidar_odom_pose_file_.open(result_path_ + "lidar_odom_pose_file.txt");
      lidar_key_pose_file_.open(result_path_ + "lidar_key_pose_file.txt");
    }

    if (ptr_config_->save_runinfo_to_file) {
      runinfo_result_.open(result_path_ + "runinfo_result.txt");
    }
  }
}

void System::lidarOdomThread() {
  while (!is_stopped_) {
    if (new_keyframe_flag_ == true) {
      std::this_thread::sleep_for(std::chrono::milliseconds(5));
      continue;
    }

    // 1. 创建关键帧: 提取特征；状态预测；
    bool creatFrame_flag = creatFrame();
    if (creatFrame_flag == false) {
      std::this_thread::sleep_for(std::chrono::milliseconds(5));
      continue;
    }

    // 2. 进行地图匹配和状态更新；
    static TicToc processKeyFrame_cost;
    processKeyFrame_cost.tic();
    bool match_flag =
        ptr_lidar_odom_->processNewKeyFrame(ptr_cur_lidar_odom_keyframe_, is_match_ok_);

    lidar_odom_to_odom_ = ptr_lidar_odom_->getMap2Odom();

    if (match_flag == true || is_first_keyframe_) {
      new_keyframe_flag_ = true;
    }

    LOG(INFO) << "lidarOdomThread - processKeyFrame_time: " << processKeyFrame_cost.toc()
              << ", ave_time: " << processKeyFrame_cost.getAveTime();
    LOG(INFO);
  }
}

void System::mapTrackThread() {
  while (!is_stopped_) {
    if (new_keyframe_flag_ == true) {
      ptr_cur_keyframe_ = ptr_lidar_odom_->getKeyframeMaptrack();
      if (ptr_cur_keyframe_ == nullptr) {
        LOG(WARNING) << "mapTrackThread: ptr_cur_keyframe_ == nullptr !";
        new_keyframe_flag_ = false;
        continue;
      }

      // use lidar_odom to predict cur pose;
      Mat34d predict_cur_frame_pose =
          Mathbox::multiplePose34d(cur_map_to_odom_, ptr_cur_keyframe_->getLaserOdomPose());

      ptr_cur_keyframe_->updatePose(predict_cur_frame_pose);

      new_keyframe_flag_ = false;
    } else {
      std::this_thread::sleep_for(std::chrono::milliseconds(5));
      continue;
    }

    // 进行地图匹配优化,更新当前位姿；
    static TicToc processKeyFrame_time;
    processKeyFrame_time.tic();
    is_match_ok_ = ptr_map_track_->processNewKeyFrame(ptr_cur_keyframe_, false);

    // 获取匹配结果；
    cur_map_to_odom_ = ptr_map_track_->getMap2Odom();

    key_pose_cloud_->emplace_back(ptr_cur_keyframe_->getPosition());

    float process_time = processKeyFrame_time.toc();
    float ave_time = processKeyFrame_time.getAveTime();
    float cpu_usage = cpu_metric.usage();
    float mem_usage = MemoryMetric::instanceCPU().usage() / 1024.0;

    // 保存cpu、内存、耗时等运行数据到文件；
    if (ptr_config_->save_runinfo_to_file) {
      if (is_first_keyframe_) {
        runinfo_result_ << "#format: cpu_usage mem_usage process_time ave_time" << std::endl;
      }
      runinfo_result_ << std::setprecision(5) << cpu_usage << ' ' << mem_usage << ' '
                      << process_time << ' ' << ave_time << std::endl;
    }

    // 保存pose到文件；
    if (ptr_config_->save_keyPose_to_file) {
      saveCurPoseToFile();
    }

    if (is_first_keyframe_) {
      is_first_keyframe_ = false;
      LOG(WARNING) << "****** mapTrackThread: first_keyframe *******";
    }

    LOG(INFO) << "mapTrackThread - processKeyFrame_time: " << process_time
              << ", ave_time: " << ave_time << ", now_cpu: " << cpu_usage
              << ", now_memory: " << mem_usage << " MB";
    LOG(INFO);
  }
}

bool System::creatFrame() {
  static int create_count = 0;
  CloudMeasure cur_cloud, next_cloud;
  OdomMeasure cur_odom;
  ImuMeasure cur_imu;

  laserCloud::Ptr input_cloud(new laserCloud());
  float curr_linear = 0.0, curr_angular = 0.0;
  std::vector<OdomMeasure> v_odom;
  std::vector<ImuMeasure> v_imu;
  std::vector<ImageMeasure> v_image;

  static Eigen::Quaterniond q_tmp;
  static Mat34d predict_lidar_odom_pose, predict_cur_frame_pose;
  static Mat34d delta_odom = Mathbox::Identity34();

  create_count++;

  static TicToc create_frame_cost;
  create_frame_cost.tic();

  {
    std::lock_guard<std::mutex> lock_cloud(cloud_data_mutex_);
    std::lock_guard<std::mutex> lock_odom(odom_data_mutex_);
    std::lock_guard<std::mutex> lock_imu(imu_data_mutex_);

    // cache cloud data to ensure imu and odom data is filled;
    if (ptr_config_->use_lidar && d_cloud_measures_.size() < 1) {
      return false;
    }
    if (ptr_config_->use_odom && d_odom_measures_.size() < 15) {
      return false;
    }
    if (ptr_config_->use_imu && d_imu_measures_.size() < 60) {
      return false;
    }

    if (d_cloud_measures_.size() > 10 && create_count % 40 == 0) {
      LOG(WARNING) << "cloud buffer size: " << d_cloud_measures_.size();
    }
    if (d_odom_measures_.size() > 50 && create_count % 40 == 0) {
      LOG(WARNING) << "odom buffer size: " << d_odom_measures_.size();
    }
    if (d_imu_measures_.size() > 200 && create_count % 40 == 0) {
      LOG(WARNING) << "imu buffer size: " << d_imu_measures_.size();
    }

    // extract lidar measurments;
    if (ptr_config_->use_lidar) {
      cur_cloud = d_cloud_measures_[0];
      *input_cloud = *cur_cloud.cloud;

      last_frame_time_ = temp_frame_time_;
      temp_frame_time_ = cur_cloud.time_stamp;
    }

    // throw old cloud
    d_cloud_measures_.pop_front();
  }

  // 运动距离或者角度超过设定值才进行优化；
  if (id_ < 20 || !ptr_lidar_odom_->isJunkFrame(temp_frame_time_, curr_correspond_odom_pose_)) {
    if (!ptr_config_->use_imu && !ptr_config_->use_odom && ptr_config_->use_lidar) {
      if (ptr_cur_lidar_odom_keyframe_ != nullptr) {
        curr_lidar_odom_pose_ = ptr_cur_lidar_odom_keyframe_->getLaserOdomPose();
      }

      delta_odom = Mathbox::deltaPose34d(last_lidar_odom_pose_, curr_lidar_odom_pose_);

      if (is_first_keyframe_) {
        delta_odom = Mathbox::Identity34();
        // for NUT dataset
        if (ptr_config_->dataset_mode == 2) {
          curr_lidar_odom_pose_.block<3, 3>(0, 0) = Mathbox::rpyToRotationMatrix(Vec3d(3.14, 0, 0));
        }
      }

      predict_lidar_odom_pose = Mathbox::multiplePose34d(curr_lidar_odom_pose_, delta_odom);

      last_lidar_odom_pose_ = curr_lidar_odom_pose_;
    }

    q_tmp = predict_lidar_odom_pose.block<3, 3>(0, 0);
    q_tmp.normalize();
    predict_lidar_odom_pose.block<3, 3>(0, 0) = q_tmp.toRotationMatrix();

    predict_cur_frame_pose = Mathbox::multiplePose34d(cur_map_to_odom_, predict_lidar_odom_pose);

    surf_cloud_->clear();
    occ_cloud_->clear();

    if (!ptr_feature_extractor_->setInputCloud(input_cloud)) {
      LOG(ERROR) << "ptr_feature_extractor_->setInputCloud failed !!!";
      return false;
    }

    if ((ptr_config_->lidar_undistortion == 1 && ptr_config_->use_imu) ||
        (ptr_config_->lidar_undistortion == 2 && ptr_config_->use_odom) ||
        (ptr_config_->lidar_undistortion == 3 && ptr_config_->use_lidar)) {
      // 根据激光帧间增量进行点云畸变去除，转换到起始时间点下；（livox的时间戳打在第一个激光点）
      static TicToc unDistortion_time;
      unDistortion_time.tic();
      delta_odom.block<3, 1>(0, 3) = Eigen::Vector3d(0.0, 0.0, 0.0);
      ptr_feature_extractor_->adjustDistortion(delta_odom);
      LOG(INFO) << "unDistortion_time: " << unDistortion_time.toc()
                << ", ave_time: " << unDistortion_time.getAveTime();
    }

    Vec3f eigen_vec;
    if (!ptr_feature_extractor_->cloudExtractor(surf_cloud_, occ_cloud_, eigen_vec)) {
      LOG(ERROR) << "ptr_feature_extractor_->cloudExtractor failed !!!";
      return false;
    }

    // // 初始几帧采用稠密点云，尽快填充地图；
    // if (id_ < 5) {
    //   float cloud_reso = 0.2 * ptr_config_->vox_size;
    //   SpaceVoxel<PointType> ds_cloud(cloud_reso, cloud_reso, cloud_reso);
    //   laserCloud::Ptr cur_cloud_ds(new laserCloud());
    //   ds_cloud.Clear();
    //   ds_cloud.InsertCloud(*input_cloud);
    //   ds_cloud.getVoxelCloud(*cur_cloud_ds);

    //   surf_cloud_->clear();
    //   *surf_cloud_ = *cur_cloud_ds;
    // }

    ptr_cur_lidar_odom_keyframe_ = nullptr;

    // 根据提取的特征和预测值，创建关键帧；
    id_++;

    ptr_cur_lidar_odom_keyframe_ = std::make_shared<KeyFrame>(
        id_, temp_frame_time_, curr_correspond_odom_pose_, predict_lidar_odom_pose,
        predict_cur_frame_pose, curr_linear, curr_angular, cur_imu, surf_cloud_, occ_cloud_);

    ptr_cur_lidar_odom_keyframe_->updateDenseCloud(ptr_feature_extractor_->getDenseRawCloud());

    if (ptr_config_->calculate_degenerate == 2) {
      ptr_cur_lidar_odom_keyframe_->updateEigenVec(eigen_vec);
    }

    curr_keyframe_time_ = temp_frame_time_;

    LOG(INFO) << "CreateFrame -- "
              << "id: " << id_ << ", surf: " << surf_cloud_->size()
              << ", occ: " << occ_cloud_->size() << ", time: " << create_frame_cost.toc()
              << ", ave_create_time: " << create_frame_cost.getAveTime();
    LOG(INFO);

    return true;
  } else {
    return false;
  }
}

void System::addCloudFrame(const laserCloud::Ptr cloud_in, const double time_stamp) {
  std::lock_guard<std::mutex> lock(cloud_data_mutex_);

  if (is_first_cloud_ == true) {
    is_first_cloud_ = false;
  }

  d_cloud_measures_.emplace_back(cloud_in, time_stamp);
  lidar_count_++;

  if (d_cloud_measures_.size() > 10) {
    LOG(WARNING) << "addCloudFrame-size: " << d_cloud_measures_.size();
  }
}

void System::addOdom(const OdomMeasure &odom) {
  static Mat34d _last_raw_odom_pose = Mathbox::Identity34();
  static Mat34d _last_smooth_pose = Mathbox::Identity34();
  float fusion_pose_smooth_ratio = ptr_config_->fusion_pose_smooth_ratio;

  std::lock_guard<std::mutex> lock(odom_data_mutex_);

  // 将odom转换到初始定位后坐标系下；
  Mat34d odom_pose = Mathbox::multiplePose34d(init_map_to_odom_, odom.pose);

  d_odom_measures_[odom.time_stamp] =
      OdomMeasure(odom_pose, odom.linear, odom.angular, odom.time_stamp);
  odom_count_++;

  Mat34d correct_lidar_odom_pose = Mathbox::multiplePose34d(lidar_odom_to_odom_, odom_pose);

  Mat34d correct_map_pose = Mathbox::multiplePose34d(cur_map_to_odom_, correct_lidar_odom_pose);

  if (is_first_odom_ == true) {
    if (odom_count_ < 50) {
      _last_raw_odom_pose = odom.pose;
      _last_smooth_pose = correct_map_pose;
      is_first_odom_ = false;
    }
  }

  Mat34d delta_odom = Mathbox::deltaPose34d(_last_raw_odom_pose, odom.pose);

  if (ptr_lidar_odom_->isWheelStable() == false) {
    Vec3d p_predict = Vec3d::Zero();
    delta_odom.block<3, 1>(0, 3) = p_predict;
  }
  Mat34d predict_robot_pose = Mathbox::multiplePose34d(_last_smooth_pose, delta_odom);
  Mat34d smooth_robot_pose = Mathbox::Interp_SE3(predict_robot_pose, correct_map_pose,
                                                 fusion_pose_smooth_ratio);  // 0.01

  cur_robot_state_.pose = smooth_robot_pose;
  cur_robot_state_.time_stamp = odom.time_stamp;

  _last_smooth_pose = smooth_robot_pose;
  _last_raw_odom_pose = odom.pose;

  if (d_odom_measures_.size() > 50) {
    LOG(WARNING) << "addOdom-size: " << d_odom_measures_.size();
  }
}

const laserCloud::Ptr System::getCurKeyFrameSurf() {
  if (ptr_cur_keyframe_ != nullptr) {
    return ptr_cur_keyframe_->surf_cloud_;
  } else {
    return nullptr;
  }
}

const laserCloud::Ptr System::getCurrSurroundCloud() {
  if (ptr_config_->view_map_min_prob < 0.0) {
    return ptr_map_track_->getCurrSurroundMap();
    // return ptr_lidar_odom_->getCurrSurroundMap();
  } else {
    laserCloud::Ptr surround_map = ptr_map_track_->getCurrSurroundMap();
    laserCloud::Ptr surround_map_prob(new laserCloud());
    for (u_int32_t i = 0; i < surround_map->size(); i++) {
      if (surround_map->points[i].intensity > ptr_config_->view_map_min_prob) {
        surround_map_prob->emplace_back(surround_map->points[i]);
      }
    }
    return surround_map_prob;
  }
}

const laserCloud::Ptr System::getKeyPoseCloud() {
  return key_pose_cloud_;
}

double System::getCurKeyFrameTimeStamp() {
  double t = -1.0;
  if (ptr_cur_keyframe_ != nullptr) {
    t = ptr_cur_keyframe_->time_stamp_;
  }
  return t;
}

const RobotState &System::getRobotState() {
  return cur_robot_state_;
}

const Mat34d System::getKeyFramePose() {
  Mat34d pose = Mathbox::Identity34();
  if (ptr_cur_keyframe_ != nullptr) {
    pose = ptr_cur_keyframe_->T_wb_;
  }
  return pose;
}

const Mat34d System::getKeyFrameLidarOdomPose() {
  Mat34d pose = Mathbox::Identity34();
  if (ptr_cur_keyframe_ != nullptr) {
    pose = ptr_cur_keyframe_->T_wl_;
  }
  return pose;
}

const Mat34d System::getKeyFrameWheelOdomPose() {
  Mat34d pose = Mathbox::Identity34();
  if (ptr_cur_keyframe_ != nullptr) {
    pose = ptr_cur_keyframe_->T_wo_;
  }
  return pose;
}

void System::saveCurPoseToFile() {
  // for kitti dataset, read calib file first;
  static Mat34d kitti_T_cl, kitti_T_lc;
  static bool calib_file_read_flag = false;
  if (ptr_config_->dataset_mode == 1 && !calib_file_read_flag) {
    // the ground truth of kitti dataset is in camera coordinate;
    int sq_num = std::stoi(ptr_config_->kitti_sequence);
    LOG(WARNING) << "sq_num: " << sq_num;

    std::string calib_path =
        "/home/suyun/CVTE/dataset/dataset-3D/kitti/data_odometry_calib/dataset/"
        "sequences/" +
        ptr_config_->kitti_sequence + "/calib.txt";
    std::ifstream calib_file(calib_path, std::ifstream::in);
    if (!calib_file.is_open()) {
      LOG(ERROR) << "cannot open kitti calib file !!!";
      return;
    }

    int line_num = 0;
    std::string line;
    while (std::getline(calib_file, line)) {
      line_num++;
      if (line.empty() || line_num < 5) {
        continue;
      }
      if (line.front() == 'T') {
        std::stringstream ss;
        ss << line;
        std::string head;
        ss >> head;

        double T_cl[12] = {0.};
        for (int i = 0; i < 12; ++i) { ss >> T_cl[i]; }
        kitti_T_cl << T_cl[0], T_cl[1], T_cl[2], T_cl[3], T_cl[4], T_cl[5], T_cl[6], T_cl[7],
            T_cl[8], T_cl[9], T_cl[10], T_cl[11];
        LOG(WARNING) << "kitti_T_cl: \n" << kitti_T_cl;
        kitti_T_lc = Mathbox::inversePose34d(kitti_T_cl);
      } else {
        LOG(ERROR) << "System:: read calib file failed !!!";
        return;
      }
      break;
    }
    calib_file.close();
    calib_file_read_flag = true;
  }

  Eigen::Vector3d cur_t;
  Eigen::Quaterniond cur_q;
  double time = ptr_cur_keyframe_->time_stamp_;
  Mat34d lidar_odom_pose = ptr_cur_keyframe_->getLaserOdomPose();
  Mat34d cur_key_pose = ptr_cur_keyframe_->getPose();

  // 0: tum; 1: kitti;
  if (ptr_config_->dataset_mode == 0) {
    cur_t = lidar_odom_pose.block<3, 1>(0, 3);
    cur_q = Eigen::Quaterniond(lidar_odom_pose.block<3, 3>(0, 0));
    cur_q.normalize();
    lidar_odom_pose_file_ << std::fixed << time << ' ' << cur_t.x() << ' ' << cur_t.y() << ' '
                          << cur_t.z() << ' ' << cur_q.x() << ' ' << cur_q.y() << ' ' << cur_q.z()
                          << ' ' << cur_q.w() << std::endl;

    cur_t = cur_key_pose.block<3, 1>(0, 3);
    cur_q = Eigen::Quaterniond(cur_key_pose.block<3, 3>(0, 0));
    cur_q.normalize();
    lidar_key_pose_file_ << std::fixed << time << ' ' << cur_t.x() << ' ' << cur_t.y() << ' '
                         << cur_t.z() << ' ' << cur_q.x() << ' ' << cur_q.y() << ' ' << cur_q.z()
                         << ' ' << cur_q.w() << std::endl;
  } else if (ptr_config_->dataset_mode == 1) {
    // convert pose from lidar to left camera coordinate;
    Mat34d cur_pose =
        Mathbox::multiplePose34d(kitti_T_cl, Mathbox::multiplePose34d(lidar_odom_pose, kitti_T_lc));

    lidar_odom_pose_file_ << std::fixed << cur_pose(0, 0) << ' ' << cur_pose(0, 1) << ' '
                          << cur_pose(0, 2) << ' ' << cur_pose(0, 3) << ' ' << cur_pose(1, 0) << ' '
                          << cur_pose(1, 1) << ' ' << cur_pose(1, 2) << ' ' << cur_pose(1, 3) << ' '
                          << cur_pose(2, 0) << ' ' << cur_pose(2, 1) << ' ' << cur_pose(2, 2) << ' '
                          << cur_pose(2, 3) << std::endl;

    cur_pose =
        Mathbox::multiplePose34d(kitti_T_cl, Mathbox::multiplePose34d(cur_key_pose, kitti_T_lc));

    lidar_key_pose_file_ << std::fixed << cur_pose(0, 0) << ' ' << cur_pose(0, 1) << ' '
                         << cur_pose(0, 2) << ' ' << cur_pose(0, 3) << ' ' << cur_pose(1, 0) << ' '
                         << cur_pose(1, 1) << ' ' << cur_pose(1, 2) << ' ' << cur_pose(1, 3) << ' '
                         << cur_pose(2, 0) << ' ' << cur_pose(2, 1) << ' ' << cur_pose(2, 2) << ' '
                         << cur_pose(2, 3) << std::endl;
  }
}

void System::calculateTrajError() {
  // 计算平均轨迹误差  0: tum; 1: kitti;
  if (ptr_config_->dataset_mode == 1) {
    LOG(WARNING) << "begin calculate ave error of kitti dataset !!! ";

    std::string gt_dir = ptr_config_->dataset_path + "/data_odometry_poses/dataset/poses/" +
                         ptr_config_->kitti_sequence + ".txt";
    std::string result_dir = result_path_ + "lidar_odom_pose_file.txt";
    LOG(WARNING) << "gt_dir: " << gt_dir;
    LOG(WARNING) << "result_dir: " << result_dir;

    std::pair<float, float> tr_error = CalcAveErrorFromFile(gt_dir, result_dir);
    LOG(WARNING) << "ave lidar_odom trans error: " << tr_error.first;
    LOG(WARNING) << "ave lidar_odom rotat error: " << tr_error.second;

    result_dir = result_path_ + "lidar_key_pose_file.txt";
    LOG(WARNING) << "result_dir: " << result_dir;

    tr_error = CalcAveErrorFromFile(gt_dir, result_dir);
    LOG(WARNING) << "ave lidar_key trans error: " << tr_error.first;
    LOG(WARNING) << "ave lidar_key rotat error: " << tr_error.second;
  }
}

void System::Reset() {
  init_map_to_odom_ = Mathbox::Identity34();
  last_map_to_odom_ = Mathbox::Identity34();
  cur_map_to_odom_ = Mathbox::Identity34();
  lidar_odom_to_odom_ = Mathbox::Identity34();
  init_imu_pose_ = Mathbox::Identity34();
  curr_correspond_odom_pose_ = Mathbox::Identity34();
  last_correspond_odom_pose_ = Mathbox::Identity34();
  curr_lidar_odom_pose_ = Mathbox::Identity34();
  last_lidar_odom_pose_ = Mathbox::Identity34();

  cur_robot_state_.Reset();
  is_first_cloud_ = true;
  is_first_odom_ = true;
  is_first_keyframe_ = true;
  new_keyframe_flag_ = false;
  id_ = 0;
  {
    std::lock_guard<std::mutex> lock(odom_data_mutex_);
    d_odom_measures_.clear();
    std::map<double, OdomMeasure>().swap(d_odom_measures_);
  }
  {
    std::lock_guard<std::mutex> lock(imu_data_mutex_);
    d_imu_measures_.clear();
    std::deque<ImuMeasure>().swap(d_imu_measures_);
  }
  {
    std::lock_guard<std::mutex> lock(cloud_data_mutex_);
    d_cloud_measures_.clear();
    std::deque<CloudMeasure>().swap(d_cloud_measures_);
  }
  odom_count_ = 0;
  imu_count_ = 0;
  lidar_count_ = 0;

  ptr_lidar_odom_->Reset();
  ptr_map_track_->Reset();

  malloc_trim(0);

  LOG(WARNING) << "System: Reset all data !!!" << std::endl;
}

void System::shutdown() {
  d_odom_measures_.clear();
  d_imu_measures_.clear();
  d_cloud_measures_.clear();

  is_stopped_ = true;

  if (ptr_lidarOdom_thread_->joinable()) {
    ptr_lidarOdom_thread_->join();
  }

  if (ptr_maptrack_thread_->joinable()) {
    ptr_maptrack_thread_->join();
  }

  if (ptr_config_->save_keyPose_to_file) {
    lidar_odom_pose_file_.close();
    lidar_key_pose_file_.close();
  }
  if (ptr_config_->save_runinfo_to_file) {
    runinfo_result_.close();
  }

  calculateTrajError();

  LOG(ERROR) << "system stopped";
}

}  // namespace GR_SLAM
