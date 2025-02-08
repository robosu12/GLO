/************************************************************************
 * Software License Agreement (BSD License)

 *@author Yun Su(robosu12@gmail.com)
 *@version 1.0
 *@data 2024-07-24
 ************************************************************************/
#ifndef KITTI_HPP
#define KITTI_HPP

#include <atomic>
#include <deque>
#include <list>
#include <string>
#include <thread>
#include <iostream>
#include <fstream>
#include <map>

#include <glog/logging.h>

#include <Eigen/Core>
#include <Eigen/Eigenvalues>
#include <Eigen/Geometry>

#include "common/math_base/slam_math.hpp"
#include "common/data_struct/pc_base.hpp"

namespace GR_SLAM {

std::pair<float, laserCloud::Ptr> readKittiSequence(std::string path, std::string sequence_number) {
  static int frame_count = 0;

  laserCloud::Ptr new_cloud(new laserCloud());
  float timestamp = -1.0;

  std::string timestamp_path =
      path + "/data_odometry_calib/dataset/sequences/" + sequence_number + "/times.txt";
  static std::ifstream timestamp_file(timestamp_path, std::ifstream::in);

  if (!timestamp_file.is_open()) {
    LOG(ERROR) << "open kitti timestamp file failed !!!";
    return std::make_pair(timestamp, new_cloud);
  }

  std::string line;
  if (std::getline(timestamp_file, line)) {
    timestamp = std::stof(line);

    // read lidar point cloud
    std::stringstream lidar_data_path;
    lidar_data_path << path + "/data_odometry_velodyne/dataset/sequences/" + sequence_number +
                           "/velodyne/"
                    << std::setfill('0') << std::setw(6) << frame_count << ".bin";
    std::ifstream lidar_data_file(lidar_data_path.str(), std::ifstream::in | std::ifstream::binary);
    lidar_data_file.seekg(0, std::ios::end);
    const size_t num_elements = lidar_data_file.tellg() / sizeof(float);
    lidar_data_file.seekg(0, std::ios::beg);

    std::vector<float> lidar_data(num_elements);
    lidar_data_file.read(reinterpret_cast<char *>(&lidar_data[0]), num_elements * sizeof(float));

    PointType point;
    float angle_offset = 0.205 * Deg2Rad;
    const Eigen::Vector3d uz(0., 0., 1.);
    Eigen::Vector3d pi, new_pi;
    Eigen::Vector3d rotationVector;

    for (std::size_t i = 0; i < lidar_data.size(); i += 4) {
      point.x = lidar_data[i];
      point.y = lidar_data[i + 1];
      point.z = lidar_data[i + 2];
      point.intensity = lidar_data[i + 3];

      // Correct wrong intrinsic calibration in the original kitti datasets:
      pi.x() = point.x;
      pi.y() = point.y;
      pi.z() = point.z;
      rotationVector = pi.cross(uz);

      const auto dr = Eigen::AngleAxisd(angle_offset, rotationVector.normalized());
      new_pi = dr * pi;

      point.x = new_pi.x();
      point.y = new_pi.y();
      point.z = new_pi.z();
      new_cloud->push_back(point);
    }

    frame_count++;

    LOG(WARNING) << "readKittiDataset - frame_count: " << frame_count;
  }

  return std::make_pair(timestamp, new_cloud);
}

// All this not so beatifull C++ functions are taken from kitti-dev-kit
double lengths[] = {100, 200, 300, 400, 500, 600, 700, 800};
int32_t num_lengths = 8;

struct errors {
  int32_t first_frame;
  double r_err;
  double t_err;
  double len;
  double speed;
  errors(int32_t first_frame, double r_err, double t_err, double len, double speed)
      : first_frame(first_frame), r_err(r_err), t_err(t_err), len(len), speed(speed) {}
};

std::vector<Eigen::Matrix4d> loadPoses(std::string file_name) {
  std::vector<Eigen::Matrix4d> poses;

  FILE *fp = fopen(file_name.c_str(), "r");
  if (!fp)
    return poses;

  double v[16];
  while (!feof(fp)) {
    if (fscanf(fp, "%lf %lf %lf %lf %lf %lf %lf %lf %lf %lf %lf %lf", &v[0], &v[1], &v[2], &v[3],
               &v[4], &v[5], &v[6], &v[7], &v[8], &v[9], &v[10], &v[11]) == 12) {
      v[12] = 0.0;
      v[13] = 0.0;
      v[14] = 0.0;
      v[15] = 1.0;
      Eigen::Matrix4d P = Eigen::Matrix4d::Identity();
      P << v[0], v[1], v[2], v[3], v[4], v[5], v[6], v[7], v[8], v[9], v[10], v[11], v[12], v[13],
          v[14], v[15];
      poses.push_back(P);
    }
  }
  fclose(fp);
  return poses;
}

std::vector<double> TrajectoryDistances(const std::vector<Eigen::Matrix4d> &poses) {
  std::vector<double> dist;
  dist.push_back(0);
  for (uint32_t i = 1; i < poses.size(); i++) {
    const Eigen::Matrix4d &P1 = poses[i - 1];
    const Eigen::Matrix4d &P2 = poses[i];

    double dx = P1(0, 3) - P2(0, 3);
    double dy = P1(1, 3) - P2(1, 3);
    double dz = P1(2, 3) - P2(2, 3);

    dist.push_back(dist[i - 1] + std::sqrt(dx * dx + dy * dy + dz * dz));
  }

  return dist;
}

int32_t LastFrameFromSegmentLength(const std::vector<double> &dist, int32_t first_frame,
                                   double len) {
  for (uint32_t i = first_frame; i < dist.size(); i++) {
    if (dist[i] > dist[first_frame] + len) {
      return i;
    }
  }
  return -1;
}

double RotationError(const Eigen::Matrix4d &pose_error) {
  double a = pose_error(0, 0);
  double b = pose_error(1, 1);
  double c = pose_error(2, 2);
  double d = 0.5 * (a + b + c - 1.0);
  return std::acos(std::max(std::min(d, 1.0), -1.0));
}

double TranslationError(const Eigen::Matrix4d &pose_error) {
  double dx = pose_error(0, 3);
  double dy = pose_error(1, 3);
  double dz = pose_error(2, 3);
  return std::sqrt(dx * dx + dy * dy + dz * dz);
}

std::vector<errors> CalcSequenceErrors(const std::vector<Eigen::Matrix4d> &poses_gt,
                                       const std::vector<Eigen::Matrix4d> &poses_result) {
  // error vector
  std::vector<errors> err;

  // parameters
  int32_t step_size = 10;  // every second

  // pre-compute distances (from ground truth as reference)
  std::vector<double> dist = TrajectoryDistances(poses_gt);

  // for all start positions do
  for (uint32_t first_frame = 0; first_frame < poses_gt.size(); first_frame += step_size) {
    // for all segment lengths do
    for (int32_t i = 0; i < num_lengths; i++) {
      // current length
      double len = lengths[i];

      // compute last frame
      int32_t last_frame = LastFrameFromSegmentLength(dist, first_frame, len);

      // continue, if sequence not long enough
      if (last_frame == -1) {
        continue;
      }

      // compute rotational and translational errors
      Eigen::Matrix4d pose_delta_gt = poses_gt[first_frame].inverse() * poses_gt[last_frame];
      Eigen::Matrix4d pose_delta_result =
          poses_result[first_frame].inverse() * poses_result[last_frame];
      Eigen::Matrix4d pose_error = pose_delta_result.inverse() * pose_delta_gt;
      double r_err = RotationError(pose_error);
      double t_err = TranslationError(pose_error);

      // compute speed
      auto num_frames = static_cast<double>(last_frame - first_frame + 1);
      double speed = len / (0.1 * num_frames);

      // write to file
      err.emplace_back(first_frame, r_err / len, t_err / len, len, speed);
    }
  }

  // return error vector
  return err;
}

std::pair<float, float> CalcAveError(const std::vector<Eigen::Matrix4d> &poses_gt,
                                     const std::vector<Eigen::Matrix4d> &poses_result) {
  std::vector<errors> err = CalcSequenceErrors(poses_gt, poses_result);
  double t_err = 0;
  double r_err = 0;

  for (const auto &it : err) {
    t_err += it.t_err;
    r_err += it.r_err;
  }

  double avg_trans_error = 100.0 * (t_err / static_cast<double>(err.size()));
  double avg_rot_error = (r_err / static_cast<double>(err.size())) / 3.14 * 180.0;

  return std::make_pair(avg_trans_error, avg_rot_error);
}

std::pair<float, float> CalcAveErrorFromFile(std::string gt_dir, std::string result_dir) {
  std::pair<float, float> trans_rot(0.0, 0.0);

  // read ground truth and result poses
  std::vector<Eigen::Matrix4d> poses_gt = loadPoses(gt_dir);
  std::vector<Eigen::Matrix4d> poses_result = loadPoses(result_dir);

  std::cout << "load poses_gt: " << poses_gt.size() << std::endl;
  std::cout << "load poses_result: " << poses_result.size() << std::endl;
  if (poses_gt.size() > 0 && poses_gt.size() == poses_result.size()) {
    std::cout << "load pose successful !!! " << std::endl;
  } else {
    std::cout << "load pose failed !!! " << std::endl;
    return trans_rot;
  }

  trans_rot = CalcAveError(poses_gt, poses_result);

  return trans_rot;
}

}  // namespace GR_SLAM
#endif