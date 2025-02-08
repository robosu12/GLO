/************************************************************************
 * Software License Agreement (BSD License)

 *@file load_param_ros1.hpp
 *
 *@brief
 * 相关参数配置类
 *
 *@author YunSu(robosu12@gmail.com)
 *@version v1.0
 *@data 2024-08-05
 ************************************************************************/
#ifndef _NEW_LIDAR_SLAM_LOAD_PARAM_
#define _NEW_LIDAR_SLAM_LOAD_PARAM_

#include "common/config/system_config.hpp"
#include <memory>
#include <glog/logging.h>
#include <yaml-cpp/yaml.h>

namespace GR_SLAM {

class LoadParams {
 public:
  explicit LoadParams();
  ~LoadParams();

  LoadParams(const LoadParams &) = delete;
  LoadParams &operator=(const LoadParams &) = delete;

  void loadSystemConfig(const std::string &config_path);

  void loadBackConfig();

  void loadLoopConfig();

  void loadMSFConfig();

  void loadOccMapConfig();

  void loadDepthCameraParams();

  void loadFeatureConfig();

  void loadUwbOdomConfig();

  void loadScanContextConfig();

  void loadDenseMaptConfig();

  void loadGpsConfig();

  std::shared_ptr<SystemConfig> ptr_config_ = nullptr;

  template <typename ParameterT>
  void getParameter(const std::string &name, ParameterT &value,
                    const ParameterT &alternative_value) const;

 private:
  YAML::Node yaml_reader_;
};

}  // namespace GR_SLAM

#endif
