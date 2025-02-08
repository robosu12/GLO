/************************************************************************
 * Software License Agreement (BSD License)
 * @version: v 1.0
 ************************************************************************/
#ifndef FEATURE_EXTRACT_FACTORY_HPP
#define FEATURE_EXTRACT_FACTORY_HPP

#include "frontend/feature_extractor.hpp"
#include <memory>

namespace GR_SLAM {
class FeatureExtractorFactory {
 public:
  static std::shared_ptr<BaseFeatureExtractor> creatExtractor(
      const std::shared_ptr<SystemConfig> config) {
    if (nullptr == ptr_instance_) {
      ptr_instance_ = std::make_shared<FeatureExtractor>(config);
    }
    return ptr_instance_;
  }

 private:
  static std::shared_ptr<BaseFeatureExtractor> ptr_instance_;
  FeatureExtractorFactory() = default;
};  // end of class

std::shared_ptr<BaseFeatureExtractor> FeatureExtractorFactory::ptr_instance_ = nullptr;

}  // namespace GR_SLAM

#endif