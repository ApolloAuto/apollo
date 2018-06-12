/******************************************************************************
 * Copyright 2017 The Apollo Authors. All Rights Reserved.
 *
 * Licensed under the Apache License, Version 2.0 (the "License");
 * you may not use this file except in compliance with the License.
 * You may obtain a copy of the License at
 *
 * http://www.apache.org/licenses/LICENSE-2.0
 *
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS,
 * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 * See the License for the specific language governing permissions and
 * limitations under the License.
 *****************************************************************************/

#ifndef MODULES_PERCEPTION_OBSTACLE_TRACKER_HM_TRACKER_FEATURE_DESCRIPTOR_H_
#define MODULES_PERCEPTION_OBSTACLE_TRACKER_HM_TRACKER_FEATURE_DESCRIPTOR_H_

#include <algorithm>
#include <vector>

#include "modules/perception/common/pcl_types.h"

namespace apollo {
namespace perception {

class FeatureDescriptor {
 public:
  // @brief intialize feature descriptor
  // @params[IN] cloud: given cloud for feature extraction
  // @return nothing
  explicit FeatureDescriptor(
      apollo::perception::pcl_util::PointCloudPtr cloud) {
    cloud_ = cloud;
  }
  ~FeatureDescriptor() {}

  // @brief compute histogram feature of given cloud
  // @params[IN] bin_size: bin size of histogram
  // @params[OUT] feature: histogram feature of given cloud
  // @return nothing
  void ComputeHistogram(const int bin_size, std::vector<float>* feature);

 private:
  void GetMinMaxCenter();
  apollo::perception::pcl_util::PointCloudPtr cloud_;
  pcl_util::Point min_pt_;
  pcl_util::Point max_pt_;
  pcl_util::Point center_pt_;
};  // class FeatureDescriptor

}  // namespace perception
}  // namespace apollo

#endif  // MODULES_PERCEPTION_OBSTACLE_LIDAR_CLASSIFIER_FEATURE_DESCRIPTOR_H_
