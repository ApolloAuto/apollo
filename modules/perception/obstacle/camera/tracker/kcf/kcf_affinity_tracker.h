/******************************************************************************
 * Copyright 2018 The Apollo Authors. All Rights Reserved.
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

#ifndef MODULES_PERCEPTION_OBSTACLE_CAMERA_TRACKER_KCF_AFFINITY_TRACKER_H_
#define MODULES_PERCEPTION_OBSTACLE_CAMERA_TRACKER_KCF_AFFINITY_TRACKER_H_

#include <limits>
#include <unordered_map>
#include <vector>

#include "modules/perception/obstacle/camera/tracker/base_affinity_tracker.h"
#include "modules/perception/obstacle/camera/tracker/cascaded_camera_tracker_util.h"
#include "modules/perception/obstacle/camera/tracker/kcf/kcf_components.h"

namespace apollo {
namespace perception {

class KCFAffinityTracker : public BaseAffinityTracker {
 public:
  KCFAffinityTracker() : BaseAffinityTracker() {}

  virtual ~KCFAffinityTracker() {}

  bool Init() override;

  bool GetAffinityMatrix(
      const cv::Mat &img, const std::vector<Tracked> &tracked,
      const std::vector<Detected> &detected,
      std::vector<std::vector<float>> *affinity_matrix) override;

  bool UpdateTracked(const cv::Mat &img, const std::vector<Detected> &detected,
                     std::vector<Tracked> *tracked) override;

 private:
  // KCF module
  KCFComponents kcf_component_;

  // z_f for all detected objects
  std::unordered_map<int, std::vector<cv::Mat>> detected_features_;

  const float kKeepThreshold_ = 0.3f;
  const float kScale_ = 2.5f;
};

}  // namespace perception
}  // namespace apollo

#endif  // MODULES_PERCEPTION_OBSTACLE_CAMERA_TRACKER_KCF_AFFINITY_TRACKER_H_
