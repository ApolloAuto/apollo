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

// Tracker which uses deep learning ROI features from detection

#ifndef MODULES_PERCEPTION_OBSTACLE_CAMERA_TRACKER_DLF_AFFINITY_TRACKER_H_
#define MODULES_PERCEPTION_OBSTACLE_CAMERA_TRACKER_DLF_AFFINITY_TRACKER_H_

#include <limits>
#include <unordered_map>
#include <vector>

#include "modules/perception/obstacle/camera/tracker/base_affinity_tracker.h"

namespace apollo {
namespace perception {

class DLFAffinityTracker : public BaseAffinityTracker {
 public:
  DLFAffinityTracker() : BaseAffinityTracker() {}

  virtual ~DLFAffinityTracker() {}

  bool Init() override;

  bool GetAffinityMatrix(
      const cv::Mat &img, const std::vector<Tracked> &tracked,
      const std::vector<Detected> &detected,
      std::vector<std::vector<float>> *affinity_matrix) override;

  bool UpdateTracked(const cv::Mat &img, const std::vector<Detected> &detected,
                     std::vector<Tracked> *tracked) override;

 private:
  //  Thresholds are fine-tuned and detector-dependant
  const float kConfThreshold_ = 1.0f;
  const float kFilterThreshold_ = 0.0f;
};

}  // namespace perception
}  // namespace apollo

#endif  // MODULES_PERCEPTION_OBSTACLE_CAMERA_TRACKER_DLF_AFFINITY_TRACKER_H_
