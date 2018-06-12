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

#ifndef MODULES_PERCEPTION_OBSTACLE_CAMERA_TRACKER_CS2D_AFFINITY_TRACKER_H
#define MODULES_PERCEPTION_OBSTACLE_CAMERA_TRACKER_CS2D_AFFINITY_TRACKER_H

#include <cmath>
#include <limits>
#include <unordered_map>
#include <vector>

#include "modules/perception/obstacle/camera/tracker/base_affinity_tracker.h"

namespace apollo {
namespace perception {

class CS2DAffinityTracker : public BaseAffinityTracker {
 public:
  CS2DAffinityTracker() : BaseAffinityTracker() {}

  virtual ~CS2DAffinityTracker() {}

  bool Init() override;
  bool GetAffinityMatrix(
      const cv::Mat &img, const std::vector<Tracked> &tracked,
      const std::vector<Detected> &detected,
      std::vector<std::vector<float>> *affinity_matrix) override;

  bool UpdateTracked(const cv::Mat &img, const std::vector<Detected> &detected,
                     std::vector<Tracked> *tracked) override;

 private:
  float sz_lim_ = 0.5f;         // max 2d box scale change
  float pos_range_ = 1.3f;      // max 2D center change, based on 2d box size
  float center_range_ = 10.0f;  // max diff in meter between unfiltered 3d pos
};

}  // namespace perception
}  // namespace apollo

#endif  // MODULES_PERCEPTION_OBSTACLE_CAMERA_TRACKER_CS2D_AFFINITY_TRACKER_H
