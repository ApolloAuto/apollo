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

// Track ID association for detected camera objects between frames
// Multiple trackers' affinity matrix outputs are cascaded and merged

#ifndef MODULES_PERCEPTION_OBSTACLE_CAMERA_TRACKER_CASCADED_H_
#define MODULES_PERCEPTION_OBSTACLE_CAMERA_TRACKER_CASCADED_H_

#include <algorithm>
#include <map>
#include <memory>
#include <string>
#include <unordered_map>
#include <unordered_set>
#include <utility>
#include <vector>

#include "opencv2/opencv.hpp"

#include "modules/common/log.h"
#include "modules/common/macro.h"
#include "modules/perception/lib/base/registerer.h"
#include "modules/perception/obstacle/camera/common/visual_object.h"
#include "modules/perception/obstacle/camera/interface/base_camera_tracker.h"
#include "modules/perception/obstacle/camera/tracker/base_affinity_tracker.h"
#include "modules/perception/obstacle/camera/tracker/cascaded_camera_tracker_util.h"
#include "modules/perception/obstacle/camera/tracker/cs2d/cs2d_affinity_tracker.h"
#include "modules/perception/obstacle/camera/tracker/dlf/dlf_affinity_tracker.h"
#include "modules/perception/obstacle/camera/tracker/kcf/kcf_affinity_tracker.h"

namespace apollo {
namespace perception {

class CascadedCameraTracker : public BaseCameraTracker {
 public:
  CascadedCameraTracker() : BaseCameraTracker() {}

  virtual ~CascadedCameraTracker() {}

  bool Init() override;

  bool Associate(const cv::Mat& img, const double& timestamp,
                 std::vector<std::shared_ptr<VisualObject>>* objects) override;

  std::string Name() const override;

 private:
  bool dl_feature_ = true;
  bool use_kcf_ = false;

  // Trackers for different stages
  CS2DAffinityTracker cs2d_tracker_;
  DLFAffinityTracker dlf_tracker_;
  KCFAffinityTracker kcf_tracker_;

  // Tracking and ID management
  std::vector<Tracked> tracks_;
  int frame_idx_ = 0;
  int next_track_id_ = 0;
  const int kKeptFrameCnt = 10;  // Latent space of potential tracks

  DISALLOW_COPY_AND_ASSIGN(CascadedCameraTracker);
};

REGISTER_CAMERA_TRACKER(CascadedCameraTracker);

}  // namespace perception
}  // namespace apollo

#endif  // MODULES_PERCEPTION_OBSTACLE_CAMERA_TRACKER_CASCADED_H_
