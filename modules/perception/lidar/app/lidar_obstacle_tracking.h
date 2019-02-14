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
#pragma once

#include <memory>
#include <string>

#include "modules/perception/lidar/common/lidar_error_code.h"
#include "modules/perception/lidar/lib/interface/base_classifier.h"
#include "modules/perception/lidar/lib/interface/base_multi_target_tracker.h"

namespace apollo {
namespace perception {
namespace lidar {

struct LidarObstacleTrackingInitOptions {
  std::string sensor_name = "velodyne64";
};

struct LidarObstacleTrackingOptions {
  std::string sensor_name;
};

class LidarObstacleTracking {
 public:
  LidarObstacleTracking() = default;
  ~LidarObstacleTracking() = default;

  bool Init(const LidarObstacleTrackingInitOptions& options =
                LidarObstacleTrackingInitOptions());

  LidarProcessResult Process(const LidarObstacleTrackingOptions& options,
                             LidarFrame* frame);

  std::string Name() const { return "LidarObstacleTracking"; }

 private:
  std::shared_ptr<BaseMultiTargetTracker> multi_target_tracker_;
  std::shared_ptr<BaseClassifier> fusion_classifier_;
  std::string multi_target_tracker_name_;
  std::string fusion_classifier_name_;
};  // class LidarObstacleTracking

}  // namespace lidar
}  // namespace perception
}  // namespace apollo
