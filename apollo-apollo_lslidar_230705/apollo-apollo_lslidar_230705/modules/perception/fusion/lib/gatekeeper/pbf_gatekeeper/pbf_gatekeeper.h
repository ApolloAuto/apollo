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

#include <string>

#include "cyber/common/macros.h"
#include "modules/perception/fusion/lib/interface/base_gatekeeper.h"
#include "modules/perception/pipeline/plugin.h"

namespace apollo {
namespace perception {
namespace fusion {

struct PbfGatekeeperParams {
  bool publish_if_has_lidar = true;
  bool publish_if_has_radar = true;
  bool publish_if_has_camera = true;
  bool use_camera_3d = true;

  double min_radar_confident_distance = 40;
  double max_radar_confident_angle = 20;
  double min_camera_publish_distance = 50;
  double invisible_period_threshold = 0.001;
  double toic_threshold = 0.8;
  double existence_threshold = 0.7;
  double radar_existence_threshold = 0.9;

  bool use_track_time_pub_strategy = true;
  int pub_track_time_thresh = 3;
};

class PbfGatekeeper : public BaseGatekeeper {
 public:
  PbfGatekeeper() { name_ = "PbfGatekeeper"; }
  explicit PbfGatekeeper(const PluginConfig& plugin_config);
  ~PbfGatekeeper() = default;

  bool Init() override;

  bool AbleToPublish(const TrackPtr& track) override;

  bool Init(const PluginConfig& plugin_config) override;

  bool IsEnabled() const override { return enable_; }

  std::string Name() const override { return name_; }

 private:
  bool LidarAbleToPublish(const TrackPtr& track);
  bool RadarAbleToPublish(const TrackPtr& track, bool is_night);
  bool CameraAbleToPublish(const TrackPtr& track, bool is_night);

  PbfGatekeeperParams params_;

  DISALLOW_COPY_AND_ASSIGN(PbfGatekeeper);
};

}  // namespace fusion
}  // namespace perception
}  // namespace apollo
