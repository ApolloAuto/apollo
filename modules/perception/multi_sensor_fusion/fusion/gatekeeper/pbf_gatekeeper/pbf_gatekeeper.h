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
#include "modules/perception/multi_sensor_fusion/interface/base_gatekeeper.h"

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
  PbfGatekeeper();
  ~PbfGatekeeper();

  /**
   * @brief Initialize
   *
   * @param options
   * @return true
   * @return false
   */
  bool Init(const GatekeeperInitOptions &options) override;

  /**
   * @brief whether able to publish
   *
   * @param track
   * @return true
   * @return false
   */
  bool AbleToPublish(const TrackPtr& track) override;

  std::string Name() const override { return "PbfGatekeeper"; }

 private:
  /**
   * @brief whether lidar able to publish
   *
   * @param track
   * @return true
   * @return false
   */
  bool LidarAbleToPublish(const TrackPtr& track);

  /**
   * @brief whether radar able to publish
   *
   * @param track
   * @param is_night
   * @return true
   * @return false
   */
  bool RadarAbleToPublish(const TrackPtr& track, bool is_night);

  /**
   * @brief whether camera able to publish
   *
   * @param track
   * @param is_night
   * @return true
   * @return false
   */
  bool CameraAbleToPublish(const TrackPtr& track, bool is_night);

  PbfGatekeeperParams params_;

  DISALLOW_COPY_AND_ASSIGN(PbfGatekeeper);
};

}  // namespace fusion
}  // namespace perception
}  // namespace apollo
