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

#include "modules/perception/fusion/base/sensor_data_manager.h"
#include "modules/perception/fusion/base/track.h"
#include "modules/perception/fusion/lib/interface/base_shape_fusion.h"

namespace apollo {
namespace perception {
namespace fusion {

class PbfShapeFusion : public BaseShapeFusion {
 public:
  explicit PbfShapeFusion(TrackPtr track) : BaseShapeFusion(track) {}
  virtual ~PbfShapeFusion() {}
  PbfShapeFusion(const PbfShapeFusion&) = delete;
  PbfShapeFusion& operator=(const PbfShapeFusion&) = delete;

  bool Init() override;

  void UpdateWithMeasurement(const SensorObjectPtr measurement,
                             double target_timestamp) override;

  void UpdateWithoutMeasurement(const std::string& sensor_id,
                                double measurement_timestamp,
                                double target_timestamp) override;

  std::string Name() const override;

  inline TrackPtr GetTrack() { return track_ref_; }

 private:
  // Update state
  void UpdateState(const SensorObjectConstPtr& measurement);
  void UpdateShape(const SensorObjectConstPtr& measurement);
  void UpdateCenter(const SensorObjectConstPtr& measurement);

  static bool s_use_camera_3d_;
  static float s_camera_radar_time_diff_th_;
};

}  // namespace fusion
}  // namespace perception
}  // namespace apollo
