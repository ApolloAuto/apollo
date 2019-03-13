/******************************************************************************
 * Copyright 2018 The Apollo Authors. All Rights Reserved.
 *
 * Licensed under the Apache License, Version 2.0 (the License);
 * you may not use this file except in compliance with the License.
 * You may obtain a copy of the License at
 *
 * http://www.apache.org/licenses/LICENSE-2.0
 *
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an AS IS BASIS,
 * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 * See the License for the specific language governing permissions and
 * limitations under the License.
 *****************************************************************************/
#pragma once

#include <string>
#include <vector>

#include "modules/perception/camera/lib/calibrator/laneline/lane_based_calibrator.h"
#include "modules/perception/camera/lib/interface/base_calibrator.h"

#include "modules/perception/common/i_lib/core/i_blas.h"

namespace apollo {
namespace perception {
namespace camera {

class LaneLineCalibrator : public BaseCalibrator {
 public:
  LaneLineCalibrator() : BaseCalibrator() {}

  virtual ~LaneLineCalibrator() {}

  bool Init(
      const CalibratorInitOptions &options = CalibratorInitOptions()) override;

  // @brief: estimate camera-ground plane pitch.
  // @param [in]: options
  // @param [in/out]: pitch_angle
  bool Calibrate(const CalibratorOptions &options, float *pitch_angle) override;

  std::string Name() const override { return "LaneLineCalibrator"; }

  inline float GetTimeDiff() const { return time_diff_; }
  inline float GetYawRate() const { return yaw_rate_; }
  inline float GetVelocity() const { return velocity_; }
  inline float GetVanishingRow() const {
    return calibrator_.get_vanishing_row();
  }

 private:
  // @brief: load ego lane line from camera frame
  // @param [in]: lane_objects
  // @param [out]: ego_lane
  bool LoadEgoLaneline(const std::vector<base::LaneLine> &lane_objects,
                       EgoLane *ego_lane);

 private:
  int image_width_ = 0;
  int image_height_ = 0;
  double cam_coord_pre_pre_[3] = {0.0f};
  double cam_coord_pre_[3] = {0.0f};
  double cam_coord_cur_[3] = {0.0f};
  double timestamp_pre_ = 0.0f;
  double timestamp_cur_ = 0.0f;
  bool is_first_frame_ = true;
  float time_diff_ = kTimeDiffDefault;
  float yaw_rate_ = kYawRateDefault;
  float velocity_ = kVelocityDefault;
  LaneBasedCalibrator calibrator_;
};

}  // namespace camera
}  // namespace perception
}  // namespace apollo
