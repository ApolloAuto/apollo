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

#include <deque>
#include <utility>

#include "modules/perception/camera/lib/calibrator/common/histogram_estimator.h"
#include "modules/perception/camera/lib/calibrator/laneline/lane_struct_for_calib.h"
#include "modules/perception/common/i_lib/core/i_basic.h"

namespace apollo {
namespace perception {
namespace camera {

const float kYawRateDefault = 0.0f;
const float kVelocityDefault = 8.333f;  // m/s
const float kTimeDiffDefault = 0.067f;  // in seconds, 15FPS

void GetYawVelocityInfo(const float &time_diff, const double cam_coord_cur[3],
                        const double cam_coord_pre[3],
                        const double cam_coord_pre_pre[3], float *yaw_rate,
                        float *velocity);

struct CalibratorParams {
  CalibratorParams() { Init(); }

  void Init();

  int min_nr_pts_laneline;

  float sampling_lane_point_rate;
  float max_allowed_yaw_angle_in_radian;
  float min_distance_to_update_calibration_in_meter;
  float min_required_straight_driving_distance_in_meter;

  HistogramEstimatorParams hist_estimator_params;

  void operator=(const CalibratorParams &params) {
    min_nr_pts_laneline = params.min_nr_pts_laneline;

    min_distance_to_update_calibration_in_meter =
        params.min_distance_to_update_calibration_in_meter;
    min_distance_to_update_calibration_in_meter =
        params.min_distance_to_update_calibration_in_meter;
    min_required_straight_driving_distance_in_meter =
        params.min_required_straight_driving_distance_in_meter;

    hist_estimator_params = params.hist_estimator_params;
  }
};

struct LocalCalibratorInitOptions {
  int image_width = 0;
  int image_height = 0;
  float focal_x = 0.0f;
  float focal_y = 0.0f;
  float cx = 0.0f;
  float cy = 0.0f;
};

class LaneBasedCalibrator {
 public:
  static const int kMaxNrHistoryFrames = 1000;

  LaneBasedCalibrator() { vp_buffer_.clear(); }

  ~LaneBasedCalibrator() { ClearUp(); }

  void Init(const LocalCalibratorInitOptions &options,
            const CalibratorParams *params = nullptr);

  void ClearUp();

  // Main function. process every frame, return true if get valid
  // estimation. suppose the points in lane are already sorted.
  bool Process(const EgoLane &lane, const float &velocity,
               const float &yaw_rate, const float &time_diff);

  float get_pitch_estimation() const { return pitch_estimation_; }

  // float get_pitch_cur() const {
  //   return pitch_cur_;
  // }

  float get_vanishing_row() const { return vanishing_row_; }

  // float get_accumulated_straight_driving_in_meter() const {
  //   return accumulated_straight_driving_in_meter_;
  // }

 private:
  bool is_in_image(const Eigen::Vector2f &point) const {
    int x = common::IRound(point(0));
    int y = common::IRound(point(1));
    bool is_in_image =
        x >= 0 && x < image_width_ && y >= 0 && y < image_height_;
    return (is_in_image);
  }

  bool IsTravelingStraight(const float &vehicle_yaw_changed) const {
    float abs_yaw = static_cast<float>(fabs(vehicle_yaw_changed));
    return abs_yaw < params_.max_allowed_yaw_angle_in_radian;
  }

  bool GetVanishingPoint(const EgoLane &lane, VanishingPoint *v_point);

  int GetCenterIndex(const Eigen::Vector2f *points, int nr_pts) const;

  bool SelectTwoPointsFromLineForVanishingPoint(const LaneLine &line,
                                                float line_seg[4]);

  bool GetIntersectionFromTwoLineSegments(const float line_seg_l[4],
                                          const float line_seg_r[4],
                                          VanishingPoint *v_point);

  void PushVanishingPoint(const VanishingPoint &v_point);

  bool PopVanishingPoint(VanishingPoint *v_point);

  bool GetPitchFromVanishingPoint(const VanishingPoint &vp, float *pitch) const;

  bool AddPitchToHistogram(float pitch);

 private:
  int image_width_ = 0;
  int image_height_ = 0;
  float k_mat_[9] = {0};
  float pitch_cur_ = 0.0f;
  float pitch_estimation_ = 0.0f;
  float vanishing_row_ = 0.0f;
  float accumulated_straight_driving_in_meter_ = 0.0f;

  // EgoLane lane_;
  HistogramEstimator pitch_histogram_;
  std::deque<VanishingPoint> vp_buffer_;
  CalibratorParams params_;
};

}  // namespace camera
}  // namespace perception
}  // namespace apollo
