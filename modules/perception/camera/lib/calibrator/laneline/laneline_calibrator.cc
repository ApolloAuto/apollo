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
#include "modules/perception/camera/lib/calibrator/laneline/laneline_calibrator.h"

#include "absl/strings/str_cat.h"

namespace apollo {
namespace perception {
namespace camera {

bool LaneLineCalibrator::Init(const CalibratorInitOptions &options) {
  LocalCalibratorInitOptions local_options;
  local_options.cx = options.cx;
  local_options.cy = options.cy;
  local_options.focal_x = options.focal_x;
  local_options.focal_y = options.focal_y;
  image_width_ = local_options.image_width = options.image_width;
  image_height_ = local_options.image_height = options.image_height;
  calibrator_.Init(local_options);
  return true;
}

bool LaneLineCalibrator::Calibrate(const CalibratorOptions &options,
                                   float *pitch_angle) {
  if (pitch_angle == nullptr) {
    AERROR << "pitch_angle is not available";
    return false;
  }
  EgoLane ego_lane;
  if (!LoadEgoLaneline(*options.lane_objects, &ego_lane)) {
    AINFO << "Failed to get the ego lane.";
    return false;
  }

  double cam_ori[4] = {0};
  cam_ori[3] = 1.0;

  // Camera to world pose
  Eigen::Affine3d c2w = *options.camera2world_pose;

  double p2w[12] = {
      c2w(0, 0), c2w(0, 1), c2w(0, 2), c2w(0, 3), c2w(1, 0), c2w(1, 1),
      c2w(1, 2), c2w(1, 3), c2w(2, 0), c2w(2, 1), c2w(2, 2), c2w(2, 3),
  };

  ADEBUG << "c2w transform this frame:\n"
         << p2w[0] << ", " << p2w[1] << ", " << p2w[2] << ", " << p2w[3] << "\n"
         << p2w[4] << ", " << p2w[5] << ", " << p2w[6] << ", " << p2w[7] << "\n"
         << p2w[8] << ", " << p2w[9] << ", " << p2w[10] << ", " << p2w[11];

  common::IMultAx3x4(p2w, cam_ori, cam_coord_cur_);
  time_diff_ = kTimeDiffDefault;
  yaw_rate_ = kYawRateDefault;
  velocity_ = kVelocityDefault;

  timestamp_cur_ = *options.timestamp;
  if (!is_first_frame_) {
    time_diff_ = fabsf(static_cast<float>(timestamp_cur_ - timestamp_pre_));
    ADEBUG << timestamp_cur_ << " " << timestamp_pre_ << std::endl;
    camera::GetYawVelocityInfo(time_diff_, cam_coord_cur_, cam_coord_pre_,
                               cam_coord_pre_pre_, &yaw_rate_, &velocity_);
    std::string timediff_yawrate_velocity_text =
        absl::StrCat("time_diff_: ", std::to_string(time_diff_).substr(0, 4),
                     " | yaw_rate_: ", std::to_string(yaw_rate_).substr(0, 4),
                     " | velocity_: ", std::to_string(velocity_).substr(0, 4));
    ADEBUG << timediff_yawrate_velocity_text << std::endl;
  }

  bool updated =
      calibrator_.Process(ego_lane, velocity_, yaw_rate_, time_diff_);
  if (updated) {
    *pitch_angle = calibrator_.get_pitch_estimation();
    float vanishing_row = calibrator_.get_vanishing_row();
    AINFO << "#updated pitch angle: " << *pitch_angle;
    AINFO << "#vanishing row: " << vanishing_row;
  }

  if (!is_first_frame_) {
    memcpy(cam_coord_pre_pre_, cam_coord_pre_, sizeof(double) * 3);
  }
  is_first_frame_ = false;
  memcpy(cam_coord_pre_, cam_coord_cur_, sizeof(double) * 3);
  timestamp_pre_ = timestamp_cur_;
  return updated;
}

bool LaneLineCalibrator::LoadEgoLaneline(
    const std::vector<base::LaneLine> &lane_objects, EgoLane *ego_lane) {
  if (ego_lane == nullptr) {
    AERROR << "ego_lane is not available";
    return false;
  }

  bool found_ego_left = false;
  bool found_ego_right = false;
  // Using points from model fitting
  for (size_t i = 0; i < lane_objects.size(); ++i) {
    if (lane_objects[i].pos_type == base::LaneLinePositionType::EGO_LEFT) {
      int num_points =
          static_cast<int>(lane_objects[i].curve_image_point_set.size());
      ego_lane->left_line.lane_point.resize(num_points);
      const auto &curve = lane_objects[i].curve_image_coord;
      float y_curve = 0.0f;
      float x_curve = 0.0f;
      float x_raw = 0.0f;
      for (int j = 0; j < num_points; ++j) {
        y_curve = lane_objects[i].curve_image_point_set[j].y;
        x_curve = curve.a * common::ICub(y_curve) +
                  curve.b * common::ISqr(y_curve) + curve.c * y_curve + curve.d;
        x_raw = lane_objects[i].curve_image_point_set[j].x;
        ego_lane->left_line.lane_point[j](1) = y_curve;
        ego_lane->left_line.lane_point[j](0) = (x_curve + x_raw) / 2;
      }
      found_ego_left = true;
    }
    if (lane_objects[i].pos_type == base::LaneLinePositionType::EGO_RIGHT) {
      int num_points =
          static_cast<int>(lane_objects[i].curve_image_point_set.size());
      ego_lane->right_line.lane_point.resize(num_points);
      const auto &curve = lane_objects[i].curve_image_coord;
      float y_curve = 0.0f;
      float x_curve = 0.0f;
      float x_raw = 0.0f;
      for (int j = 0; j < num_points; ++j) {
        y_curve = lane_objects[i].curve_image_point_set[j].y;
        x_curve = curve.a * common::ICub(y_curve) +
                  curve.b * common::ISqr(y_curve) + curve.c * y_curve + curve.d;
        x_raw = lane_objects[i].curve_image_point_set[j].x;
        ego_lane->right_line.lane_point[j](1) = y_curve;
        ego_lane->right_line.lane_point[j](0) = (x_curve + x_raw) / 2;
      }
      found_ego_right = true;
    }
    if (found_ego_left && found_ego_right) {
      return true;
    }
  }
  return false;
}

// Register plugin.
REGISTER_CALIBRATOR(LaneLineCalibrator);

}  // namespace camera
}  // namespace perception
}  // namespace apollo
