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
#include "modules/perception/camera/lib/calibrator/laneline/lane_based_calibrator.h"
#include "modules/perception/common/i_lib/core/i_blas.h"

namespace apollo {
namespace perception {
namespace camera {

void GetYawVelocityInfo(const float &time_diff, const double cam_coord_cur[3],
                        const double cam_coord_pre[3],
                        const double cam_coord_pre_pre[3], float *yaw_rate,
                        float *velocity) {
  assert(yaw_rate != nullptr);
  assert(velocity != nullptr);
  double time_diff_r = common::IRec(static_cast<double>(time_diff));
  double dist =
      common::ISqrt(common::ISquaresumDiffU2(cam_coord_cur, cam_coord_pre));
  *velocity = static_cast<float>(dist * time_diff_r);

  double offset_cur[2] = {
      cam_coord_cur[0] - cam_coord_pre[0],
      cam_coord_cur[1] - cam_coord_pre[1],
  };
  double offset_pre[2] = {
      cam_coord_pre[0] - cam_coord_pre_pre[0],
      cam_coord_pre[1] - cam_coord_pre_pre[1],
  };
  double yaw_cur = atan2(offset_cur[1], offset_cur[0]);
  double yaw_pre = atan2(offset_pre[1], offset_pre[0]);
  double yaw_rate_db = (yaw_cur - yaw_pre) * time_diff_r;
  *yaw_rate = static_cast<float>(yaw_rate_db);
}

void CalibratorParams::Init() {
  // General
  min_nr_pts_laneline = 20;
  sampling_lane_point_rate = 0.05f;  // 0.05f
  max_allowed_yaw_angle_in_radian = common::IDegreeToRadians(3.0f);

  // Slow set-up
  // min_distance_to_update_calibration_in_meter = 1600.0f;
  // min_required_straight_driving_distance_in_meter = 40.0f;

  // Fast set-up
  min_distance_to_update_calibration_in_meter = 100.0f;
  min_required_straight_driving_distance_in_meter = 20.0f;

  // Histogram params
  hist_estimator_params.nr_bins_in_histogram = 400;
  hist_estimator_params.data_sp = -common::IDegreeToRadians(10.0f);
  // -10 ~ 10 degree
  hist_estimator_params.data_ep = -hist_estimator_params.data_sp;

  hist_estimator_params.step_bin =
      (hist_estimator_params.data_ep - hist_estimator_params.data_sp);
  hist_estimator_params.step_bin /=
      static_cast<float>(hist_estimator_params.nr_bins_in_histogram);

  hist_estimator_params.smooth_kernel.clear();
  hist_estimator_params.smooth_kernel.push_back(1);
  hist_estimator_params.smooth_kernel.push_back(3);
  hist_estimator_params.smooth_kernel.push_back(5);
  hist_estimator_params.smooth_kernel.push_back(3);
  hist_estimator_params.smooth_kernel.push_back(1);
  hist_estimator_params.smooth_kernel_width =
      static_cast<int>(hist_estimator_params.smooth_kernel.size());
  hist_estimator_params.smooth_kernel_radius =
      hist_estimator_params.smooth_kernel_width >> 1;

  hist_estimator_params.hat_min_allowed = 0.40f;
  hist_estimator_params.hat_std_allowed = 6.25f;
  // hist_estimator_params.histogram_mass_limit = 500;
  hist_estimator_params.histogram_mass_limit = 50;
  // Decay 6-times to half value
  hist_estimator_params.decay_factor = 0.8908987f;
}

void LaneBasedCalibrator::Init(const LocalCalibratorInitOptions &options,
                               const CalibratorParams *params) {
  ClearUp();
  image_width_ = options.image_width;
  image_height_ = options.image_height;
  assert(image_width_ > 0);
  assert(image_height_ > 0);
  k_mat_[0] = options.focal_x;
  k_mat_[4] = options.focal_y;
  k_mat_[2] = options.cx;
  k_mat_[5] = options.cy;
  k_mat_[1] = k_mat_[3] = k_mat_[6] = k_mat_[7] = 0.0f;
  k_mat_[8] = 1.0f;

  if (params != nullptr) {
    params_ = *params;
  }
  pitch_histogram_.Init(&params_.hist_estimator_params);
}

void LaneBasedCalibrator::ClearUp() {
  vp_buffer_.clear();
  pitch_histogram_.Clear();
  image_width_ = 0;
  image_height_ = 0;
  memset(k_mat_, 0.0f, sizeof(float) * 9);
  pitch_cur_ = 0.0f;
  pitch_estimation_ = 0.0f;
  vanishing_row_ = 0.0f;
  accumulated_straight_driving_in_meter_ = 0.0f;
}

bool LaneBasedCalibrator::Process(const EgoLane &lane, const float &velocity,
                                  const float &yaw_rate,
                                  const float &time_diff) {
  float distance_traveled_in_meter = velocity * time_diff;
  float vehicle_yaw_changed = yaw_rate * time_diff;

  // distance_traveled_in_meter = 0.5f;  // hardcode for debug

  // TODO(Xun): Change cout or cerr to log.

  // Check for driving straight
  if (!IsTravelingStraight(vehicle_yaw_changed)) {
    AINFO << "Do not calibate if not moving straight: "
          << "yaw angle changed " << vehicle_yaw_changed;
    vp_buffer_.clear();
    return false;
  }

  VanishingPoint vp_cur;
  VanishingPoint vp_work;

  // Get the current estimation on vanishing point from lane
  if (!GetVanishingPoint(lane, &vp_cur)) {
    AINFO << "Lane is not valid for calibration.";
    return false;
  }
  vp_cur.distance_traveled = distance_traveled_in_meter;
  //  std::cout << "#current v-row: " << vp_cur.pixel_pos[1] << std::endl;

  // Push vanishing point into buffer
  PushVanishingPoint(vp_cur);
  if (!PopVanishingPoint(&vp_work)) {
    AINFO << "Driving distance is not long enough";
    return false;
  }

  // Get current estimation on pitch
  pitch_cur_ = 0.0f;
  if (!GetPitchFromVanishingPoint(vp_work, &pitch_cur_)) {
    AINFO << "Failed to estimate pitch from vanishing point.";
    return false;
  }
  //  std::cout << "#current pitch: " << pitch_cur_ << std::endl;
  vanishing_row_ = vp_work.pixel_pos[1];

  // Get the filtered output using histogram
  if (!AddPitchToHistogram(pitch_cur_)) {
    AINFO << "Calculated pitch is out-of-range.";
    return false;
  }

  accumulated_straight_driving_in_meter_ += distance_traveled_in_meter;
  //  std::cout << "acc_d: " << accumulated_straight_driving_in_meter_ << "\n";
  if (accumulated_straight_driving_in_meter_ >
          params_.min_distance_to_update_calibration_in_meter &&
      pitch_histogram_.Process()) {
    pitch_estimation_ = pitch_histogram_.get_val_estimation();
    const float cy = k_mat_[5];
    const float fy = k_mat_[4];
    vanishing_row_ = tanf(pitch_estimation_) * fy + cy;
    accumulated_straight_driving_in_meter_ = 0.0f;
    return true;
  }
  return false;
}

void LaneBasedCalibrator::PushVanishingPoint(const VanishingPoint &v_point) {
  int nr_vps = static_cast<int>(vp_buffer_.size());
  if (nr_vps < kMaxNrHistoryFrames) {
    vp_buffer_.push_back(v_point);
  } else {
    vp_buffer_.pop_front();
    vp_buffer_.push_back(v_point);
  }
}

bool LaneBasedCalibrator::PopVanishingPoint(VanishingPoint *v_point) {
  float accumulated_distance = 0.0f;
  for (const auto &vp : vp_buffer_) {
    accumulated_distance += vp.distance_traveled;
  }
  if (accumulated_distance <
      params_.min_required_straight_driving_distance_in_meter) {
    return false;
  }
  *v_point = vp_buffer_.back();
  vp_buffer_.pop_back();
  return true;
}

bool LaneBasedCalibrator::AddPitchToHistogram(float pitch) {
  return pitch_histogram_.Push(pitch);
}

bool LaneBasedCalibrator::GetPitchFromVanishingPoint(const VanishingPoint &vp,
                                                     float *pitch) const {
  assert(pitch != nullptr);
  const float cx = k_mat_[2];
  const float cy = k_mat_[5];
  const float fx = k_mat_[0];
  const float fy = k_mat_[4];
  float yaw_check = static_cast<float>(atan2(vp.pixel_pos[0] - cx, fx));
  if (fabs(yaw_check) > params_.max_allowed_yaw_angle_in_radian) {
    return false;
  }
  *pitch = static_cast<float>(atan2(vp.pixel_pos[1] - cy, fy));
  return true;
}

bool LaneBasedCalibrator::GetVanishingPoint(const EgoLane &lane,
                                            VanishingPoint *v_point) {
  assert(v_point != nullptr);
  float line_seg_l[4] = {0};
  float line_seg_r[4] = {0};

  // Get line segment
  bool get_line_seg_left =
      SelectTwoPointsFromLineForVanishingPoint(lane.left_line, line_seg_l);
  if (!get_line_seg_left) {
    AINFO << "Left lane is too short.";
    return false;
  }

  bool get_line_seg_right =
      SelectTwoPointsFromLineForVanishingPoint(lane.right_line, line_seg_r);
  if (!get_line_seg_right) {
    AINFO << "Right lane is too short.";
    return false;
  }

  // Get vanishing point by line segments intersection
  return GetIntersectionFromTwoLineSegments(line_seg_l, line_seg_r, v_point);
}

int LaneBasedCalibrator::GetCenterIndex(const Eigen::Vector2f *points,
                                        int nr_pts) const {
  assert(points != nullptr);
  if (nr_pts <= 0) {
    return -1;
  }
  float center_x = 0.0f;
  float center_y = 0.0f;
  for (int i = 0; i < nr_pts; ++i) {
    center_x += points[i](0);
    center_y += points[i](1);
  }
  center_x /= static_cast<float>(nr_pts);
  center_y /= static_cast<float>(nr_pts);
  float dist = 0.0f;
  float dist_min = static_cast<float>(fabs(points[0](0) - center_x) +
                                      fabs(points[0](1) - center_y));
  int center_index = 0;
  for (int i = 1; i < nr_pts; ++i) {
    dist = static_cast<float>(fabs(points[i](0) - center_x) +
                              fabs(points[i](1) - center_y));
    if (dist < dist_min) {
      dist_min = dist;
      center_index = i;
    }
  }
  return is_in_image(points[center_index]) ? center_index : -1;
}

bool LaneBasedCalibrator::SelectTwoPointsFromLineForVanishingPoint(
    const LaneLine &line, float line_seg[4]) {
  int nr_pts = static_cast<int>(line.lane_point.size());
  if (nr_pts < params_.min_nr_pts_laneline) {
    return false;
  }

  int nr_samples = nr_pts * static_cast<int>(params_.sampling_lane_point_rate);
  int offset_end = nr_pts - nr_samples - 1;
  int sampled_start = GetCenterIndex(line.lane_point.data(), nr_samples);
  int sampled_end =
      offset_end +
      GetCenterIndex(line.lane_point.data() + offset_end, nr_samples);

  if (sampled_start >= 0 && sampled_end >= 0) {
    line_seg[0] = line.lane_point[sampled_start](0);  // start
    line_seg[1] = line.lane_point[sampled_start](1);
    line_seg[2] = line.lane_point[sampled_end](0);  // end
    line_seg[3] = line.lane_point[sampled_end](1);
  } else {
    line_seg[0] = line.lane_point.front()(0);  // start
    line_seg[1] = line.lane_point.front()(1);
    line_seg[2] = line.lane_point.back()(0);  // end
    line_seg[3] = line.lane_point.back()(1);
  }

  // Ensure start is lower than end
  if (line_seg[3] > line_seg[1]) {
    std::swap(line_seg[0], line_seg[2]);
    std::swap(line_seg[1], line_seg[3]);
  }
  return true;
}

bool LaneBasedCalibrator::GetIntersectionFromTwoLineSegments(
    const float line_seg_l[4], const float line_seg_r[4],
    VanishingPoint *v_point) {
  assert(v_point != nullptr);
  // ref: https://stackoverflow.com/questions/563198/...
  // how-do-you-detect-where-two-line-segments-intersect/1968345#1968345
  // need further debug!

  // std::cout << "line seg from left:\n"
  //   << line_seg_l[0] << ", " << line_seg_l[1] << "\n"
  //   << line_seg_l[2] << ", " << line_seg_l[3] << std::endl;
  // std::cout << "line seg from right:\n"
  //   << line_seg_r[0] << ", " << line_seg_r[1] << "\n"
  //   << line_seg_r[2] << ", " << line_seg_r[3] << std::endl;
  /*
    end    end
    0      2
    ^      ^
    |      |
    |      |
    1      3
    start  start
  */

  float left_start_x = line_seg_l[0];
  float left_start_y = line_seg_l[1];
  float left_end_x = line_seg_l[2];
  float left_end_y = line_seg_l[3];

  float right_start_x = line_seg_r[0];
  float right_start_y = line_seg_r[1];
  float right_end_x = line_seg_r[2];
  float right_end_y = line_seg_r[3];

  float v10[2] = {left_end_x - left_start_x, left_end_y - left_start_y};
  float v32[2] = {right_end_x - right_start_x, right_end_y - right_start_y};
  float dn = v10[0] * v32[1] - v10[1] * v32[0];

  // Colinear
  if (fabs(dn) < 1e-5) {
    return false;
  }

  float v13[2] = {left_start_x - right_start_x, left_start_y - right_start_y};
  float t = v32[0] * v13[1] - v32[1] * v13[0];
  t *= common::IRec(dn);

  v_point->pixel_pos[0] = left_start_x + t * v10[0];
  v_point->pixel_pos[1] = left_start_y + t * v10[1];
  return true;
}

}  // namespace camera
}  // namespace perception
}  // namespace apollo
