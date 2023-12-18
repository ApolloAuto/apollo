/******************************************************************************
 * Copyright 2019 The Apollo Authors. All Rights Reserved.
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
#include "modules/perception/lane_detection/lib/postprocessor/darkSCNN/darkSCNN_lane_postprocessor.h"

#include <algorithm>
#include <map>
#include <utility>

#include "cyber/common/file.h"
#include "modules/perception/common/base/object_types.h"
#include "modules/perception/common/camera/common/math_functions.h"
#include "modules/perception/common/camera/common/timer.h"
#include "modules/perception/common/util.h"

namespace apollo {
namespace perception {
namespace camera {

// Lane image valid size
static const int kMaxValidSize = 3000;

std::vector<base::LaneLinePositionType> spatialLUT(
    {base::LaneLinePositionType::UNKNOWN,
     base::LaneLinePositionType::FOURTH_LEFT,
     base::LaneLinePositionType::THIRD_LEFT,
     base::LaneLinePositionType::ADJACENT_LEFT,
     base::LaneLinePositionType::EGO_LEFT,
     base::LaneLinePositionType::EGO_CENTER,
     base::LaneLinePositionType::EGO_RIGHT,
     base::LaneLinePositionType::ADJACENT_RIGHT,
     base::LaneLinePositionType::THIRD_RIGHT,
     base::LaneLinePositionType::FOURTH_RIGHT,
     base::LaneLinePositionType::OTHER, base::LaneLinePositionType::CURB_LEFT,
     base::LaneLinePositionType::CURB_RIGHT});

std::map<base::LaneLinePositionType, int> spatialLUTind = {
    {base::LaneLinePositionType::UNKNOWN, 0},
    {base::LaneLinePositionType::FOURTH_LEFT, 1},
    {base::LaneLinePositionType::THIRD_LEFT, 2},
    {base::LaneLinePositionType::ADJACENT_LEFT, 3},
    {base::LaneLinePositionType::EGO_LEFT, 4},
    {base::LaneLinePositionType::EGO_CENTER, 5},
    {base::LaneLinePositionType::EGO_RIGHT, 6},
    {base::LaneLinePositionType::ADJACENT_RIGHT, 7},
    {base::LaneLinePositionType::THIRD_RIGHT, 8},
    {base::LaneLinePositionType::FOURTH_RIGHT, 9},
    {base::LaneLinePositionType::OTHER, 10},
    {base::LaneLinePositionType::CURB_LEFT, 11},
    {base::LaneLinePositionType::CURB_RIGHT, 12}};

// @brief: evaluating y value of given x for a third-order polynomial function
template <typename T = float>
T GetPolyValue(T a, T b, T c, T d, T x) {
  T y = d;
  T v = x;
  y += (c * v);
  v *= x;
  y += (b * v);
  v *= x;
  y += (a * v);
  return y;
}

bool DarkSCNNLanePostprocessor::Init(
    const LanePostprocessorInitOptions& options) {
  // Read postprocessor parameter
  std::string config_file =
      GetConfigFile(options.config_path, options.config_file);
  if (!cyber::common::GetProtoFromFile(config_file,
                                       &lane_postprocessor_param_)) {
    AERROR << "Failed to read config detect_param: " << config_file;
    return false;
  }
  AINFO << "lane_postprocessor param: "
        << lane_postprocessor_param_.DebugString();

  input_offset_x_ = lane_postprocessor_param_.input_offset_x();
  input_offset_y_ = lane_postprocessor_param_.input_offset_y();
  lane_map_width_ = lane_postprocessor_param_.resize_width();
  lane_map_height_ = lane_postprocessor_param_.resize_height();
  DCHECK_LT(lane_map_width_, kMaxValidSize);
  DCHECK_GT(lane_map_width_, 0);
  DCHECK_LT(lane_map_height_, kMaxValidSize);
  DCHECK_GT(lane_map_height_, 0);

  roi_height_ = lane_postprocessor_param_.roi_height();
  roi_start_ = lane_postprocessor_param_.roi_start();
  roi_width_ = lane_postprocessor_param_.roi_width();

  lane_type_num_ = static_cast<int>(spatialLUTind.size());
  AINFO << "lane_type_num_: " << lane_type_num_;
  return true;
}

bool DarkSCNNLanePostprocessor::Process2D(
    const LanePostprocessorOptions& options, CameraFrame* frame) {
  ADEBUG << "Begin to Process2D.";
  frame->lane_objects.clear();
  auto start = std::chrono::high_resolution_clock::now();

  cv::Mat lane_map(lane_map_height_, lane_map_width_, CV_32FC1);
  memcpy(lane_map.data, frame->lane_detected_blob->cpu_data(),
         lane_map_width_ * lane_map_height_ * sizeof(float));

  // if (options.use_lane_history &&
  //     (!use_history_ || time_stamp_ > options.timestamp)) {
  //   InitLaneHistory();
  // }

  // 1. Sample points on lane_map and project them onto world coordinate

  // TODO(techoe): Should be fixed
  int y = static_cast<int>(lane_map.rows * 0.9 - 1);
  // TODO(techoe): Should be fixed
  int step_y = (y - 40) * (y - 40) / 6400 + 1;

  xy_points.clear();
  xy_points.resize(lane_type_num_);
  uv_points.clear();
  uv_points.resize(lane_type_num_);

  while (y > 0) {
    for (int x = 1; x < lane_map.cols - 1; ++x) {
      int value = static_cast<int>(round(lane_map.at<float>(y, x)));
      // lane on left

      if ((value > 0 && value < 5) || value == 11) {
        // right edge (inner) of the lane
        if (value != static_cast<int>(round(lane_map.at<float>(y, x + 1)))) {
          Eigen::Matrix<float, 3, 1> img_point(
              static_cast<float>(x * roi_width_ / lane_map.cols),
              static_cast<float>(y * roi_height_ / lane_map.rows + roi_start_),
              1.0);
          Eigen::Matrix<float, 3, 1> xy_p;
          xy_p = trans_mat_ * img_point;
          Eigen::Matrix<float, 2, 1> xy_point;
          Eigen::Matrix<float, 2, 1> uv_point;
          if (std::fabs(xy_p(2)) < 1e-6) continue;
          xy_point << xy_p(0) / xy_p(2), xy_p(1) / xy_p(2);

          // Filter out lane line points
          if (xy_point(0) < 0.0 ||  // This condition is only for front camera
              xy_point(0) > max_longitudinal_distance_ ||
              std::abs(xy_point(1)) > 30.0) {
            continue;
          }
          uv_point << static_cast<float>(x * roi_width_ / lane_map.cols),
              static_cast<float>(y * roi_height_ / lane_map.rows + roi_start_);
          if (xy_points[value].size() < minNumPoints_ || xy_point(0) < 50.0f ||
              std::fabs(xy_point(1) - xy_points[value].back()(1)) < 1.0f) {
            xy_points[value].push_back(xy_point);
            uv_points[value].push_back(uv_point);
          }
        }
      } else if (value >= 5 && value < lane_type_num_) {
        // Left edge (inner) of the lane
        if (value != static_cast<int>(round(lane_map.at<float>(y, x - 1)))) {
          Eigen::Matrix<float, 3, 1> img_point(
              static_cast<float>(x * roi_width_ / lane_map.cols),
              static_cast<float>(y * roi_height_ / lane_map.rows + roi_start_),
              1.0);
          Eigen::Matrix<float, 3, 1> xy_p;
          xy_p = trans_mat_ * img_point;
          Eigen::Matrix<float, 2, 1> xy_point;
          Eigen::Matrix<float, 2, 1> uv_point;
          if (std::fabs(xy_p(2)) < 1e-6) continue;
          xy_point << xy_p(0) / xy_p(2), xy_p(1) / xy_p(2);
          // Filter out lane line points
          if (xy_point(0) < 0.0 ||  // This condition is only for front camera
              xy_point(0) > max_longitudinal_distance_ ||
              std::abs(xy_point(1)) > 30.0) {
            continue;
          }
          uv_point << static_cast<float>(x * roi_width_ / lane_map.cols),
              static_cast<float>(y * roi_height_ / lane_map.rows + roi_start_);
          if (xy_points[value].size() < minNumPoints_ || xy_point(0) < 50.0f ||
              std::fabs(xy_point(1) - xy_points[value].back()(1)) < 1.0f) {
            xy_points[value].push_back(xy_point);
            uv_points[value].push_back(uv_point);
          }
        } else if (value >= lane_type_num_) {
          AWARN << "Lane line value shouldn't be equal or more than: "
                << lane_type_num_;
        }
      }
    }
    step_y = (y - 45) * (y - 45) / 6400 + 1;
    y -= step_y;
  }

  auto elapsed_1 = std::chrono::high_resolution_clock::now() - start;
  int64_t microseconds_1 =
      std::chrono::duration_cast<std::chrono::microseconds>(elapsed_1).count();
  time_1 += microseconds_1;

  // 2. Remove outliers and Do a ransac fitting
  EigenVector<Eigen::Matrix<float, 4, 1>> coeffs;
  EigenVector<Eigen::Matrix<float, 4, 1>> img_coeffs;
  EigenVector<Eigen::Matrix<float, 2, 1>> selected_xy_points;
  coeffs.resize(lane_type_num_);
  img_coeffs.resize(lane_type_num_);
  for (int i = 1; i < lane_type_num_; ++i) {
    coeffs[i] << 0, 0, 0, 0;
    if (xy_points[i].size() < minNumPoints_) continue;
    Eigen::Matrix<float, 4, 1> coeff;
    // Solve linear system to estimate polynomial coefficients
    if (RansacFitting<float>(xy_points[i], &selected_xy_points, &coeff, 200,
                             static_cast<int>(minNumPoints_), 0.1f)) {
      coeffs[i] = coeff;

      xy_points[i].clear();
      xy_points[i] = selected_xy_points;
    } else {
      xy_points[i].clear();
    }
  }

  auto elapsed_2 = std::chrono::high_resolution_clock::now() - start;
  int64_t microseconds_2 =
      std::chrono::duration_cast<std::chrono::microseconds>(elapsed_2).count();
  time_2 += microseconds_2 - microseconds_1;

  // 3. Write values into lane_objects
  std::vector<float> c0s(lane_type_num_, 0);
  for (int i = 1; i < lane_type_num_; ++i) {
    if (xy_points[i].size() < minNumPoints_) continue;
    c0s[i] = GetPolyValue(
        static_cast<float>(coeffs[i](3)), static_cast<float>(coeffs[i](2)),
        static_cast<float>(coeffs[i](1)), static_cast<float>(coeffs[i](0)),
        static_cast<float>(3.0));
  }
  // [1] Determine lane spatial tag in special cases
  if (xy_points[4].size() < minNumPoints_ &&
      xy_points[5].size() >= minNumPoints_) {
    std::swap(xy_points[4], xy_points[5]);
    std::swap(uv_points[4], uv_points[5]);
    std::swap(coeffs[4], coeffs[5]);
    std::swap(c0s[4], c0s[5]);
  } else if (xy_points[6].size() < minNumPoints_ &&
             xy_points[5].size() >= minNumPoints_) {
    std::swap(xy_points[6], xy_points[5]);
    std::swap(uv_points[6], uv_points[5]);
    std::swap(coeffs[6], coeffs[5]);
    std::swap(c0s[6], c0s[5]);
  }

  if (xy_points[4].size() < minNumPoints_ &&
      xy_points[11].size() >= minNumPoints_) {
    // Use left lane boundary as the right most missing left lane,
    bool use_boundary = true;
    for (int k = 3; k >= 1; --k) {
      if (xy_points[k].size() >= minNumPoints_) {
        use_boundary = false;
        break;
      }
    }
    if (use_boundary) {
      std::swap(xy_points[4], xy_points[11]);
      std::swap(uv_points[4], uv_points[11]);
      std::swap(coeffs[4], coeffs[11]);
      std::swap(c0s[4], c0s[11]);
    }
  }

  if (xy_points[6].size() < minNumPoints_ &&
      xy_points[12].size() >= minNumPoints_) {
    // Use right lane boundary as the left most missing right lane,
    bool use_boundary = true;
    for (int k = 7; k <= 9; ++k) {
      if (xy_points[k].size() >= minNumPoints_) {
        use_boundary = false;
        break;
      }
    }
    if (use_boundary) {
      std::swap(xy_points[6], xy_points[12]);
      std::swap(uv_points[6], uv_points[12]);
      std::swap(coeffs[6], coeffs[12]);
      std::swap(c0s[6], c0s[12]);
    }
  }

  for (int i = 1; i < lane_type_num_; ++i) {
    base::LaneLine cur_object;
    if (xy_points[i].size() < minNumPoints_) {
      continue;
    }

    // [2] Set spatial label
    cur_object.pos_type = spatialLUT[i];

    // [3] Determine which lines are valid according to the y value at x = 3
    if ((i < 5 && c0s[i] < c0s[i + 1]) ||
        (i > 5 && i < 10 && c0s[i] > c0s[i - 1])) {
      continue;
    }
    if (i == 11 || i == 12) {
      std::sort(c0s.begin(), c0s.begin() + 10);
      if ((c0s[i] > c0s[0] && i == 12) || (c0s[i] < c0s[9] && i == 11)) {
        continue;
      }
    }
    // [4] Write values
    cur_object.curve_car_coord.x_start =
        static_cast<float>(xy_points[i].front()(0));
    cur_object.curve_car_coord.x_end =
        static_cast<float>(xy_points[i].back()(0));
    cur_object.curve_car_coord.a = static_cast<float>(coeffs[i](3));
    cur_object.curve_car_coord.b = static_cast<float>(coeffs[i](2));
    cur_object.curve_car_coord.c = static_cast<float>(coeffs[i](1));
    cur_object.curve_car_coord.d = static_cast<float>(coeffs[i](0));
    // if (cur_object.curve_car_coord.x_end -
    //     cur_object.curve_car_coord.x_start < 5) continue;
    // cur_object.order = 2;
    cur_object.curve_car_coord_point_set.clear();
    for (size_t j = 0; j < xy_points[i].size(); ++j) {
      base::Point2DF p_j;
      p_j.x = static_cast<float>(xy_points[i][j](0));
      p_j.y = static_cast<float>(xy_points[i][j](1));
      cur_object.curve_car_coord_point_set.push_back(p_j);
    }

    cur_object.curve_image_point_set.clear();
    for (size_t j = 0; j < uv_points[i].size(); ++j) {
      base::Point2DF p_j;
      p_j.x = static_cast<float>(uv_points[i][j](0));
      p_j.y = static_cast<float>(uv_points[i][j](1));
      cur_object.curve_image_point_set.push_back(p_j);
    }

    // cur_object.confidence.push_back(1);
    cur_object.confidence = 1.0f;
    frame->lane_objects.push_back(cur_object);
  }

  // Special case riding on a lane:
  // 0: no center lane, 1: center lane as left, 2: center lane as right
  int has_center_ = 0;
  for (auto lane_ : frame->lane_objects) {
    if (lane_.pos_type == base::LaneLinePositionType::EGO_CENTER) {
      if (lane_.curve_car_coord.d >= 0) {
        has_center_ = 1;
      } else if (lane_.curve_car_coord.d < 0) {
        has_center_ = 2;
      }
      break;
    }
  }
  // Change labels for all lanes from one side
  if (has_center_ == 1) {
    for (auto& lane_ : frame->lane_objects) {
      int spatial_id = spatialLUTind[lane_.pos_type];
      if (spatial_id >= 1 && spatial_id <= 5) {
        lane_.pos_type = spatialLUT[spatial_id - 1];
      }
    }
  } else if (has_center_ == 2) {
    for (auto& lane_ : frame->lane_objects) {
      int spatial_id = spatialLUTind[lane_.pos_type];
      if (spatial_id >= 5 && spatial_id <= 9) {
        lane_.pos_type = spatialLUT[spatial_id + 1];
      }
    }
  }

  auto elapsed = std::chrono::high_resolution_clock::now() - start;
  int64_t microseconds =
      std::chrono::duration_cast<std::chrono::microseconds>(elapsed).count();
  // AINFO << "Time for writing: " << microseconds - microseconds_2 << " us";
  time_3 += microseconds - microseconds_2;
  ++time_num;

  ADEBUG << "frame->lane_objects.size(): " << frame->lane_objects.size();

  ADEBUG << "Avg sampling time: " << time_1 / time_num
         << " Avg fitting time: " << time_2 / time_num
         << " Avg writing time: " << time_3 / time_num;
  ADEBUG << "darkSCNN lane_postprocess done!";
  return true;
}

// Produce laneline output in camera coordinates (optional)
bool DarkSCNNLanePostprocessor::Process3D(
    const LanePostprocessorOptions& options, CameraFrame* frame) {
  ConvertImagePoint2Camera(frame);
  PolyFitCameraLaneline(frame);
  return true;
}

void DarkSCNNLanePostprocessor::ConvertImagePoint2Camera(CameraFrame* frame) {
  float pitch_angle = frame->calibration_service->QueryPitchAngle();
  float camera_ground_height =
      frame->calibration_service->QueryCameraToGroundHeight();
  const Eigen::Matrix3f& intrinsic_params = frame->camera_k_matrix;
  const Eigen::Matrix3f& intrinsic_params_inverse = intrinsic_params.inverse();
  std::vector<base::LaneLine>& lane_objects = frame->lane_objects;
  int laneline_num = static_cast<int>(lane_objects.size());
  for (int line_index = 0; line_index < laneline_num; ++line_index) {
    std::vector<base::Point2DF>& image_point_set =
        lane_objects[line_index].curve_image_point_set;
    std::vector<base::Point3DF>& camera_point_set =
        lane_objects[line_index].curve_camera_point_set;
    for (int i = 0; i < static_cast<int>(image_point_set.size()); i++) {
      base::Point3DF camera_point;
      Eigen::Vector3d camera_point3d;
      const base::Point2DF& image_point = image_point_set[i];
      ImagePoint2Camera(image_point, pitch_angle, camera_ground_height,
                        intrinsic_params_inverse, &camera_point3d);
      camera_point.x = static_cast<float>(camera_point3d(0));
      camera_point.y = static_cast<float>(camera_point3d(1));
      camera_point.z = static_cast<float>(camera_point3d(2));
      camera_point_set.push_back(camera_point);
    }
  }
}

// @brief: Fit camera lane line using polynomial
void DarkSCNNLanePostprocessor::PolyFitCameraLaneline(CameraFrame* frame) {
  std::vector<base::LaneLine>& lane_objects = frame->lane_objects;
  int laneline_num = static_cast<int>(lane_objects.size());
  for (int line_index = 0; line_index < laneline_num; ++line_index) {
    const std::vector<base::Point3DF>& camera_point_set =
        lane_objects[line_index].curve_camera_point_set;
    // z: longitudinal direction
    // x: latitudinal direction
    float x_start = camera_point_set[0].z;
    float x_end = 0.0f;
    Eigen::Matrix<float, max_poly_order + 1, 1> camera_coeff;
    EigenVector<Eigen::Matrix<float, 2, 1>> camera_pos_vec;
    for (int i = 0; i < static_cast<int>(camera_point_set.size()); ++i) {
      x_end = std::max(camera_point_set[i].z, x_end);
      x_start = std::min(camera_point_set[i].z, x_start);
      Eigen::Matrix<float, 2, 1> camera_pos;
      camera_pos << camera_point_set[i].z, camera_point_set[i].x;
      camera_pos_vec.push_back(camera_pos);
    }

    bool is_x_axis = true;
    bool fit_flag =
        PolyFit(camera_pos_vec, max_poly_order, &camera_coeff, is_x_axis);
    if (!fit_flag) {
      continue;
    }
    lane_objects[line_index].curve_camera_coord.a = camera_coeff(3, 0);
    lane_objects[line_index].curve_camera_coord.b = camera_coeff(2, 0);
    lane_objects[line_index].curve_camera_coord.c = camera_coeff(1, 0);
    lane_objects[line_index].curve_camera_coord.d = camera_coeff(0, 0);
    lane_objects[line_index].curve_camera_coord.x_start = x_start;
    lane_objects[line_index].curve_camera_coord.x_end = x_end;
    lane_objects[line_index].use_type = base::LaneLineUseType::REAL;
  }
}

std::string DarkSCNNLanePostprocessor::Name() const {
  return "DarkSCNNLanePostprocessor";
}

REGISTER_LANE_POSTPROCESSOR(DarkSCNNLanePostprocessor);
}  // namespace camera
}  // namespace perception
}  // namespace apollo
