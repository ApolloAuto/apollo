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
#include "modules/perception/lane_detection/lib/postprocessor/denseline/denseline_lane_postprocessor.h"

#include <algorithm>

#include "cyber/common/file.h"
#include "cyber/common/log.h"
#include "modules/common/util/eigen_defs.h"
#include "modules/perception/common/base/object_types.h"
#include "modules/perception/common/camera/common/math_functions.h"
#include "modules/perception/common/util.h"

namespace apollo {
namespace perception {
namespace camera {

using apollo::common::EigenVector;

bool DenselineLanePostprocessor::Init(
    const LanePostprocessorInitOptions& options) {
  //  read postprocessor parameter
  std::string config_file =
      GetConfigFile(options.config_path, options.config_file);
  if (!cyber::common::GetProtoFromFile(config_file,
                                       &lane_postprocessor_param_)) {
    AERROR << "Read config detect_param failed: " << config_file;
    return false;
  }

  AINFO << "lane_postprocessor param: "
        << lane_postprocessor_param_.DebugString();
  input_offset_x_ = lane_postprocessor_param_.input_offset_x();
  input_offset_y_ = lane_postprocessor_param_.input_offset_y();
  input_crop_height_ = lane_postprocessor_param_.crop_height();
  input_crop_width_ = lane_postprocessor_param_.crop_width();

  omit_bottom_line_num_ = lane_postprocessor_param_.omit_bottom_line_num();
  laneline_map_score_thresh_ =
      lane_postprocessor_param_.laneline_map_score_thresh();
  laneline_point_score_thresh_ =
      lane_postprocessor_param_.laneline_point_score_thresh();
  laneline_point_min_num_thresh_ =
      lane_postprocessor_param_.laneline_point_min_num_thresh();
  cc_valid_pixels_ratio_ = lane_postprocessor_param_.cc_valid_pixels_ratio();
  laneline_reject_dist_thresh_ =
      lane_postprocessor_param_.laneline_reject_dist_thresh();

  lane_map_dim_ = lane_map_width_ * lane_map_height_;
  lane_pos_blob_.Reshape({4, lane_map_dim_});
  lane_hist_blob_.Reshape({2, lane_map_dim_});
  return true;
}

bool DenselineLanePostprocessor::Process2D(
    const LanePostprocessorOptions& options, CameraFrame* frame) {
  frame->lane_objects.clear();
  // 1. locate the lane line point set
  bool flag = LocateLanelinePointSet(frame);
  if (!flag) {
    return true;
  }

  //  2. classify the lane line pos type
  std::vector<base::LaneLinePositionType> line_pos_type_vec(
      image_group_point_set_.size());
  std::vector<bool> line_flag(image_group_point_set_.size(), false);
  ClassifyLanelinePosTypeInImage(image_group_point_set_, &line_pos_type_vec,
                                 &line_flag);
  base::LaneLineType line_type = base::LaneLineType::WHITE_SOLID;
  for (int line_index = 0;
       line_index < static_cast<int>(image_group_point_set_.size());
       line_index++) {
    if (!line_flag[line_index]) {
      ADEBUG << "line_pos_type is invalid";
      continue;
    }
    AddImageLaneline(image_group_point_set_[line_index], line_type,
                     line_pos_type_vec[line_index], line_index,
                     &(frame->lane_objects));
  }
  AINFO << "[AfterProcess2D]lane_lines_num: " << frame->lane_objects.size();

  return true;
}

bool DenselineLanePostprocessor::Process3D(
    const LanePostprocessorOptions& options, CameraFrame* frame) {
  ConvertImagePoint2Camera(frame);
  PolyFitCameraLaneline(frame);
  return true;
}

void DenselineLanePostprocessor::ConvertImagePoint2Camera(CameraFrame* frame) {
  float pitch_angle = frame->calibration_service->QueryPitchAngle();
  float camera_ground_height =
      frame->calibration_service->QueryCameraToGroundHeight();
  const Eigen::Matrix3f& intrinsic_params = frame->camera_k_matrix;
  const Eigen::Matrix3f& intrinsic_params_inverse = intrinsic_params.inverse();
  std::vector<base::LaneLine>& lane_objects = frame->lane_objects;
  int laneline_num = static_cast<int>(lane_objects.size());
  for (int line_index = 0; line_index < laneline_num; line_index++) {
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

void DenselineLanePostprocessor::CalLaneMap(
    const float* output_data, int width, int height,
    std::vector<unsigned char>* lane_map) {
  int out_dim = width * height;
  for (int y = 0; y < height - omit_bottom_line_num_; y++) {
    float score_channel[4];
    int row_start = y * width;
    int channel0_pos = row_start;
    int channel1_pos = out_dim + row_start;
    int channel2_pos = 2 * out_dim + row_start;
    int channel3_pos = 3 * out_dim + row_start;
    int channel5_pos = 5 * out_dim + row_start;
    int channel6_pos = 6 * out_dim + row_start;

    for (int x = 0; x < width; ++x) {
      score_channel[0] = output_data[channel0_pos + x];
      score_channel[1] = output_data[channel1_pos + x];
      score_channel[2] = output_data[channel2_pos + x];
      score_channel[3] = output_data[channel3_pos + x];
      // Utilize softmax to get the probability
      float sum_score = 0.0f;
      for (int i = 0; i < 4; i++) {
        score_channel[i] = static_cast<float>(exp(score_channel[i]));
        sum_score += score_channel[i];
      }
      for (int i = 0; i < 4; i++) {
        score_channel[i] /= sum_score;
      }
      // 1: ego-lane; 2: adj-left lane; 3: adj-right lane
      //  find the score with max lane map
      int max_channel_idx = 0;
      float max_score = score_channel[0];
      for (int channel_idx = 1; channel_idx < 4; channel_idx++) {
        if (max_score < score_channel[channel_idx]) {
          max_score = score_channel[channel_idx];
          max_channel_idx = channel_idx;
        }
      }
      //  if the channel 0 has the maximum probability
      //  or the score is less than the setting value
      //  omit it
      if (max_channel_idx == 0 || max_score < laneline_map_score_thresh_) {
        continue;
      }
      int pixel_pos = row_start + x;
      (*lane_map)[pixel_pos] = 1;

      float dist_left = sigmoid(output_data[channel5_pos + x]);
      float dist_right = sigmoid(output_data[channel6_pos + x]);
      lane_output_[pixel_pos] = dist_left;
      lane_output_[out_dim + pixel_pos] = dist_right;
      lane_output_[out_dim * 2 + pixel_pos] = max_score;
    }
  }
}

// @brief infer the lane line points using lane center point information
void DenselineLanePostprocessor::InferPointSetFromLaneCenter(
    const std::vector<ConnectedComponent>& lane_ccs,
    const std::vector<LaneType>& ccs_pos_type,
    std::vector<std::vector<LanePointInfo>>* lane_map_group_point_set) {
  //  0: adj-left lane center
  //  1: ego-lane center;
  //  2: adj-right lane center
  int map_dim = lane_map_width_ * lane_map_height_;
  if (lane_map_dim_ != map_dim) {
    lane_map_dim_ = map_dim;
    lane_pos_blob_.Reshape({4, lane_map_dim_});
    lane_hist_blob_.Reshape({2, lane_map_dim_});
  }

  for (int i = 0; i < static_cast<int>(lane_ccs.size()); i++) {
    int left_index = -1;
    int right_index = -1;
    if (ccs_pos_type[i] == LaneType::EGO_LANE) {
      left_index = 1;
      right_index = 2;
    } else if (ccs_pos_type[i] == LaneType::ADJACENT_LEFT_LANE) {
      left_index = 0;
    } else if (ccs_pos_type[i] == LaneType::ADJACENT_RIGHT_LANE) {
      right_index = 3;
    }
    InferPointSetFromOneCC(lane_ccs[i], left_index, right_index,
                           lane_map_group_point_set);
  }
}

// @brief infer the lane line points from one CC
void DenselineLanePostprocessor::InferPointSetFromOneCC(
    const ConnectedComponent& lane_cc, int left_index, int right_index,
    std::vector<std::vector<LanePointInfo>>* lane_map_group_point_set) {
  //  find the points which belongs to this CC
  const std::vector<base::Point2DI>& pixels = lane_cc.GetPixels();
  //  initialize the memory
  float* pos_blob_head = lane_pos_blob_.mutable_cpu_data();
  float* score_left_vec = pos_blob_head;
  float* x_left_vec = pos_blob_head + lane_pos_blob_.offset(1);
  float* score_right_vec = pos_blob_head + lane_pos_blob_.offset(2);
  float* x_right_vec = pos_blob_head + lane_pos_blob_.offset(3);

  int* hist_blob_head = lane_hist_blob_.mutable_cpu_data();
  int* x_left_count = hist_blob_head;
  int* x_right_count = hist_blob_head + lane_hist_blob_.offset(1);
  memset(lane_pos_blob_.mutable_cpu_data(), 0,
         sizeof(float) * lane_pos_blob_.count());
  memset(lane_hist_blob_.mutable_cpu_data(), 0,
         sizeof(int) * lane_hist_blob_.count());

  int lane_map_dim = lane_map_width_ * lane_map_height_;
  for (int j = 0; j < static_cast<int>(pixels.size()); j++) {
    int pixel_x = pixels[j].x;
    int pixel_y = pixels[j].y;
    int pixel_pos = pixel_y * lane_map_width_ + pixel_x;

    float score = lane_output_[lane_map_dim * 2 + pixel_pos];
    float dist_left = lane_output_[pixel_pos];
    float x_left_map = static_cast<float>(pixel_x) -
                       dist_left * static_cast<float>(lane_map_width_);
    int x_left = static_cast<int>(x_left_map);
    if (left_index != -1 && x_left > 0 && x_left < lane_map_width_ - 1) {
      int pos_left = pixel_y * lane_map_width_ + x_left;
      score_left_vec[pos_left] += score;
      x_left_vec[pos_left] += x_left_map;
      ++x_left_count[pos_left];
    }

    float dist_right = lane_output_[lane_map_dim + pixel_pos];
    float x_right_map = static_cast<float>(pixel_x) +
                        dist_right * static_cast<float>(lane_map_width_);
    int x_right = static_cast<int>(x_right_map);
    if (right_index != -1 && x_right < lane_map_width_ - 1 && x_right > 0) {
      int pos_right = pixel_y * lane_map_width_ + x_right;
      score_right_vec[pos_right] += score;
      x_right_vec[pos_right] += x_right_map;
      ++x_right_count[pos_right];
    }
  }

  const base::BBox2DI& bbox = lane_cc.GetBBox();
  int ymax = bbox.ymax;
  int ymin = bbox.ymin;
  for (int j = ymin; j <= ymax; j++) {
    int start_pos = j * lane_map_width_;
    // Find the position with maximum value for left side
    if (left_index != -1) {
      float* score_left_pointer = score_left_vec + start_pos;
      float* x_left_pointer = x_left_vec + start_pos;
      int* x_left_count_pointer = x_left_count + start_pos;
      LanePointInfo left_point;
      bool left_flag = MaxScorePoint(score_left_pointer, x_left_pointer,
                                     x_left_count_pointer, j, &left_point);
      if (left_flag) {
        (*lane_map_group_point_set)[left_index].push_back(left_point);
      }
    }

    //  find the position with maximum value for right side
    if (right_index != -1) {
      float* score_right_pointer = score_right_vec + start_pos;
      float* x_right_pointer = x_right_vec + start_pos;
      int* x_right_count_pointer = x_right_count + start_pos;
      LanePointInfo right_point;
      bool right_flag = MaxScorePoint(score_right_pointer, x_right_pointer,
                                      x_right_count_pointer, j, &right_point);
      if (right_flag) {
        (*lane_map_group_point_set)[right_index].push_back(right_point);
      }
    }
  }
}

bool DenselineLanePostprocessor::MaxScorePoint(const float* score_pointer,
                                               const float* x_pointer,
                                               const int* x_count_pointer,
                                               int y_pos,
                                               LanePointInfo* point_info) {
  int large_index[2];
  bool flag = FindKLargeValue(score_pointer, lane_map_width_, 2, large_index);
  if (!flag) {
    return false;
  }
  int max_x = large_index[0];
  float max_score = score_pointer[large_index[0]];
  if (max_score <= laneline_point_score_thresh_) {
    return false;
  }
  (*point_info).x =
      x_pointer[max_x] / static_cast<float>(x_count_pointer[max_x]);
  (*point_info).y = static_cast<float>(y_pos);
  (*point_info).score = max_score / static_cast<float>(x_count_pointer[max_x]);
  return true;
}

bool DenselineLanePostprocessor::SelectLanecenterCCs(
    const std::vector<ConnectedComponent>& lane_ccs,
    std::vector<ConnectedComponent>* select_lane_ccs) {
  select_lane_ccs->clear();
  int lane_ccs_num = static_cast<int>(lane_ccs.size());
  if (lane_ccs_num == 0) {
    AINFO << "lane_ccs_num is 0.";
    return false;
  }
  //  select top 3 ccs with largest pixels size
  int valid_pixels_num = static_cast<int>(cc_valid_pixels_ratio_ *
                                          static_cast<float>(lane_map_height_));
  std::vector<ConnectedComponent> valid_lane_ccs;
  for (int i = 0; i < lane_ccs_num; i++) {
    const std::vector<base::Point2DI>& pixels = lane_ccs[i].GetPixels();
    if (static_cast<int>(pixels.size()) < valid_pixels_num) {
      AINFO << "pixels_size < valid_pixels_num.";
      continue;
    }
    valid_lane_ccs.push_back(lane_ccs[i]);
  }
  int valid_ccs_num = static_cast<int>(valid_lane_ccs.size());
  if (valid_ccs_num == 0) {
    AINFO << "valid_ccs_num is 0.";
    return false;
  }
  std::sort(valid_lane_ccs.begin(), valid_lane_ccs.end(), CompareCCSize);
  int select_cc_num = std::min(valid_ccs_num, 3);
  for (int i = 0; i < select_cc_num; i++) {
    select_lane_ccs->push_back(valid_lane_ccs[i]);
  }
  return true;
}

// @brief: locate lane line points
bool DenselineLanePostprocessor::LocateLanelinePointSet(
    const CameraFrame* frame) {
  //  find laneline_point center_point of each row
  //  0:adj-left 1:ego-left 2:ego-right 3:adj-right
  //  1.get the lane points in the feature map
  auto data_provider = frame->data_provider;
  input_image_width_ = data_provider->src_width();
  input_image_height_ = data_provider->src_height();
  std::vector<int> out_put_shape = frame->lane_detected_blob->shape();
  int channels = frame->lane_detected_blob->channels();
  if (channels < net_model_channel_num_) {
    AERROR << "channel (" << channels << ") is less than net channel ("
           << net_model_channel_num_ << ")";
    return false;
  }

  lane_map_height_ = frame->lane_detected_blob->height();
  lane_map_width_ = frame->lane_detected_blob->width();
  const float* output_data = frame->lane_detected_blob->cpu_data();
  ADEBUG << "input_size: [" << input_image_width_ << "," << input_image_height_
         << "] "
         << "output_shape: channels=" << channels
         << " height=" << lane_map_height_ << " width=" << lane_map_width_;

  image_group_point_set_.clear();
  image_group_point_set_.resize(4);

  int out_dim = lane_map_width_ * lane_map_height_;
  lane_map_.clear();
  lane_output_.clear();
  lane_map_.resize(out_dim, 0);
  lane_output_.resize(out_dim * 3, 0);
  CalLaneMap(output_data, lane_map_width_, lane_map_height_, &lane_map_);
  //  2.group the lane points
  base::RectI roi;
  roi.x = 0;
  roi.y = 0;
  roi.width = lane_map_width_;
  roi.height = lane_map_height_;
  lane_ccs_.clear();
  bool flag =
      FindCC(lane_map_, lane_map_width_, lane_map_height_, roi, &lane_ccs_);
  if (!flag) {
    return false;
  }

  //  3. select lane center ccs
  flag = SelectLanecenterCCs(lane_ccs_, &select_lane_ccs_);
  if (!flag) {
    return false;
  }

  //  4. Classify the lane_ccs_ type
  std::vector<LaneType> ccs_pos_type(select_lane_ccs_.size(),
                                     LaneType::UNKNOWN_LANE);
  flag = ClassifyLaneCCsPosTypeInImage(select_lane_ccs_, &ccs_pos_type);
  if (!flag) {
    return false;
  }

  //  5. get the lane line points
  std::vector<std::vector<LanePointInfo>> lane_map_group_point_set(4);
  InferPointSetFromLaneCenter(select_lane_ccs_, ccs_pos_type,
                              &lane_map_group_point_set);

  //  6. convert to the original image
  Convert2OriginalCoord(lane_map_group_point_set, &image_group_point_set_);
  return true;
}

// @brief: classify lane ccs type in image domain
bool DenselineLanePostprocessor::ClassifyLaneCCsPosTypeInImage(
    const std::vector<ConnectedComponent>& select_lane_ccs,
    std::vector<LaneType>* ccs_pos_type) {
  //
  int ccs_num = static_cast<int>(select_lane_ccs.size());
  std::vector<float> cc_bottom_center_x(ccs_num);
  float min_dist = 1e6;
  int min_index = -1;
  float lane_map_center_x = static_cast<float>(lane_map_width_ >> 1);
  for (int i = 0; i < ccs_num; i++) {
    const std::vector<base::Point2DI>& pixels = select_lane_ccs[i].GetPixels();
    const base::BBox2DI& bbox = select_lane_ccs[i].GetBBox();
    int x_min = -1;
    int x_max = -1;
    int y_max = bbox.ymax;
    for (int j = 0; j < static_cast<int>(pixels.size()); j++) {
      int pixel_y = pixels[j].y;
      if (pixel_y != y_max) {
        continue;
      }
      int pixel_x = pixels[j].x;
      if (x_min == -1) {
        x_min = pixel_x;
      } else {
        x_min = std::min(x_min, pixel_x);
      }
      if (x_max == -1) {
        x_max = pixel_x;
      } else {
        x_max = std::max(x_max, pixel_x);
      }
    }
    cc_bottom_center_x[i] = static_cast<float>(x_min + x_max) / 2.0f;
    float dist =
        static_cast<float>(fabs(cc_bottom_center_x[i] - lane_map_center_x));
    if (dist < min_dist) {
      min_dist = dist;
      min_index = i;
    }
  }
  if (min_index == -1) {
    AERROR << "min_index=-1";
    return false;
  }
  //  0: ego-lane
  //  1: adj-left lane
  //  2: adj-right lane
  float center_x = cc_bottom_center_x[min_index];
  // ego lane
  (*ccs_pos_type)[min_index] = LaneType::EGO_LANE;
  for (int i = 0; i < ccs_num; i++) {
    if (i == min_index) {
      continue;
    }
    if (cc_bottom_center_x[i] > center_x) {
      //  adj-right lane
      (*ccs_pos_type)[i] = LaneType::ADJACENT_RIGHT_LANE;
    } else if (cc_bottom_center_x[i] < center_x) {
      //  adj-left lane
      (*ccs_pos_type)[i] = LaneType::ADJACENT_LEFT_LANE;
    }
  }
  return true;
}

// @brief classify lane line pos type in image
// [adj-left/ego-left/ego-right/adj-right]
void DenselineLanePostprocessor::ClassifyLanelinePosTypeInImage(
    const std::vector<std::vector<LanePointInfo>>& image_group_point_set,
    std::vector<base::LaneLinePositionType>* laneline_type,
    std::vector<bool>* line_flag) {
  int set_size = static_cast<int>(image_group_point_set.size());
  std::vector<float> latitude_intersection(set_size, lane_max_value_);

  int ego_left_index = -1;
  int ego_right_index = -1;
  float ego_left_value = -1e6;
  float ego_right_value = 1e6;
  float lane_center_pos = static_cast<float>(input_image_width_ >> 1);
  for (int i = 0; i < set_size; i++) {
    int point_num = static_cast<int>(image_group_point_set[i].size());
    if (point_num <= laneline_point_min_num_thresh_) {
      continue;
    }
    float fx0 = image_group_point_set[i][point_num - 2].x;
    float fy0 = image_group_point_set[i][point_num - 2].y;
    float fx1 = image_group_point_set[i][point_num - 1].x;
    float fy1 = image_group_point_set[i][point_num - 1].y;
    float dif_x = fx1 - fx0;
    float dif_y = fy1 - fy0;
    if (fabs(dif_y) < 1e-3) {
      latitude_intersection[i] = fx1;
    } else {
      float fk = dif_x / dif_y;
      float fb = fx1 - fk * fy1;
      latitude_intersection[i] =
          fk * static_cast<float>(input_image_height_ - 1) + fb;
    }
    if (latitude_intersection[i] <= lane_center_pos &&
        latitude_intersection[i] > ego_left_value) {
      ego_left_value = latitude_intersection[i];
      ego_left_index = i;
    }
    if (latitude_intersection[i] > lane_center_pos &&
        latitude_intersection[i] < ego_right_value) {
      ego_right_value = latitude_intersection[i];
      ego_right_index = i;
    }
  }
  if (ego_left_index != -1) {
    (*laneline_type)[ego_left_index] = base::LaneLinePositionType::EGO_LEFT;
    (*line_flag)[ego_left_index] = true;
  }
  if (ego_right_index != -1) {
    (*laneline_type)[ego_right_index] = base::LaneLinePositionType::EGO_RIGHT;
    (*line_flag)[ego_right_index] = true;
  }
  // locate adjacent left laneline
  int adj_left_index = -1;
  LocateNeighborLaneLine(latitude_intersection, ego_left_index, true,
                         &adj_left_index);
  if (adj_left_index != -1) {
    (*laneline_type)[adj_left_index] =
        base::LaneLinePositionType::ADJACENT_LEFT;
    (*line_flag)[adj_left_index] = true;
  }
  // locate adjacent right laneline
  int adj_right_index = -1;
  LocateNeighborLaneLine(latitude_intersection, ego_right_index, false,
                         &adj_right_index);
  if (adj_right_index != -1) {
    (*laneline_type)[adj_right_index] =
        base::LaneLinePositionType::ADJACENT_RIGHT;
    (*line_flag)[adj_right_index] = true;
  }
}

// @brief: locate neighbor lane lines
bool DenselineLanePostprocessor::LocateNeighborLaneLine(
    const std::vector<float>& latitude_intersection, int line_index,
    bool left_flag, int* locate_index) {
  // left_flag = true: find the line which is at left side of the line
  // left_flag = false: find the line which is at right side of the line
  int set_size = static_cast<int>(latitude_intersection.size());
  if (line_index < 0 || line_index >= set_size) {
    return false;
  }
  float intersection_x = latitude_intersection[line_index];
  if (left_flag) {
    float max_value = -1e6;
    for (int i = 0; i < set_size; i++) {
      if (latitude_intersection[i] >= intersection_x ||
          latitude_intersection[i] >= lane_max_value_) {
        continue;
      }
      if (latitude_intersection[i] > max_value) {
        max_value = latitude_intersection[i];
        *locate_index = i;
      }
    }
  } else {
    float min_value = 1e6;
    for (int i = 0; i < set_size; i++) {
      if (latitude_intersection[i] <= intersection_x ||
          latitude_intersection[i] >= lane_max_value_) {
        continue;
      }
      if (latitude_intersection[i] < min_value) {
        min_value = latitude_intersection[i];
        *locate_index = i;
      }
    }
  }
  return true;
}

// @brief: convert the point to the original image
void DenselineLanePostprocessor::Convert2OriginalCoord(
    const std::vector<std::vector<LanePointInfo>>& lane_map_group_point_set,
    std::vector<std::vector<LanePointInfo>>* image_group_point_set) {
  float x_ratio =
      static_cast<float>(input_crop_width_) * lane_map_width_inverse_;
  float y_ratio =
      static_cast<float>(input_crop_height_) * lane_map_height_inverse_;
  for (int i = 0; i < static_cast<int>(lane_map_group_point_set.size()); i++) {
    for (int j = 0; j < static_cast<int>(lane_map_group_point_set[i].size());
         j++) {
      LanePointInfo lane_map_info = lane_map_group_point_set[i][j];
      LanePointInfo original_info;
      original_info.x =
          lane_map_info.x * x_ratio + static_cast<float>(input_offset_x_);
      original_info.y =
          lane_map_info.y * y_ratio + static_cast<float>(input_offset_y_);
      original_info.score = lane_map_info.score;
      (*image_group_point_set)[i].push_back(original_info);
    }
  }
}

// @brief: add image lane line
void DenselineLanePostprocessor::AddImageLaneline(
    const std::vector<LanePointInfo>& image_point_set,
    const base::LaneLineType type, const base::LaneLinePositionType pos_type,
    int line_index, std::vector<base::LaneLine>* lane_marks) {
  // x: longitudinal direction
  // y: horizontal direction
  // image: x = f(y)
  int image_point_set_size = static_cast<int>(image_point_set.size());
  if (image_point_set_size <= laneline_point_min_num_thresh_) {
    return;
  }
  base::LaneLine lane_mark;
  EigenVector<Eigen::Matrix<float, 2, 1>> img_pos_vec(image_point_set_size);
  Eigen::Matrix<float, max_poly_order + 1, 1> img_coeff;
  bool is_x_axis = false;
  float r_start = -1;
  float r_end = -1;
  float confidence = 0.0f;
  lane_mark.curve_image_point_set.resize(image_point_set_size);
  for (int i = 0; i < image_point_set_size; i++) {
    float x_pos = image_point_set[i].x;
    float y_pos = image_point_set[i].y;
    img_pos_vec[i] << x_pos, y_pos;
    lane_mark.curve_image_point_set[i].x = image_point_set[i].x;
    lane_mark.curve_image_point_set[i].y = image_point_set[i].y;
    if (r_start == -1) {
      r_start = y_pos;
    } else if (y_pos < r_start) {
      r_start = y_pos;
    }
    if (r_end == -1) {
      r_end = y_pos;
    } else if (y_pos > r_end) {
      r_end = y_pos;
    }
    confidence += image_point_set[i].score;
  }
  confidence /= static_cast<float>(image_point_set_size);

  bool fit_flag = PolyFit(img_pos_vec, max_poly_order, &img_coeff, is_x_axis);
  if (!fit_flag) {
    return;
  }
  //  check the validity of laneline
  float sum_dist = 0.0f;
  float avg_dist = 0.0f;
  int count = 0;
  for (int i = 0; i < image_point_set_size; i++) {
    float x_pos = img_pos_vec[i](0, 0);
    float y_pos = img_pos_vec[i](1, 0);
    float x_poly = 0.0f;
    PolyEval(y_pos, max_poly_order, img_coeff, &x_poly);
    float dist = static_cast<float>(fabs(x_poly - x_pos));
    sum_dist += dist;
    count++;
  }
  if (count > 0) {
    avg_dist = sum_dist / static_cast<float>(count);
  }
  if (avg_dist >= laneline_reject_dist_thresh_) {
    AERROR << "avg_dist>=laneline_reject_dist_thresh_";
    return;
  }
  lane_mark.curve_image_coord.a = img_coeff(3, 0);
  lane_mark.curve_image_coord.b = img_coeff(2, 0);
  lane_mark.curve_image_coord.c = img_coeff(1, 0);
  lane_mark.curve_image_coord.d = img_coeff(0, 0);
  lane_mark.curve_image_coord.x_start = r_start;
  lane_mark.curve_image_coord.x_end = r_end;

  lane_mark.confidence = confidence;
  lane_mark.track_id = line_index;
  lane_mark.type = type;
  lane_mark.pos_type = pos_type;
  lane_marks->push_back(lane_mark);
}

// @brief: fit camera lane line using polynomial
void DenselineLanePostprocessor::PolyFitCameraLaneline(CameraFrame* frame) {
  std::vector<base::LaneLine>& lane_objects = frame->lane_objects;
  int laneline_num = static_cast<int>(lane_objects.size());
  for (int line_index = 0; line_index < laneline_num; line_index++) {
    const std::vector<base::Point3DF>& camera_point_set =
        lane_objects[line_index].curve_camera_point_set;
    // z: longitudinal direction
    // x: latitudinal direction
    float x_start = camera_point_set[0].z;
    float x_end = 0.0f;
    Eigen::Matrix<float, max_poly_order + 1, 1> camera_coeff;
    EigenVector<Eigen::Matrix<float, 2, 1>> camera_pos_vec;
    for (int i = 0; i < static_cast<int>(camera_point_set.size()); i++) {
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
    lane_objects[line_index].curve_car_coord.a = -camera_coeff(3, 0);
    lane_objects[line_index].curve_car_coord.b = -camera_coeff(2, 0);
    lane_objects[line_index].curve_car_coord.c = -camera_coeff(1, 0);
    lane_objects[line_index].curve_car_coord.d = -camera_coeff(0, 0);
    lane_objects[line_index].curve_car_coord.x_start = x_start;
    lane_objects[line_index].curve_car_coord.x_end = x_end;
    lane_objects[line_index].use_type = base::LaneLineUseType::REAL;
  }
}

std::vector<std::vector<LanePointInfo>>
DenselineLanePostprocessor::GetLanelinePointSet() {
  return image_group_point_set_;
}

std::vector<LanePointInfo>
DenselineLanePostprocessor::GetAllInferLinePointSet() {
  float x_ratio =
      static_cast<float>(input_crop_width_) * lane_map_width_inverse_;
  float y_ratio =
      static_cast<float>(input_crop_height_) * lane_map_height_inverse_;
  image_laneline_point_set_.clear();
  int lane_map_dim = lane_map_width_ * lane_map_height_;
  float lane_map_width_float = static_cast<float>(lane_map_width_);
  for (int i = 0; i < lane_map_height_; i++) {
    for (int j = 0; j < lane_map_width_; j++) {
      int pixel_pos = i * lane_map_width_ + j;
      float score = lane_output_[lane_map_dim * 2 + pixel_pos];
      if (score < laneline_point_score_thresh_) {
        continue;
      }
      float dist_left = lane_output_[pixel_pos];
      float dist_right = lane_output_[lane_map_dim + pixel_pos];
      float map_x_left =
          static_cast<float>(j) - dist_left * lane_map_width_float;
      float map_x_right =
          static_cast<float>(j) + dist_right * lane_map_width_float;
      float org_img_y =
          static_cast<float>(i) * y_ratio + static_cast<float>(input_offset_y_);
      if (map_x_left > 0) {
        LanePointInfo left_point;
        float org_img_x_left =
            map_x_left * x_ratio + static_cast<float>(input_offset_x_);
        left_point.x = org_img_x_left;
        left_point.y = org_img_y;
        left_point.score = score;
        image_laneline_point_set_.push_back(left_point);
      }
      if (map_x_right < lane_map_width_float) {
        LanePointInfo right_point;
        float org_img_x_right =
            map_x_right * x_ratio + static_cast<float>(input_offset_x_);
        right_point.x = org_img_x_right;
        right_point.y = org_img_y;
        right_point.score = score;
        image_laneline_point_set_.push_back(right_point);
      }
    }
  }
  return image_laneline_point_set_;
}

void DenselineLanePostprocessor::GetLaneCCs(
    std::vector<unsigned char>* lane_map, int* lane_map_width,
    int* lane_map_height, std::vector<ConnectedComponent>* connected_components,
    std::vector<ConnectedComponent>* select_connected_components) {
  *lane_map = lane_map_;
  *lane_map_width = lane_map_width_;
  *lane_map_height = lane_map_height_;
  *connected_components = lane_ccs_;
  *select_connected_components = select_lane_ccs_;
}

std::string DenselineLanePostprocessor::Name() const {
  return "DenselineLanePostprocessor";
}

REGISTER_LANE_POSTPROCESSOR(DenselineLanePostprocessor);
}  // namespace camera
}  // namespace perception
}  // namespace apollo
