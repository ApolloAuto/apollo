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

// @brief: CC lane post-processor source file

#include <algorithm>
#include <cmath>
#include <limits>
#include <numeric>
#include <unordered_map>

#include "modules/perception/obstacle/camera/lane_post_process/cc_lane_post_processor/cc_lane_post_processor.h"

namespace apollo {
namespace perception {

using std::pair;
using std::shared_ptr;
using std::string;
using std::to_string;
using std::unordered_map;
using std::vector;

bool CompOriginLateralDistObjectID(const pair<ScalarType, int> &a,
                                   const pair<ScalarType, int> &b) {
  return a.first > b.first;
}

bool CCLanePostProcessor::Init() {
  // 1. get model config
  ConfigManager *config_manager = ConfigManager::instance();
  //     Singleton<config_manager::ConfigManager>::get();

  const ModelConfig *model_config = NULL;
  if (!config_manager->GetModelConfig(this->name(), &model_config)) {
    AERROR << "not found model: " << this->name();
    return false;
  }

  // 2. get parameters
  string space_type;
  if (!model_config->GetValue("space_type", &space_type)) {
    AERROR << "space type not found.";
    return false;
  }
  if (space_type == "vehicle") {
    options_.space_type = SpaceType::VEHICLE;
  } else if (space_type == "image") {
    AINFO << "using image space to generate lane instances ...";
    options_.space_type = SpaceType::IMAGE;
  } else {
    AFATAL << "invalid space type" << space_type;
    return false;
  }
  options_.frame.space_type = options_.space_type;

  if (!model_config->GetValue("image_width", &image_width_)) {
    AERROR << "image width not found.";
    return false;
  }
  if (!model_config->GetValue("image_height", &image_height_)) {
    AERROR << "image height not found.";
    return false;
  }

  std::vector<float> roi;
  if (!model_config->GetValue("roi", &roi)) {
    AERROR << "roi not found.";
    return false;
  }
  if (static_cast<int>(roi.size()) != 4) {
    AERROR << "roi format error.";
    return false;
  } else {
    roi_.x = static_cast<int>(roi[0]);
    roi_.y = static_cast<int>(roi[1]);
    roi_.width = static_cast<int>(roi[2]);
    roi_.height = static_cast<int>(roi[3]);
    options_.frame.image_roi = roi_;
    AINFO << "project ROI = [" << roi_.x << ", " << roi_.y << ", "
          << roi_.x + roi_.width - 1 << ", " << roi_.y + roi_.height - 1 << "]";
  }

  if (!model_config->GetValue("lane_map_confidence_thresh",
                              &options_.lane_map_conf_thresh)) {
    AERROR << "the confidence threshold of label map not found.";
    return false;
  }

  if (!model_config->GetValue("cc_split_siz", &options_.cc_split_siz)) {
    AERROR << "maximum bounding-box size for splitting CC not found.";
    return false;
  }
  if (!model_config->GetValue("cc_split_len", &options_.cc_split_len)) {
    AERROR << "unit length for splitting CC not found.";
    return false;
  }

  // parameters on generating markers
  if (!model_config->GetValue("min_cc_pixel_num",
                              &options_.frame.min_cc_pixel_num)) {
    AERROR << "minimum CC pixel number not found.";
    return false;
  }

  if (!model_config->GetValue("min_cc_size", &options_.frame.min_cc_size)) {
    AERROR << "minimum CC size not found.";
    return false;
  }

  if (!model_config->GetValue(options_.frame.space_type == SpaceType::IMAGE
                                  ? "min_y_search_offset_image"
                                  : "min_y_search_offset",
                              &options_.frame.min_y_search_offset)) {
    AERROR << "minimum verticle offset used for marker association not found.";
    return false;
  }

  // parameters on marker association
  string assoc_method;
  if (!model_config->GetValue("assoc_method", &assoc_method)) {
    AERROR << "marker association method not found.";
    return false;
  }
  if (assoc_method == "greedy_group_connect") {
    options_.frame.assoc_param.method = AssociationMethod::GREEDY_GROUP_CONNECT;
  } else {
    AERROR << "invalid marker association method." << assoc_method;
    return false;
  }

  if (!model_config->GetValue(options_.frame.space_type == SpaceType::IMAGE
                                  ? "assoc_min_distance_image"
                                  : "assoc_min_distance",
                              &options_.frame.assoc_param.min_distance)) {
    AERROR << "minimum distance threshold for marker association not found.";
    return false;
  }
  AINFO << "assoc_min_distance = " << options_.frame.assoc_param.min_distance;

  if (!model_config->GetValue(options_.frame.space_type == SpaceType::IMAGE
                                  ? "assoc_max_distance_image"
                                  : "assoc_max_distance",
                              &options_.frame.assoc_param.max_distance)) {
    AERROR << "maximum distance threshold for marker association not found.";
    return false;
  }
  AINFO << "assoc_max_distance = " << options_.frame.assoc_param.max_distance;

  if (!model_config->GetValue("assoc_distance_weight",
                              &options_.frame.assoc_param.distance_weight)) {
    AERROR << "distance weight for marker association not found.";
    return false;
  }
  AINFO << "assoc_distance_weight = "
        << options_.frame.assoc_param.distance_weight;

  if (!model_config->GetValue(
          options_.frame.space_type == SpaceType::IMAGE
              ? "assoc_max_deviation_angle_image"
              : "assoc_max_deviation_angle",
          &options_.frame.assoc_param.max_deviation_angle)) {
    AERROR << "max deviation angle threshold "
           << "for marker association not found.";
    return false;
  }
  AINFO << "assoc_max_deviation_angle = "
        << options_.frame.assoc_param.max_deviation_angle;
  options_.frame.assoc_param.max_deviation_angle *= (M_PI / 180.0);

  if (!model_config->GetValue(
          "assoc_deviation_angle_weight",
          &options_.frame.assoc_param.deviation_angle_weight)) {
    AERROR << "deviation angle weight "
           << "for marker association not found.";
    return false;
  }
  AINFO << "assoc_deviation_angle_weight = "
        << options_.frame.assoc_param.deviation_angle_weight;

  if (!model_config->GetValue(options_.frame.space_type == SpaceType::IMAGE
                                  ? "assoc_max_relative_orie_image"
                                  : "assoc_max_relative_orie",
                              &options_.frame.assoc_param.max_relative_orie)) {
    AERROR << "max relative orientation threshold "
           << "for marker association not found.";
    return false;
  }
  AINFO << "assoc_max_relative_orie = "
        << options_.frame.assoc_param.max_relative_orie;
  options_.frame.assoc_param.max_relative_orie *= (M_PI / 180.0);

  if (!model_config->GetValue(
          "assoc_relative_orie_weight",
          &options_.frame.assoc_param.relative_orie_weight)) {
    AERROR << "relative orientation weight "
           << "for marker association not found.";
    return false;
  }
  AINFO << "assoc_relative_orie_weight = "
        << options_.frame.assoc_param.relative_orie_weight;

  if (!model_config->GetValue(
          options_.frame.space_type == SpaceType::IMAGE
              ? "assoc_max_departure_distance_image"
              : "assoc_max_departure_distance",
          &options_.frame.assoc_param.max_departure_distance)) {
    AERROR << "max departure distance threshold "
           << "for marker association not found.";
    return false;
  }
  AINFO << "assoc_max_departure_distance = "
        << options_.frame.assoc_param.max_departure_distance;

  if (!model_config->GetValue(
          "assoc_departure_distance_weight",
          &options_.frame.assoc_param.departure_distance_weight)) {
    AERROR << "departure distance weight "
           << "for marker association not found.";
    return false;
  }
  AINFO << "assoc_departure_distance_weight = "
        << options_.frame.assoc_param.departure_distance_weight;

  if (!model_config->GetValue(
          options_.frame.space_type == SpaceType::IMAGE
              ? "assoc_min_orientation_estimation_size_image"
              : "assoc_min_orientation_estimation_size",
          &options_.frame.assoc_param.min_orientation_estimation_size)) {
    AERROR << "minimum size threshold used for orientation estimation"
           << " in marker association not found.";
    return false;
  }
  AINFO << "assoc_min_orientation_estimation_size = "
        << options_.frame.assoc_param.min_orientation_estimation_size;

  if (options_.frame.assoc_param.method ==
      AssociationMethod::GREEDY_GROUP_CONNECT) {
    if (!model_config->GetValue(
            "max_group_prediction_marker_num",
            &options_.frame.group_param.max_group_prediction_marker_num)) {
      AERROR << "maximum number of markers used for orientation estimation"
             << " in greed group connect association not found.";
      return false;
    }
  } else {
    AFATAL << "invalid marker association method.";
    return false;
  }

  if (!model_config->GetValue(
          "orientation_estimation_skip_marker_num",
          &options_.frame.orientation_estimation_skip_marker_num)) {
    AERROR << "skip marker number used for orientation estimation in "
           << "marker association";
    return false;
  }

  // parameters on finding lane objects
  if (!model_config->GetValue("lane_interval_distance",
                              &options_.frame.lane_interval_distance)) {
    AERROR << "The predefined lane interval distance is not found.";
    return false;
  }

  if (!model_config->GetValue(options_.frame.space_type == SpaceType::IMAGE
                                  ? "min_instance_size_prefiltered_image"
                                  : "min_instance_size_prefiltered",
                              &options_.frame.min_instance_size_prefiltered)) {
    AERROR << "The minimum size of lane instances "
           << "to be prefiltered is not found.";
    return false;
  }

  if (!model_config->GetValue(options_.frame.space_type == SpaceType::IMAGE
                                  ? "max_size_to_fit_straight_line_image"
                                  : "max_size_to_fit_straight_line",
                              &options_.frame.max_size_to_fit_straight_line)) {
    AERROR << "The maximum size used for fitting straight lines "
           << "on lane instances is not found.";
    return false;
  }

  // 3. initialize projector
  if (!model_config->GetValue("max_distance_to_see_for_transformer",
                              &max_distance_to_see_)) {
    AERROR << "maximum perception distance for transformer is not found, "
              "use default value";
    return false;
  }
  AINFO << "initial max_distance_to_see: " << max_distance_to_see_
        << " (meters)";

  if (options_.space_type == SpaceType::VEHICLE) {
    projector_.reset(new Projector<ScalarType>());
    projector_->Init(roi_, max_distance_to_see_, vis_);
    is_x_longitude_ = true;
  } else if (options_.space_type == SpaceType::IMAGE) {
    is_x_longitude_ = false;
  } else {
    AFATAL << "invalid space type" << space_type;
    return false;
  }

  time_stamp_ = 0.0;
  frame_id_ = 0;
  cc_generator_.reset(
      new ConnectedComponentGenerator(image_width_, image_height_, roi_));
  cur_frame_.reset(new LaneFrame);

  is_init_ = true;
  return true;
}

bool CCLanePostProcessor::AddInstanceIntoLaneObject(
    const LaneInstance &instance, LaneObject *lane_object) {
  if (lane_object == nullptr) {
    AERROR << "lane object is a null pointer";
    return false;
  }

  if (lane_object->pos.empty()) {
    lane_object->pos.reserve(20);
    lane_object->orie.reserve(20);
    lane_object->image_pos.reserve(20);
    lane_object->confidence.reserve(20);
  }

  auto graph = cur_frame_->graph(instance.graph_id);

  int i_prev = -1;
  for (size_t j = 0; j < graph->size(); ++j) {
    int i = graph->at(j).first;
    if (cur_frame_->marker(i)->shape_type != MarkerShapeType::POINT) {
      if (i_prev < 0 ||
          cur_frame_->marker(i)->cc_id != cur_frame_->marker(i_prev)->cc_id) {
        lane_object->pos.push_back(cur_frame_->marker(i)->start_pos);
        lane_object->orie.push_back(cur_frame_->marker(i)->orie);
        lane_object->image_pos.push_back(
            cur_frame_->marker(i)->image_start_pos);
        lane_object->confidence.push_back(cur_frame_->marker(i)->confidence);
        lane_object->longitude_start =
            std::min(lane_object->pos.back().x(), lane_object->longitude_start);
        lane_object->longitude_end =
            std::max(lane_object->pos.back().x(), lane_object->longitude_end);
        lane_object->point_num++;
      }
    }

    lane_object->pos.push_back(cur_frame_->marker(i)->pos);
    lane_object->orie.push_back(cur_frame_->marker(i)->orie);
    lane_object->image_pos.push_back(cur_frame_->marker(i)->image_pos);
    lane_object->confidence.push_back(cur_frame_->marker(i)->confidence);
    lane_object->longitude_start =
        std::min(lane_object->pos.back().x(), lane_object->longitude_start);
    lane_object->longitude_end =
        std::max(lane_object->pos.back().x(), lane_object->longitude_end);
    lane_object->point_num++;
    i_prev = i;
  }

  if (lane_object->point_num != lane_object->pos.size() ||
      lane_object->point_num != lane_object->orie.size() ||
      lane_object->point_num != lane_object->image_pos.size() ||
      lane_object->point_num != lane_object->confidence.size()) {
    AERROR << "the number of points in lane object does not match.";
    return false;
  }

  if (lane_object->point_num < 2) {
    AERROR << "the number of points in lane object should be no less than 2";
    return false;
  }

  // fit polynomial model and compute lateral distance for lane object
  if (lane_object->point_num < 3 ||
      lane_object->longitude_end - lane_object->longitude_start <
          options_.frame.max_size_to_fit_straight_line) {
    // fit a 1st-order polynomial curve (straight line)
    lane_object->order = 1;
  } else {
    // fit a 2nd-order polynomial curve;
    lane_object->order = 2;
  }

  if (!PolyFit(lane_object->pos, lane_object->order, &(lane_object->model))) {
    AERROR << "failed to fit " << lane_object->order
           << " order polynomial curve.";
  }
  lane_object->lateral_distance = lane_object->model(0);

  return true;
}

bool CCLanePostProcessor::AddInstanceIntoLaneObjectImage(
    const LaneInstance &instance, LaneObject *lane_object) {
  if (lane_object == nullptr) {
    AERROR << "lane object is a null pointer";
    return false;
  }

  if (lane_object->pos.empty()) {
    lane_object->pos.reserve(20);
    lane_object->orie.reserve(20);
    lane_object->image_pos.reserve(20);
    lane_object->confidence.reserve(20);
  }

  auto graph = cur_frame_->graph(instance.graph_id);

  ADEBUG << "show points for lane object: ";

  ScalarType y_offset = static_cast<ScalarType>(image_height_ - 1);

  int i_prev = -1;
  for (size_t j = 0; j < graph->size(); ++j) {
    int i = graph->at(j).first;
    if (cur_frame_->marker(i)->shape_type != MarkerShapeType::POINT) {
      if (i_prev < 0 ||
          cur_frame_->marker(i)->cc_id != cur_frame_->marker(i_prev)->cc_id) {
        lane_object->pos.push_back(cur_frame_->marker(i)->start_pos);
        lane_object->pos.back()(1) = y_offset - lane_object->pos.back()(1);
        lane_object->orie.push_back(cur_frame_->marker(i)->orie);
        lane_object->orie.back()(1) = -lane_object->orie.back()(1);
        lane_object->image_pos.push_back(
            cur_frame_->marker(i)->image_start_pos);
        lane_object->confidence.push_back(cur_frame_->marker(i)->confidence);
        lane_object->longitude_start =
            std::min(lane_object->pos.back().y(), lane_object->longitude_start);
        lane_object->longitude_end =
            std::max(lane_object->pos.back().y(), lane_object->longitude_end);

        ADEBUG << " point " << lane_object->point_num << " = "
               << "(" << lane_object->pos.back()(0) << ", "
               << lane_object->pos.back()(1) << "), "
               << "(" << lane_object->image_pos.back()(0) << ", "
               << lane_object->image_pos.back()(1) << ")";

        lane_object->point_num++;
      }
    }

    lane_object->pos.push_back(cur_frame_->marker(i)->pos);
    lane_object->pos.back()(1) = y_offset - lane_object->pos.back()(1);
    lane_object->orie.push_back(cur_frame_->marker(i)->orie);
    lane_object->orie.back()(1) = -lane_object->orie.back()(1);
    lane_object->image_pos.push_back(cur_frame_->marker(i)->image_pos);
    lane_object->confidence.push_back(cur_frame_->marker(i)->confidence);
    lane_object->longitude_start =
        std::min(lane_object->pos.back().y(), lane_object->longitude_start);
    lane_object->longitude_end =
        std::max(lane_object->pos.back().y(), lane_object->longitude_end);

    ADEBUG << " point " << lane_object->point_num << " = "
           << "(" << lane_object->pos.back()(0) << ", "
           << lane_object->pos.back()(1) << "), "
           << "(" << lane_object->image_pos.back()(0) << ", "
           << lane_object->image_pos.back()(1) << ")";

    lane_object->point_num++;
    i_prev = i;
  }
  ADEBUG << "longitude_start = " << lane_object->longitude_start;
  ADEBUG << "longitude_end = " << lane_object->longitude_end;

  if (lane_object->point_num != lane_object->pos.size() ||
      lane_object->point_num != lane_object->orie.size() ||
      lane_object->point_num != lane_object->image_pos.size() ||
      lane_object->point_num != lane_object->confidence.size()) {
    AERROR << "the number of points in lane object does not match.";
    return false;
  }

  if (lane_object->point_num < 2) {
    AERROR << "the number of points in lane object should be no less than 2";
    return false;
  }

  // fit polynomial model and compute lateral distance for lane object
  ADEBUG << "max_size_to_fit_straight_line = "
         << options_.frame.max_size_to_fit_straight_line;
  if (lane_object->point_num < 3 ||
      lane_object->longitude_end - lane_object->longitude_start <
          options_.frame.max_size_to_fit_straight_line) {
    // fit a 1st-order polynomial curve (straight line)
    lane_object->order = 1;
  } else {
    // fit a 2nd-order polynomial curve;
    lane_object->order = 2;
  }

  if (!PolyFit(lane_object->pos, lane_object->order, &(lane_object->model),
               false)) {
    AERROR << "failed to fit " << lane_object->order
           << " order polynomial curve";
  }

  lane_object->lateral_distance = lane_object->model(0);

  return true;
}

bool CCLanePostProcessor::GenerateLaneInstances(const cv::Mat &lane_map) {
  if (!is_init_) {
    AERROR << "the lane post-processor is not initialized.";
    return false;
  }

  if (lane_map.empty()) {
    AERROR << "input lane map is empty.";
    return false;
  }

  float time_cur_frame = 0.0f;
  Timer timer;

  // 1. get binary lane label mask
  timer.tic();
  cv::Mat lane_mask;
  if (lane_map.type() == CV_32FC1) {
    // confidence heatmap
    ADEBUG << "confidence threshold = " << options_.lane_map_conf_thresh;
    ADEBUG << "lane map size = "
           << "(" << lane_map.cols << ", " << lane_map.rows << ")";
    lane_mask.create(lane_map.rows, lane_map.cols, CV_8UC1);
    lane_mask.setTo(cv::Scalar(0));
    for (int h = 0; h < lane_mask.rows; ++h) {
      for (int w = 0; w < lane_mask.cols; ++w) {
        if (lane_map.at<float>(h, w) >= options_.lane_map_conf_thresh) {
          lane_mask.at<unsigned char>(h, w) = 1;
        }
      }
    }
  } else if (lane_map.type() == CV_8UC1) {
    // lane label map
    lane_mask = lane_map;
  } else {
    AERROR << "invalid input lane map type: " << lane_map.type();
    return false;
  }
  float time_lane_mask = static_cast<float>(timer.toc());
  time_cur_frame += time_lane_mask;

  // 2. find connected components from lane label mask
  timer.tic();
  vector<ConnectedComponentPtr> cc_list;
  cc_generator_->FindConnectedComponents(lane_mask, &cc_list);

  float time_find_cc = static_cast<float>(timer.toc());
  AINFO << "time to find connected components: " << time_find_cc << " ms";
  AINFO << "number of connected components = " << cc_list.size();
  time_cur_frame += time_find_cc;

  // 3. split CC and find inner edges
  timer.tic();
  int tot_inner_edge_count = 0;
  for (int i = 0; i < static_cast<int>(cc_list.size()); ++i) {
    auto it_cc = cc_list[i];
    it_cc->FindBboxPixels();
    it_cc->FindVertices();
    it_cc->FindEdges();
    if (it_cc->DetermineSplit(options_.cc_split_siz) !=
        ConnectedComponent::BoundingBoxSplitType::NONE) {
      it_cc->FindContourForSplit();
      it_cc->SplitContour(options_.cc_split_len);
    }
    tot_inner_edge_count += static_cast<int>(it_cc->GetInnerEdges()->size());
  }
  float time_find_inner_edges = static_cast<float>(timer.toc());
  time_cur_frame += time_find_inner_edges;
  AINFO << "time to find inner edges: " << time_find_inner_edges << " ms";
  AINFO << "number of inner edge = " << tot_inner_edge_count;

  /// 4. do lane marker association and determine lane instance labels
  timer.tic();
  cur_frame_.reset(new LaneFrame);

  if (options_.frame.space_type == SpaceType::IMAGE) {
    cur_frame_->Init(cc_list, options_.frame);
  } else if (options_.frame.space_type == SpaceType::VEHICLE) {
    cur_frame_->Init(cc_list, projector_, options_.frame);
  } else {
    AERROR << "unknown space type: " << options_.frame.space_type;
    return false;
  }

  cur_frame_->Process(cur_lane_instances_);

  float time_lane_frame = static_cast<float>(timer.toc());
  time_cur_frame += time_lane_frame;
  AINFO << "time for frame processing: " << time_lane_frame << " ms";
  AINFO << "number of lane instances = " << cur_lane_instances_->size();

  AINFO << "lane post-processing runtime for current frame: " << time_cur_frame
        << " ms";

  return true;
}

bool CCLanePostProcessor::Process(const cv::Mat &lane_map,
                                  const CameraLanePostProcessOptions &options,
                                  LaneObjectsPtr lane_objects) {
  if (!is_init_) {
    AERROR << "lane post-processor is not initialized";
    return false;
  }

  if (lane_map.empty()) {
    AERROR << "input lane map is empty";
    return false;
  }
  if (lane_map.cols != image_width_) {
    AERROR << "input lane map width does not match: "
           << "(" << lane_map.cols << " vs. " << image_width_ << ")";
    return false;
  }
  if (lane_map.rows != image_height_) {
    AERROR << "input lane map height does not match: "
           << "(" << lane_map.rows << " vs. " << image_height_ << ")";
    return false;
  }

  time_stamp_ = options.timestamp;

  cur_lane_instances_.reset(new vector<LaneInstance>);
  if (!GenerateLaneInstances(lane_map)) {
    AERROR << "failed to compute lane instances.";
    return false;
  }
  std::sort(cur_lane_instances_->begin(), cur_lane_instances_->end(),
            LaneInstance::CompareSiz);

  /// generate lane objects
  Timer timer;
  if (options_.space_type == SpaceType::IMAGE) {
    /// for image space coordinate
    AINFO << "generate lane objects in image space ...";
    ScalarType x_center = static_cast<ScalarType>(roi_.x + roi_.width / 2);

    lane_objects.reset(new LaneObjects());
    lane_objects->reserve(2);
    bool is_left_lane_found = false;
    bool is_right_lane_found = false;
    for (auto it = cur_lane_instances_->begin();
         it != cur_lane_instances_->end(); ++it) {
      ADEBUG << "for lane instance " << it - cur_lane_instances_->begin();

      if (is_left_lane_found && is_right_lane_found) {
        break;
      }

      LaneObject cur_object;
      AddInstanceIntoLaneObjectImage(*it, &cur_object);

      if (cur_object.lateral_distance <= x_center) {
        // for left lane
        if (is_left_lane_found) {
          continue;
        }
        lane_objects->push_back(cur_object);
        lane_objects->back().spatial = SpatialLabelType::L_0;
        is_left_lane_found = true;
      } else {
        // for right lane
        if (is_right_lane_found) {
          continue;
        }
        lane_objects->push_back(cur_object);
        lane_objects->back().spatial = SpatialLabelType::R_0;
        is_right_lane_found = true;
      }

      AINFO << " lane object " << lane_objects->back().GetSpatialLabel()
            << " has " << lane_objects->back().pos.size() << " points: "
            << "lateral distance = " << lane_objects->back().lateral_distance;
    }

  } else {
    /// for vehicle space coordinate
    // select lane instances with non-overlap assumption
    AINFO << "generate lane objects ...";
    lane_objects.reset(new LaneObjects());
    lane_objects->reserve(2 * MAX_LANE_SPATIAL_LABELS);
    vector<pair<ScalarType, int>> origin_lateral_dist_object_id;
    origin_lateral_dist_object_id.reserve(2 * MAX_LANE_SPATIAL_LABELS);
    int count_lane_objects = 0;
    for (auto it = cur_lane_instances_->begin();
         it != cur_lane_instances_->end(); ++it) {
      ADEBUG << "for lane instance " << it - cur_lane_instances_->begin();

      // ignore current instance if it is too small
      if (it->siz < options_.frame.min_instance_size_prefiltered) {
        ADEBUG << "current instance is too small: " << it->siz;
        continue;
      }

      LaneObject cur_object;
      AddInstanceIntoLaneObject(*it, &cur_object);

      if (lane_objects->empty()) {
        // create a new lane object
        lane_objects->push_back(cur_object);
        origin_lateral_dist_object_id.push_back(
            std::make_pair(cur_object.lateral_distance, count_lane_objects++));
        ADEBUG << "generate a new lane object from instance";
        continue;
      }

      // ignore current instance if it crosses over any existing groups
      bool is_cross_over = false;
      vector<pair<ScalarType, ScalarType>> lateral_distances(
          count_lane_objects);
      size_t cross_over_lane_object_id = 0;
      for (size_t k = 0; k < lane_objects->size(); ++k) {
        // min distance to instance group
        lateral_distances[k].first = std::numeric_limits<ScalarType>::max();
        // max distance to instance group
        lateral_distances[k].second = -std::numeric_limits<ScalarType>::max();

        // determine whether cross over or not
        for (size_t i = 0; i < cur_object.pos.size(); ++i) {
          ScalarType deta_y =
              cur_object.pos[i].y() - PolyEval(cur_object.pos[i].x(),
                                               lane_objects->at(k).order,
                                               lane_objects->at(k).model);
          lateral_distances[k].first =
              std::min(lateral_distances[k].first, deta_y);
          lateral_distances[k].second =
              std::max(lateral_distances[k].second, deta_y);
          if (lateral_distances[k].first * lateral_distances[k].second < 0) {
            is_cross_over = true;
          }
          if (is_cross_over) {
            break;
          }
        }

        if (is_cross_over) {
          cross_over_lane_object_id = k;
          break;
        }
      }
      if (is_cross_over) {
        ADEBUG << "instance crosses over lane object "
               << cross_over_lane_object_id;
        continue;
      }

      // search the very left lane w.r.t. current instance
      int left_lane_id = -1;
      ScalarType left_lane_dist = -std::numeric_limits<ScalarType>::max();
      for (int k = 0; k < count_lane_objects; ++k) {
        if (lateral_distances[k].second <= 0) {
          if (lateral_distances[k].second > left_lane_dist) {
            left_lane_dist = lateral_distances[k].second;
            left_lane_id = k;
          }
        }
      }
      if ((left_lane_id >= 0) &&
          (origin_lateral_dist_object_id.at(left_lane_id).first -
               cur_object.lateral_distance <
           MIN_BETWEEN_LANE_DISTANCE)) {
        ADEBUG << "too close to left lane object (" << left_lane_id << "): "
               << origin_lateral_dist_object_id.at(left_lane_id).first -
                      cur_object.lateral_distance
               << "(" << MIN_BETWEEN_LANE_DISTANCE << ")";
        continue;
      }

      // search the very right lane w.r.t. current instance
      int right_lane_id = -1;
      ScalarType right_lane_dist = std::numeric_limits<ScalarType>::max();
      for (int k = 0; k < count_lane_objects; ++k) {
        if (lateral_distances[k].first > 0) {
          if (lateral_distances[k].first < right_lane_dist) {
            right_lane_dist = lateral_distances[k].first;
            right_lane_id = k;
          }
        }
      }
      if ((right_lane_id >= 0) &&
          (cur_object.lateral_distance -
               origin_lateral_dist_object_id.at(right_lane_id).first <
           MIN_BETWEEN_LANE_DISTANCE)) {
        ADEBUG << "too close to right lane object (" << right_lane_id << "): "
               << origin_lateral_dist_object_id.at(right_lane_id).first -
                      cur_object.lateral_distance
               << "(" << MIN_BETWEEN_LANE_DISTANCE << ")";
        continue;
      }

      // accept the new lane object
      lane_objects->push_back(cur_object);
      origin_lateral_dist_object_id.push_back(
          std::make_pair(cur_object.lateral_distance, count_lane_objects++));
      ADEBUG << "generate a new lane object from instance.";
    }

    // determine spatial label of lane object
    std::sort(origin_lateral_dist_object_id.begin(),
              origin_lateral_dist_object_id.end(),
              CompOriginLateralDistObjectID);
    int i_l0 = -1;
    for (int k = 0; k < count_lane_objects; ++k) {
      if (origin_lateral_dist_object_id[k].first >= 0) {
        i_l0 = k;
      } else {
        break;
      }
    }

    vector<int> valid_lane_objects;
    valid_lane_objects.reserve(lane_objects->size());

    // for left-side lanes
    for (int spatial_index = 0; spatial_index <= i_l0; ++spatial_index) {
      if (spatial_index >= MAX_LANE_SPATIAL_LABELS) {
        break;
      }
      int i_l = i_l0 - spatial_index;
      int object_id = origin_lateral_dist_object_id.at(i_l).second;
      lane_objects->at(object_id).spatial =
          static_cast<SpatialLabelType>(spatial_index);
      valid_lane_objects.push_back(object_id);

      AINFO << " lane object " << lane_objects->at(object_id).GetSpatialLabel()
            << " has " << lane_objects->at(object_id).pos.size() << " points: "
            << "lateral distance="
            << lane_objects->at(object_id).lateral_distance;
    }

    // for right-side lanes
    int i_r = i_l0 + 1;
    for (int spatial_index = 0; spatial_index < MAX_LANE_SPATIAL_LABELS;
         ++spatial_index, ++i_r) {
      if (i_r >= count_lane_objects) {
        break;
      }
      int object_id = origin_lateral_dist_object_id.at(i_r).second;
      lane_objects->at(object_id).spatial = static_cast<SpatialLabelType>(
          MAX_LANE_SPATIAL_LABELS + spatial_index);
      valid_lane_objects.push_back(object_id);

      AINFO << " lane object " << lane_objects->at(object_id).GetSpatialLabel()
            << " has " << lane_objects->at(object_id).pos.size() << " points: "
            << "lateral distance="
            << lane_objects->at(object_id).lateral_distance;
    }
    if (lane_objects->size() != static_cast<size_t>(count_lane_objects)) {
      AERROR << "the number of lane objects does not match.";
      return false;
    }

    // keep only the lane objects with valid spatial labels
    std::sort(valid_lane_objects.begin(), valid_lane_objects.end());
    for (size_t i = 0; i < valid_lane_objects.size(); ++i) {
      lane_objects->at(i) = lane_objects->at(valid_lane_objects[i]);
    }
    lane_objects->resize(valid_lane_objects.size());
  }

  AINFO << "number of lane objects = " << lane_objects->size();
  if (options_.space_type != SpaceType::IMAGE) {
    if (!CompensateLaneObjects(lane_objects)) {
      AERROR << "fail to compensate lane objects.";
      return false;
    }
  }
  EnrichLaneInfo(lane_objects);
  float time_generate_lane_objects = static_cast<float>(timer.toc());
  AINFO << "runtime of generating lane objects: " << time_generate_lane_objects;

  return true;
}

bool CCLanePostProcessor::CompensateLaneObjects(LaneObjectsPtr lane_objects) {
  if (lane_objects == NULL) {
    AERROR << "lane_objects is a null pointer.";
    return false;
  }

  bool has_ego_lane_left = false;
  int ego_lane_left_idx = -1;
  bool has_ego_lane_right = false;
  int ego_lane_right_idx = -1;
  for (size_t i = 0; i < lane_objects->size(); ++i) {
    if (lane_objects->at(i).spatial == SpatialLabelType::L_0) {
      has_ego_lane_left = true;
      ego_lane_left_idx = static_cast<int>(i);
    } else if (lane_objects->at(i).spatial == SpatialLabelType::R_0) {
      has_ego_lane_right = true;
      ego_lane_right_idx = static_cast<int>(i);
    }
  }

  if ((has_ego_lane_left && has_ego_lane_right) ||
      (!has_ego_lane_left && !has_ego_lane_right)) {
    return true;
  }

  if (!has_ego_lane_left) {
    AINFO << "add virtual lane L_0 ...";
    if (ego_lane_right_idx == -1) {
      AERROR << "failed to compensate left ego lane due to no right ego lane.";
      return false;
    }

    LaneObject virtual_ego_lane_left;
    lane_objects->at(ego_lane_right_idx).CopyTo(&virtual_ego_lane_left);

    virtual_ego_lane_left.spatial = SpatialLabelType::L_0;
    virtual_ego_lane_left.is_compensated = true;

    for (size_t i = 0; i < virtual_ego_lane_left.pos.size(); ++i) {
      virtual_ego_lane_left.pos[i](1) += options_.frame.lane_interval_distance;
    }
    virtual_ego_lane_left.lateral_distance +=
        options_.frame.lane_interval_distance;
    virtual_ego_lane_left.model(0) += options_.frame.lane_interval_distance;

    lane_objects->push_back(virtual_ego_lane_left);
  }

  if (!has_ego_lane_right) {
    AINFO << "add virtual lane R_0 ...";
    if (ego_lane_left_idx == -1) {
      AERROR << "failed to compensate right ego lane due to no left ego lane.";
      return false;
    }

    LaneObject virtual_ego_lane_right;
    lane_objects->at(ego_lane_left_idx).CopyTo(&virtual_ego_lane_right);

    virtual_ego_lane_right.spatial = SpatialLabelType::R_0;
    virtual_ego_lane_right.is_compensated = true;

    for (size_t i = 0; i < virtual_ego_lane_right.pos.size(); ++i) {
      virtual_ego_lane_right.pos[i](1) -= options_.frame.lane_interval_distance;
    }
    virtual_ego_lane_right.lateral_distance -=
        options_.frame.lane_interval_distance;
    virtual_ego_lane_right.model(0) -= options_.frame.lane_interval_distance;

    lane_objects->push_back(virtual_ego_lane_right);
  }

  return true;
}

bool CCLanePostProcessor::EnrichLaneInfo(LaneObjectsPtr lane_objects) {
  if (lane_objects == NULL) {
    AERROR << "lane_objects is a null pointer.";
    return false;
  }

  for (size_t i = 0; i < lane_objects->size(); ++i) {
    LaneObject &object = (*lane_objects)[i];
    assert(object.pos.size() > 0);
    assert(object.image_pos.size() == object.pos.size());

    // for polynomial curve on vehicle space
    object.pos_curve.a = object.model(3);
    object.pos_curve.b = object.model(2);
    object.pos_curve.c = object.model(1);
    object.pos_curve.d = object.model(0);

    // let x_start be always 0 according to L3's PNC requirement
    object.pos_curve.x_start =
        std::min(object.longitude_start, static_cast<ScalarType>(0));
    object.pos_curve.x_end =
        std::max(object.longitude_end, static_cast<ScalarType>(0));

    // for polynomial curve on image space
    ScalarType img_y_start = static_cast<ScalarType>(0);
    ScalarType img_y_end = static_cast<ScalarType>(INT_MAX);
    for (size_t j = 0; j < object.image_pos.size(); ++j) {
      ScalarType y = object.image_pos[j](1);
      img_y_start = std::max(img_y_start, y);
      img_y_end = std::min(img_y_end, y);
    }

    // polynomial function: x = a*y^3 + b*y^2 + c*y + d
    Eigen::Matrix<ScalarType, MAX_POLY_ORDER + 1, 1> coeff;
    PolyFit(object.image_pos, object.order, &coeff, false);

    object.img_curve.a = coeff(3);
    object.img_curve.b = coeff(2);
    object.img_curve.c = coeff(1);
    object.img_curve.d = coeff(0);
    object.img_curve.x_start = img_y_start;
    object.img_curve.x_end = img_y_end;
  }

  return true;
}

// Register plugin.
REGISTER_CAMERA_LANE_POST_PROCESSOR(CCLanePostProcessor);

}  // namespace perception
}  // namespace apollo
