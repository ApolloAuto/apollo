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

#include "modules/perception/obstacle/camera/lane_post_process/cc_lane_post_processor/cc_lane_post_processor.h"

#include <algorithm>
#include <cmath>
#include <limits>
#include <numeric>

#include "modules/common/util/file.h"

namespace apollo {
namespace perception {

using apollo::common::util::GetProtoFromFile;
using std::pair;
using std::string;
using std::vector;

bool CCLanePostProcessor::Init() {
  // 1. get model config
  CHECK(GetProtoFromFile(FLAGS_cc_lane_post_processor_config_file, &config_));

  // 2. get parameters
  string space_type = config_.space_type();
  if (space_type == "vehicle") {
    options_.space_type = SpaceType::VEHICLE;
  } else if (space_type == "image") {
    AINFO << "using image space to generate lane instances ...";
    options_.space_type = SpaceType::IMAGE;
  } else {
    AERROR << "invalid space type" << space_type;
    return false;
  }
  options_.frame.space_type = options_.space_type;

  if (!config_.has_image_width() || !config_.has_image_height()) {
    AERROR << "image width or height not found.";
    return false;
  }
  image_width_ = config_.image_width();
  image_height_ = config_.image_height();

  vector<float> roi;
  if (config_.roi_size() != 4) {
    AERROR << "roi format error. size = " << config_.roi_size();
    return false;
  } else {
    roi_.x = config_.roi(0);
    roi_.y = config_.roi(1);
    roi_.width = config_.roi(2);
    roi_.height = config_.roi(3);
    options_.frame.image_roi = roi_;
    ADEBUG << "project ROI = [" << roi_.x << ", " << roi_.y << ", "
           << roi_.x + roi_.width - 1 << ", " << roi_.y + roi_.height - 1
           << "]";
  }

  options_.frame.use_non_mask = config_.use_non_mask();

  if (config_.non_mask_size() % 2 != 0) {
    AERROR << "the number of point coordinate values should be even.";
    return false;
  }
  size_t non_mask_polygon_point_num = config_.non_mask_size() / 2;

  non_mask_.reset(new NonMask(non_mask_polygon_point_num));
  for (size_t i = 0; i < non_mask_polygon_point_num; ++i) {
    non_mask_->AddPolygonPoint(config_.non_mask(2 * i),
                               config_.non_mask(2 * i + 1));
  }

  options_.lane_map_conf_thresh = config_.lane_map_confidence_thresh();
  options_.cc_split_siz = config_.cc_split_siz();
  options_.cc_split_len = config_.cc_split_len();

  // parameters on generating markers
  options_.frame.min_cc_pixel_num = config_.min_cc_pixel_num();
  options_.frame.min_cc_size = config_.min_cc_size();
  options_.frame.min_y_search_offset =
      (options_.frame.space_type == SpaceType::IMAGE
           ? config_.min_y_search_offset_image()
           : config_.min_y_search_offset());

  // parameters on marker association
  string assoc_method = config_.assoc_method();
  if (assoc_method == "greedy_group_connect") {
    options_.frame.assoc_param.method = AssociationMethod::GREEDY_GROUP_CONNECT;
  } else {
    AERROR << "invalid marker association method." << assoc_method;
    return false;
  }

  options_.frame.assoc_param.min_distance =
      (options_.frame.space_type == SpaceType::IMAGE
           ? config_.assoc_min_distance_image()
           : config_.assoc_min_distance());
  ADEBUG << "assoc_min_distance = " << options_.frame.assoc_param.min_distance;

  options_.frame.assoc_param.max_distance =
      (options_.frame.space_type == SpaceType::IMAGE
           ? config_.assoc_max_distance_image()
           : config_.assoc_max_distance());
  ADEBUG << "assoc_max_distance = " << options_.frame.assoc_param.max_distance;

  options_.frame.assoc_param.distance_weight = config_.assoc_distance_weight();
  ADEBUG << "assoc_distance_weight = "
         << options_.frame.assoc_param.distance_weight;

  options_.frame.assoc_param.max_deviation_angle =
      (options_.frame.space_type == SpaceType::IMAGE
           ? config_.assoc_max_deviation_angle_image()
           : config_.assoc_max_deviation_angle());
  ADEBUG << "assoc_max_deviation_angle = "
         << options_.frame.assoc_param.max_deviation_angle;
  options_.frame.assoc_param.max_deviation_angle *= (M_PI / 180.0);

  options_.frame.assoc_param.deviation_angle_weight =
      config_.assoc_deviation_angle_weight();
  ADEBUG << "assoc_deviation_angle_weight = "
         << options_.frame.assoc_param.deviation_angle_weight;

  options_.frame.assoc_param.max_relative_orie =
      (options_.frame.space_type == SpaceType::IMAGE
           ? config_.assoc_max_relative_orie_image()
           : config_.assoc_max_relative_orie());
  ADEBUG << "assoc_max_relative_orie = "
         << options_.frame.assoc_param.max_relative_orie;
  options_.frame.assoc_param.max_relative_orie *= (M_PI / 180.0);

  options_.frame.assoc_param.relative_orie_weight =
      config_.assoc_relative_orie_weight();
  ADEBUG << "assoc_relative_orie_weight = "
         << options_.frame.assoc_param.relative_orie_weight;

  options_.frame.assoc_param.max_departure_distance =
      (options_.frame.space_type == SpaceType::IMAGE
           ? config_.assoc_max_departure_distance_image()
           : config_.assoc_max_departure_distance());
  ADEBUG << "assoc_max_departure_distance = "
         << options_.frame.assoc_param.max_departure_distance;

  options_.frame.assoc_param.departure_distance_weight =
      config_.assoc_departure_distance_weight();
  ADEBUG << "assoc_departure_distance_weight = "
         << options_.frame.assoc_param.departure_distance_weight;

  options_.frame.assoc_param.min_orientation_estimation_size =
      (options_.frame.space_type == SpaceType::IMAGE
           ? config_.assoc_min_orientation_estimation_size_image()
           : config_.assoc_min_orientation_estimation_size());
  ADEBUG << "assoc_min_orientation_estimation_size = "
         << options_.frame.assoc_param.min_orientation_estimation_size;

  if (options_.frame.assoc_param.method ==
      AssociationMethod::GREEDY_GROUP_CONNECT) {
    options_.frame.group_param.max_group_prediction_marker_num =
        config_.max_group_prediction_marker_num();
  } else {
    AERROR << "invalid marker association method.";
    return false;
  }
  options_.frame.orientation_estimation_skip_marker_num =
      config_.orientation_estimation_skip_marker_num();

  // parameters on finding lane objects
  options_.frame.lane_interval_distance = config_.lane_interval_distance();
  options_.frame.min_instance_size_prefiltered =
      (options_.frame.space_type == SpaceType::IMAGE
           ? config_.min_instance_size_prefiltered_image()
           : config_.min_instance_size_prefiltered());
  options_.frame.max_size_to_fit_straight_line =
      (options_.frame.space_type == SpaceType::IMAGE
           ? config_.max_size_to_fit_straight_line_image()
           : config_.max_size_to_fit_straight_line());

  // 3. initialize projector
  max_distance_to_see_ = config_.max_distance_to_see_for_transformer();
  ADEBUG << "initial max_distance_to_see: " << max_distance_to_see_
         << " (meters)";

  if (options_.space_type == SpaceType::VEHICLE) {
    projector_.reset(new Projector<ScalarType>());
    projector_->Init(roi_, max_distance_to_see_, vis_);
    is_x_longitude_ = true;
  } else if (options_.space_type == SpaceType::IMAGE) {
    is_x_longitude_ = false;
  } else {
    AERROR << "invalid space type" << space_type;
    return false;
  }

  time_stamp_ = 0.0;
  frame_id_ = 0;

  int lane_map_width = config_.lane_map_width();
  int lane_map_height = config_.lane_map_height();
#if CUDA_CC
  cc_generator_.reset(
      new ConnectedComponentGeneratorGPU(image_width_, image_height_, roi_));
#else
  cc_generator_.reset(
      new ConnectedComponentGenerator(lane_map_width, lane_map_height,
      cv::Rect(0, 0, lane_map_width, lane_map_height)));
#endif

  scale_ = config_.lane_map_scale();
  start_y_pos_ = config_.start_y_pos();
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
  // Option 1: Use C0 for lateral distance
  lane_object->lateral_distance = lane_object->model(0);
  // Option 2: Use y-value of closest point.
  // lane_object->lateral_distance = lane_object->pos[0].y();

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

//  ADEBUG << "show points for lane object: ";

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

  // Option 1: Use C0 for lateral distance
  lane_object->lateral_distance = lane_object->model(0);
  // Option 2: Use y-value of closest point.
  // lane_object->lateral_distance = lane_object->pos[0].y();

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

  // 1. get binary lane label mask
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

  // 2. find connected components from lane label mask
  vector<ConnectedComponentPtr> cc_list;
  cc_generator_->FindConnectedComponents(lane_mask, &cc_list);

  ADEBUG << "number of connected components = " << cc_list.size();

  // 3. split CC and find inner edges
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

  /// 4. do lane marker association and determine lane instance labels
  cur_frame_.reset(new LaneFrame);

  if (options_.frame.space_type == SpaceType::IMAGE) {
    cur_frame_->Init(cc_list, non_mask_, options_.frame, scale_, start_y_pos_);
  } else if (options_.frame.space_type == SpaceType::VEHICLE) {
    cur_frame_->Init(cc_list, non_mask_, projector_,
      options_.frame, scale_, start_y_pos_);
  } else {
    AERROR << "unknown space type: " << options_.frame.space_type;
    return false;
  }

  cur_frame_->Process(cur_lane_instances_);

//  ADEBUG << "number of lane instances = " << cur_lane_instances_->size();

  return true;
}

bool CCLanePostProcessor::Process(const cv::Mat &lane_map,
                                  const CameraLanePostProcessOptions &options,
                                  LaneObjectsPtr *lane_objects) {
  if (!is_init_) {
    AERROR << "lane post-processor is not initialized";
    return false;
  }

  if (lane_map.empty()) {
    AERROR << "input lane map is empty";
    return false;
  }

  if (options.use_lane_history &&
      (!use_history_ || time_stamp_ > options.timestamp)) {
    InitLaneHistory();
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
  if (options_.space_type == SpaceType::IMAGE) {
    /// for image space coordinate
    ScalarType x_center = static_cast<ScalarType>(roi_.x + roi_.width / 2);

    lane_objects->reset(new LaneObjects());
    (*lane_objects)->reserve(2);
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
        (*lane_objects)->push_back(cur_object);
        (*lane_objects)->back().spatial = SpatialLabelType::L_0;
        is_left_lane_found = true;
      } else {
        // for right lane
        if (is_right_lane_found) {
          continue;
        }
        (*lane_objects)->push_back(cur_object);
        (*lane_objects)->back().spatial = SpatialLabelType::R_0;
        is_right_lane_found = true;
      }

      ADEBUG << " lane object " << (*lane_objects)->back().GetSpatialLabel()
             << " has " << (*lane_objects)->back().pos.size() << " points: "
             << "lateral distance = "
             << (*lane_objects)->back().lateral_distance;
    }

  } else {
    /// for vehicle space coordinate
    // select lane instances with non-overlap assumption
    // ADEBUG << "generate lane objects ...";
    lane_objects->reset(new LaneObjects());
    (*lane_objects)->reserve(2 * MAX_LANE_SPATIAL_LABELS);
    vector<pair<ScalarType, int>> origin_lateral_dist_object_id;
    origin_lateral_dist_object_id.reserve(2 * MAX_LANE_SPATIAL_LABELS);
    int count_lane_objects = 0;
    ADEBUG << "cur_lane_instances_->size(): " << cur_lane_instances_->size();
    for (auto it = cur_lane_instances_->begin();
         it != cur_lane_instances_->end(); ++it) {
       ADEBUG << "for lane instance " << it - cur_lane_instances_->begin();

      // ignore current instance if it is too small
      if (it->siz < options_.frame.min_instance_size_prefiltered) {
        ADEBUG << "current lane instance is too small: " << it->siz;
        continue;
      }

      LaneObject cur_object;
      AddInstanceIntoLaneObject(*it, &cur_object);

      if ((*lane_objects)->empty()) {
        // create a new lane object

        (*lane_objects)->push_back(cur_object);
        ADEBUG << " lane object XXX has"
             << (*lane_objects)->at(count_lane_objects).pos.size()
             << " points, lateral distance="
             << (*lane_objects)->at(count_lane_objects).lateral_distance;
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
      for (size_t k = 0; k < (*lane_objects)->size(); ++k) {
        // min distance to instance group
        lateral_distances[k].first = std::numeric_limits<ScalarType>::max();
        // max distance to instance group
        lateral_distances[k].second = -std::numeric_limits<ScalarType>::max();

        // determine whether cross over or not
        for (size_t i = 0; i < cur_object.pos.size(); ++i) {
          ScalarType delta_y =
              cur_object.pos[i].y() - PolyEval(cur_object.pos[i].x(),
                                               (*lane_objects)->at(k).order,
                                               (*lane_objects)->at(k).model);
          // lateral_distances[k].first keeps min delta_y of lane line points
          // from the fitted curve
          lateral_distances[k].first =
              std::min(lateral_distances[k].first, delta_y);
          // lateral_distances[k].first keeps max delta_y of lane line points
          // from the fitted curve
          lateral_distances[k].second =
              std::max(lateral_distances[k].second, delta_y);
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
        ADEBUG << "Lane " << cross_over_lane_object_id
               << "crosses over cur_lane. Eliminated.";
        for (size_t i = 0; i < cur_object.pos.size(); ++i) {
          ADEBUG << "[" << cur_object.pos[i].x() << ", "
                 << cur_object.pos[i].y() << "]";
        }
        continue;
      }

      // search the left-most lane w.r.t. current instance so far
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

      // search the right-most lane w.r.t. current instance so far
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
      (*lane_objects)->push_back(cur_object);
        ADEBUG << " lane object XXX has"
            << (*lane_objects)->at(count_lane_objects).pos.size()
            << " points, lateral distance="
            << (*lane_objects)->at(count_lane_objects).lateral_distance;
      origin_lateral_dist_object_id.push_back(
          std::make_pair(cur_object.lateral_distance, count_lane_objects++));
//      ADEBUG << "generate a new lane object from instance.";
    }

    // determine spatial label of lane object
    // Sort lanes with C0
    std::sort(origin_lateral_dist_object_id.begin(),
              origin_lateral_dist_object_id.end(),
              [](const pair<ScalarType, int> &a,
                 const pair<ScalarType, int> &b) { return a.first > b.first; });

    int index_closest_left = -1;
    for (int k = 0; k < count_lane_objects; ++k) {
      if (origin_lateral_dist_object_id[k].first >= 0) {
        index_closest_left = k;
      } else {
        break;
      }
    }

    std::vector<bool> b_left_index_list(MAX_LANE_SPATIAL_LABELS, false);
    vector<int> valid_lane_objects;
    valid_lane_objects.reserve((*lane_objects)->size());

    // for left-side lanes
    for (int spatial_index = 0; spatial_index <= index_closest_left;
         ++spatial_index) {
      if (spatial_index >= MAX_LANE_SPATIAL_LABELS) {
        break;
      }
      int i_l = index_closest_left - spatial_index;
      int object_id = origin_lateral_dist_object_id.at(i_l).second;
      float lateral_distance = origin_lateral_dist_object_id.at(i_l).first;

      int index = floor(lateral_distance * INVERSE_AVEAGE_LANE_WIDTH_METER);
      if (index < 0 || index > MAX_LANE_SPATIAL_LABELS - 1) {
        continue;
      }
      if (b_left_index_list[index] == true) {
        continue;
      }
      b_left_index_list[index] = true;
      (*lane_objects)->at(object_id).spatial =
          static_cast<SpatialLabelType>(index);
      valid_lane_objects.push_back(object_id);
      ADEBUG << " lane object "
            << (*lane_objects)->at(object_id).GetSpatialLabel() << " has "
            << (*lane_objects)->at(object_id).pos.size() << " points: "
            << "lateral distance="
            << (*lane_objects)->at(object_id).lateral_distance;
    }
    // for right-side lanes
    std::vector<bool> b_right_index_list(MAX_LANE_SPATIAL_LABELS, false);
    int i_r = index_closest_left + 1;
    for (int spatial_index = 0; spatial_index < MAX_LANE_SPATIAL_LABELS;
         ++spatial_index, ++i_r) {
      if (i_r >= count_lane_objects) {
        break;
      }
      int object_id = origin_lateral_dist_object_id.at(i_r).second;
      float lateral_distance = -origin_lateral_dist_object_id.at(i_r).first;

      int index = floor(lateral_distance * INVERSE_AVEAGE_LANE_WIDTH_METER);
      if (index < 0 || index > MAX_LANE_SPATIAL_LABELS - 1) {
        continue;
      }
      if (b_right_index_list[index] == true) {
        continue;
      }
      b_right_index_list[index] = true;
      (*lane_objects)->at(object_id).spatial =
          static_cast<SpatialLabelType>(MAX_LANE_SPATIAL_LABELS + index);

      valid_lane_objects.push_back(object_id);
      ADEBUG << " lane object "
            << (*lane_objects)->at(object_id).GetSpatialLabel() << " has "
            << (*lane_objects)->at(object_id).pos.size() << " points: "
            << "lateral distance="
            << (*lane_objects)->at(object_id).lateral_distance;
    }
    if ((*lane_objects)->size() != static_cast<size_t>(count_lane_objects)) {
      AERROR << "the number of lane objects does not match.";
      return false;
    }

    // keep only the lane objects with valid spatial labels
    std::sort(valid_lane_objects.begin(), valid_lane_objects.end());
    for (size_t i = 0; i < valid_lane_objects.size(); ++i) {
      (*lane_objects)->at(i) = (*lane_objects)->at(valid_lane_objects[i]);
    }
    (*lane_objects)->resize(valid_lane_objects.size());
  }

  ADEBUG << "number of lane objects = " << (*lane_objects)->size();
  // if (options_.space_type != SpaceType::IMAGE) {
  //   if (!CompensateLaneObjects((*lane_objects))) {
  //     AERROR << "fail to compensate lane objects.";
  //     return false;
  //   }
  // }

  EnrichLaneInfo((*lane_objects));
  ADEBUG << "use_lane_history_: " << use_history_;
  if (use_history_) {
    //    FilterWithLaneHistory(*lane_objects);
    std::vector<bool> is_valid(generated_lanes_->size(), false);
    size_t l = 0;
    for (l = 0; l < generated_lanes_->size(); l++) {
      CorrectWithLaneHistory(l, *lane_objects, &is_valid);
    }

    l = 0;
    while (l < is_valid.size() && !is_valid[l]) {
      l++;
    }
    if (l < is_valid.size()) {
      lane_history_.push_back(*(*lane_objects));
    } else {
      if (!lane_history_.empty()) {
        lane_history_.pop_front();
      }
    }

    for (l = 0; l < is_valid.size(); l++) {
      if (!is_valid[l]) {
        (*lane_objects)->push_back(generated_lanes_->at(l));
        AINFO << "use history instead of current lane detection";
        AINFO << generated_lanes_->at(l).model;
      }
    }
#if USE_HISTORY_TO_EXTEND_LANE
    for (size_t i = 0; i < generated_lanes_->size(); i++) {
      if (is_valid[i]) {
        int j = 0;
        if (FindLane(*(*lane_objects),
            generated_lanes_->at(i).spatial, &j)) {
          ExtendLaneWithHistory(generated_lanes_->at(i),
                                &((*lane_objects)->at(j)));
        }
      }
    }
#endif  // USE_HISTORY_TO_EXTEND_LANE
    auto vs = options.vehicle_status;
    for (auto &m : *motion_buffer_) {
      m.motion *= vs.motion;
    }
    motion_buffer_->push_back(vs);
  }
  return true;
}

bool CCLanePostProcessor::CorrectWithLaneHistory(int l,
        LaneObjectsPtr lane_objects, std::vector<bool> *is_valid) {
    // trust current lane or not
    auto &lane = generated_lanes_->at(l);
    lane.pos.clear();
    lane.longitude_start = std::numeric_limits<ScalarType>::max();
    lane.longitude_end = 0;
    lane.order = 0;
    int lane_accum_num = 0;
    for (std::size_t i = 0; i < lane_history_.size(); i++) {
      int j = 0;
      if (!FindLane(lane_history_[i], lane.spatial, &j)) continue;

      lane_accum_num++;
      lane.order = std::max(lane.order, lane_history_[i][j].order);
      Vector3D p;
      Vector2D project_p;
      for (auto &pos : lane_history_[i][j].pos) {
        p << pos.x(), pos.y(), 1.0;
        p = motion_buffer_->at(i).motion * p;
        project_p << p.x(), p.y();
        if (p.x() <= 0) continue;

        lane.longitude_start = std::min(p.x(), lane.longitude_start);
        lane.longitude_end = std::max(p.x(), lane.longitude_end);
        lane.pos.push_back(project_p);
      }
    }
    // fit polynomial model and compute lateral distance for lane object
    lane.point_num = lane.pos.size();

    if (lane.point_num < 3 ||
        lane.longitude_end - lane.longitude_start <
            options_.frame.max_size_to_fit_straight_line) {
      // fit a 1st-order polynomial curve (straight line)
      lane.order = 1;
    } else {
      // fit a 2nd-order polynomial curve;
      lane.order = 2;
    }
    ADEBUG << "history size: " << lane.point_num;
    if (lane_accum_num < 2 ||lane.point_num < 2 ||
        lane.longitude_end - lane.longitude_start < 4.0) {
      AWARN << "Failed to use history: " << lane_accum_num
            << " " << lane.point_num << " "
            << lane.longitude_end - lane.longitude_start;
      (*is_valid)[l] = true;
      return (*is_valid)[l];
    } else if (!PolyFit(lane.pos, lane.order, &(lane.model))) {
      AWARN << "failed to fit " << lane.order << " order polynomial curve.";
      (*is_valid)[l] = true;
      return (*is_valid)[l];
    }
    lane.pos_curve.x_start =
        std::min(lane.longitude_start, static_cast<ScalarType>(0));
    lane.pos_curve.x_end =
        std::max(lane.longitude_end, static_cast<ScalarType>(0));
    // for polynomial curve on vehicle space
    lane.pos_curve.a = lane.model(3);
    lane.pos_curve.b = lane.model(2);
    lane.pos_curve.c = lane.model(1);
    lane.pos_curve.d = lane.model(0);


    // Option 1: Use C0 for lateral distance
    // lane_object->lateral_distance = lane_object->model(0);
    // Option 2: Use y-value of closest point.
    lane.lateral_distance = lane.pos[0].y();
    int idx = 0;
    if (!FindLane(*lane_objects, lane.spatial, &idx)) {
    //  lane_objects->push_back(lane);
    } else {
      ScalarType ave_delta = 0;
      int count = 0;

      for (auto &pos : lane_objects->at(idx).pos) {
        if (pos.x() > 1.2 * lane.longitude_end) continue;

        ave_delta +=
         std::abs(pos.y() - PolyEval(pos.x(), lane.order, lane.model));
        count++;
      }
      if (count > 0 && ave_delta / count > AVEAGE_LANE_WIDTH_METER / 4.0) {
        ADEBUG << "ave_delta is: " << ave_delta / count;
        lane_objects->erase(lane_objects->begin() + idx);
      } else {
        (*is_valid)[l] = true;
      }
    }
  return (*is_valid)[l];
}

void CCLanePostProcessor::ExtendLaneWithHistory(
        const LaneObject &history, LaneObject *lane) {
  if (history.longitude_end > lane->longitude_end) {
    for (auto &p : history.pos) {
      if (p.x() > lane->longitude_end) {
        lane->pos.push_back(p);
      }
    }
    AINFO << "extend lane with history by "
          << history.longitude_end - lane->longitude_end;
    lane->longitude_end = history.longitude_end;
    lane->pos_curve.x_end =
        std::max(lane->longitude_end, static_cast<ScalarType>(0));
  }
}

bool CCLanePostProcessor::FindLane(const LaneObjects &lane_objects,
                                   int spatial_label, int *index) {
  size_t k = 0;
  while (k < lane_objects.size() &&
         lane_objects.at(k).spatial != spatial_label) {
    k++;
  }
  if (k == lane_objects.size()) {
    return false;
  } else {
    *index = k;
    return true;
  }
}

void CCLanePostProcessor::InitLaneHistory() {
  use_history_ = true;
  AINFO << "Init Lane History Start;";
  if (!lane_history_.empty()) {
    lane_history_.clear();
  } else {
    lane_history_.set_capacity(MAX_LANE_HISTORY);
  }
  if (motion_buffer_ != nullptr) {
    motion_buffer_->clear();
  } else {
    motion_buffer_ = std::make_shared<MotionBuffer>(MAX_LANE_HISTORY);
  }
  if (generated_lanes_ != nullptr) {
    generated_lanes_->clear();
    generated_lanes_->resize(interested_labels_.size(), LaneObject());
  } else {
    generated_lanes_ =
      std::make_shared<LaneObjects>(interested_labels_.size(), LaneObject());
  }
  for (std::size_t i = 0; i < generated_lanes_->size(); i++) {
    generated_lanes_->at(i).spatial = interested_labels_[i];
  }
  AINFO << "Init Lane History Done;";
}

void CCLanePostProcessor::FilterWithLaneHistory(LaneObjectsPtr lane_objects) {
  std::vector<int> erase_idx;
  for (size_t i = 0; i < lane_objects->size(); i++) {
    Eigen::Vector3f start_pos;
    start_pos << lane_objects->at(i).pos[0].x(), lane_objects->at(i).pos[0].y(),
        1.0;

    for (size_t j = 0; j < lane_history_.size(); j++) {
      // iter to find corresponding lane
      size_t k;
      for (k = 0; k < lane_history_[j].size(); k++) {
        if (lane_history_[j][k].spatial == lane_objects->at(i).spatial) {
          break;
        }
      }
      // if not exist, skip
      if (k == lane_history_[j].size()) {
        continue;
      }
      // project start_pos to history, check lane stability
      auto project_pos = motion_buffer_->at(j).motion * start_pos;
      auto &lane_object = lane_history_[j][k];
      ScalarType delta_y =
          project_pos.y() -
          PolyEval(project_pos.x(), lane_object.order, lane_object.model);
      // delete if too far from polyline
      if (std::abs(delta_y) > AVEAGE_LANE_WIDTH_METER) {
        erase_idx.push_back(i);
        break;
      }
    }
  }
  for (size_t i = erase_idx.size() - 1; i >= 0; i--) {
    lane_objects->erase(lane_objects->begin() + erase_idx[i]);
  }
}

bool CCLanePostProcessor::CompensateLaneObjects(LaneObjectsPtr lane_objects) {
  if (lane_objects == nullptr) {
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
    if (ego_lane_right_idx == -1) {
      AERROR << "failed to compensate left ego lane due to no right ego lane.";
      return false;
    }

    LaneObject virtual_ego_lane_left = lane_objects->at(ego_lane_right_idx);

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
    ADEBUG << "add virtual lane R_0 ...";
    if (ego_lane_left_idx == -1) {
      AERROR << "failed to compensate right ego lane due to no left ego lane.";
      return false;
    }

    LaneObject virtual_ego_lane_right = lane_objects->at(ego_lane_left_idx);

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
  if (lane_objects == nullptr) {
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

}  // namespace perception
}  // namespace apollo
