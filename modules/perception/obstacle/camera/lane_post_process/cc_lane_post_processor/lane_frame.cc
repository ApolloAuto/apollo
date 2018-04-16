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

#include "modules/perception/obstacle/camera/lane_post_process/cc_lane_post_processor/lane_frame.h"

#include <algorithm>
#include <limits>

#include "boost/filesystem.hpp"

namespace apollo {
namespace perception {

using std::make_pair;
using std::pair;
using std::shared_ptr;
using std::sort;
using std::string;
using std::unordered_map;
using std::unordered_set;
using std::vector;

/// for class LaneFrame
void LaneFrame::ComputeBbox() {
  boxes_.clear();
  boxes_.reserve(graphs_.size());
  for (size_t k = 0; k < graphs_.size(); ++k) {
    boxes_.push_back(Bbox());
    boxes_[k](0) = std::numeric_limits<ScalarType>::max();   // x_min
    boxes_[k](1) = std::numeric_limits<ScalarType>::max();   // y_min
    boxes_[k](2) = -std::numeric_limits<ScalarType>::max();  // x_max
    boxes_[k](3) = -std::numeric_limits<ScalarType>::max();  // y_max

    for (size_t j = 0; j < graphs_[k].size(); ++j) {
      int i = graphs_[k][j].first;
      boxes_[k](0) = std::min(boxes_[k](0), markers_.at(i).pos(0));
      boxes_[k](1) = std::min(boxes_[k](1), markers_.at(i).pos(1));
      boxes_[k](2) = std::max(boxes_[k](2), markers_.at(i).pos(0));
      boxes_[k](3) = std::max(boxes_[k](3), markers_.at(i).pos(1));

      if (markers_.at(i).shape_type != MarkerShapeType::POINT) {
        boxes_[k](0) = std::min(boxes_[k](0), markers_.at(i).start_pos(0));
        boxes_[k](1) = std::min(boxes_[k](1), markers_.at(i).start_pos(1));
        boxes_[k](2) = std::max(boxes_[k](2), markers_.at(i).start_pos(0));
        boxes_[k](3) = std::max(boxes_[k](3), markers_.at(i).start_pos(1));
      }
    }
  }
}

ScalarType LaneFrame::ComputeMarkerPairDistance(const Marker& ref,
                                                const Marker& tar) {
  ADEBUG << "compute distance between markers ...";
  ADEBUG << "reference marker: "
         << "x=" << std::to_string(ref.pos(0)) << ", "
         << "y=" << std::to_string(ref.pos(1));
  ADEBUG << "target marker: "
         << "x=" << std::to_string(tar.start_pos(0)) << ", "
         << "y=" << std::to_string(tar.start_pos(1));

  ScalarType dist = static_cast<ScalarType>(-1.0);

  Vector2D displacement;
  switch (ref.shape_type) {
    case MarkerShapeType::POINT: {
      displacement = tar.pos - ref.pos;
      break;
    }
    case MarkerShapeType::LINE_SEGMENT: {
      displacement = tar.start_pos - ref.pos;
      break;
    }
    default: { AERROR << "unknown marker shape type."; }
  }

  ScalarType pos_dist = static_cast<ScalarType>(displacement.norm());
  ADEBUG << "pos_dist = " << std::to_string(pos_dist);

  // orientation angle of target marker
  ScalarType alpha = tar.angle;
  if (alpha < 0) {
    alpha += (2 * static_cast<ScalarType>(M_PI));
  }
  ADEBUG << "alpha = " << std::to_string(alpha / M_PI * 180.0);

  // orientation angle of reference marker
  ScalarType beta = ref.angle;
  if (beta < 0) {
    beta += (2 * static_cast<ScalarType>(M_PI));
  }
  ADEBUG << "beta = " << std::to_string(beta / M_PI * 180.0);

  // deviation angle from reference marker to the target one
  ScalarType gamma = std::atan2(displacement(1), displacement(0));
  if (gamma < 0) {
    gamma += (2 * static_cast<ScalarType>(M_PI));
  }
  ADEBUG << "gamma = " << std::to_string(gamma / M_PI * 180.0);

  ScalarType deviation_angle_dist = std::abs(beta - gamma);
  if (deviation_angle_dist > static_cast<ScalarType>(M_PI)) {
    deviation_angle_dist =
        2 * static_cast<ScalarType>(M_PI) - deviation_angle_dist;
  }
  ADEBUG << "deviation_angle_dist = "
         << std::to_string(deviation_angle_dist / M_PI * 180.0);

  ScalarType orie_dist = std::abs(alpha - beta);

  if (orie_dist > static_cast<ScalarType>(M_PI)) {
    orie_dist = 2 * static_cast<ScalarType>(M_PI) - orie_dist;
  }

  ADEBUG << "orie_dist = " << std::to_string(orie_dist / M_PI * 180.0);
  ADEBUG << "max_distance = " << std::to_string(opts_.assoc_param.max_distance)
         << ", "
         << "max_deviation_angle = "
         << std::to_string(opts_.assoc_param.max_deviation_angle / M_PI * 180.0)
         << ", "
         << "max_relative_orie = "
         << std::to_string(opts_.assoc_param.max_relative_orie / M_PI * 180.0);

  if (pos_dist > opts_.assoc_param.max_distance ||
      deviation_angle_dist > opts_.assoc_param.max_deviation_angle ||
      orie_dist > opts_.assoc_param.max_relative_orie) {
    return dist;
  }

  pos_dist /= opts_.assoc_param.max_distance;
  deviation_angle_dist /= opts_.assoc_param.max_deviation_angle;
  orie_dist /= opts_.assoc_param.max_relative_orie;

  dist = opts_.assoc_param.distance_weight * pos_dist +
         opts_.assoc_param.deviation_angle_weight * deviation_angle_dist +
         opts_.assoc_param.relative_orie_weight * orie_dist;
  return dist;
}

bool LaneFrame::Init(const vector<ConnectedComponentPtr>& input_cc,
                     const shared_ptr<NonMask>& non_mask,
                     const LaneFrameOptions& options,
                     const double scale,
                     const int start_y_pos) {
  if (options.space_type != SpaceType::IMAGE) {
    AERROR << "the space type is not IMAGE.";
    return false;
  }
  opts_ = options;

  max_cc_num_ = static_cast<int>(input_cc.size());
  markers_.clear();
  for (int i = 0; i < static_cast<int>(input_cc.size()); ++i) {
    const ConnectedComponentPtr cc_ptr = input_cc.at(i);
    if (cc_ptr->GetPixelCount() >= opts_.min_cc_pixel_num &&
        (cc_ptr->bbox()->width() >= opts_.min_cc_size ||
         cc_ptr->bbox()->height() >= opts_.min_cc_size)) {
      const shared_ptr<vector<ConnectedComponent::Edge>>& inner_edges =
          cc_ptr->GetInnerEdges();
      int n_inner_edges = static_cast<int>(inner_edges->size());
      for (int j = 0; j < n_inner_edges; ++j) {
        const ConnectedComponent::Edge* edge_ptr = &(inner_edges->at(j));
        Marker marker;
        marker.shape_type = MarkerShapeType::LINE_SEGMENT;
        marker.space_type = opts_.space_type;

        marker.pos = cc_ptr->GetVertex(edge_ptr->end_vertex_id,
          scale, start_y_pos);
        marker.image_pos = marker.pos;
        if (opts_.use_non_mask &&
            non_mask->IsInsideMask(marker.image_pos)) {
          ADEBUG << "the marker with end point ("
                 << marker.image_pos.x() << ", "
                 << marker.image_pos.y() << ") is filtered by non_mask.";
          continue;
        }
        marker.vis_pos = cv::Point(static_cast<int>(marker.pos.x()),
                                   static_cast<int>(marker.pos.y()));

        marker.start_pos = cc_ptr->GetVertex(edge_ptr->start_vertex_id,
          scale, start_y_pos);
        marker.image_start_pos = marker.start_pos;
        if (opts_.use_non_mask &&
            non_mask->IsInsideMask(marker.image_start_pos)) {
          ADEBUG << "the marker with start point ("
                 << marker.image_start_pos.x() << ", "
                 << marker.image_start_pos.y() << ") is filtered by non_mask.";
          continue;
        }
        marker.vis_start_pos =
            cv::Point(static_cast<int>(marker.start_pos.x()),
                      static_cast<int>(marker.start_pos.y()));

        marker.angle = edge_ptr->orie;
        if (marker.angle < -static_cast<ScalarType>(M_PI) ||
            marker.angle > static_cast<ScalarType>(M_PI)) {
          AERROR << "marker.angle is out range of [-pi, pi]: " << marker.angle;
          return false;
        }
        marker.orie(0) = std::cos(marker.angle);
        marker.orie(1) = std::sin(marker.angle);
        marker.original_id = static_cast<int>(markers_.size());
        marker.cc_id = i;
        marker.inner_edge_id = j;
        marker.cc_edge_ascend_id = j;
        marker.cc_edge_descend_id = n_inner_edges - 1 - j;
        if (marker.cc_edge_descend_id < 0 ||
            marker.cc_edge_descend_id >= n_inner_edges) {
          AERROR << "marker.cc_edge_descend_id = " << marker.cc_edge_descend_id;
          return false;
        }

        if (!markers_.empty() && markers_.back().cc_id == i) {
          markers_.back().cc_next_marker_id = static_cast<int>(markers_.size());
        }
        markers_.push_back(marker);
      }
    }
  }

  for (int i = 0; i < static_cast<int>(markers_.size()); ++i) {
    if (markers_[i].original_id != i) {
      AERROR << "original marker id is not equal to " << i;
      return false;
    }
    if (markers_[i].cc_id < 0) {
      AERROR << "cc id is less than 0: " << markers_[i].cc_id;
      return false;
    }
    if (markers_[i].inner_edge_id < 0) {
      AERROR << "inner_edge_id is less than 0: " << markers_[i].inner_edge_id;
      return false;
    }
    if (markers_[i].cc_edge_ascend_id < 0) {
      AERROR << "cc_edge_ascend_id is less than 0: "
             << markers_[i].cc_edge_ascend_id;
      return false;
    }
    if (markers_[i].cc_edge_descend_id < 0) {
      AERROR << "cc_edge_descend_id is less than 0: "
             << markers_[i].cc_edge_descend_id;
      return false;
    }

    int i_next = markers_[i].cc_next_marker_id;
    if (i_next != -1) {
      if (i_next < 0) {
        AERROR << "cc_next_marker_id is less than 0: "
               << markers_[i].cc_next_marker_id;
        return false;
      }
      if (i_next >= static_cast<int>(markers_.size())) {
        AERROR << "cc_next_marker_id is larger than marker size: "
               << markers_[i].cc_next_marker_id;
        return false;
      }

      if (markers_[i_next].cc_id != markers_[i].cc_id) {
        AERROR << "markers_[i_next].cc_id (" << markers_[i_next].cc_id << ") "
               << "is not equal to markers_[i].cc_id (" << markers_[i].cc_id
               << ").";
        return false;
      }
      if (markers_[i_next].cc_edge_ascend_id !=
          markers_[i].cc_edge_ascend_id + 1) {
        AERROR << "markers_[i_next].cc_edge_ascend_id ("
               << markers_[i_next].cc_edge_ascend_id << ") "
               << "is not equal to markers_[i].cc_edge_ascend_id + 1 ("
               << markers_[i].cc_edge_ascend_id + 1 << ").";
        return false;
      }
      if (markers_[i_next].cc_edge_descend_id + 1 !=
          markers_[i].cc_edge_descend_id) {
        AERROR << "markers_[i_next].cc_edge_descend_id + 1 ("
               << markers_[i_next].cc_edge_descend_id << ") "
               << "is not equal to markers_[i].cc_edge_descend_id ("
               << markers_[i].cc_edge_descend_id << ").";
        return false;
      }
    }
  }

  is_projector_init_ = false;

  return true;
}

bool LaneFrame::Init(const vector<ConnectedComponentPtr>& input_cc,
                     const shared_ptr<NonMask>& non_mask,
                     const shared_ptr<Projector<ScalarType>>& projector,
                     const LaneFrameOptions& options,
                     const double scale,
                     const int start_y_pos) {
  if (options.space_type != SpaceType::VEHICLE) {
    AERROR << "the space type is not VEHICLE.";
    return false;
  }
  opts_ = options;

  projector_ = projector;
  is_projector_init_ = true;

  max_cc_num_ = static_cast<int>(input_cc.size());
  markers_.clear();
  for (int i = 0; i < static_cast<int>(input_cc.size()); ++i) {
    const ConnectedComponentPtr cc_ptr = input_cc.at(i);
    ADEBUG << "cc " << i;
    if (cc_ptr->GetPixelCount() >= opts_.min_cc_pixel_num) {
      const shared_ptr<vector<ConnectedComponent::Edge>>& inner_edges =
          cc_ptr->GetInnerEdges();

      int n = 0;
      for (int j = 0; j < static_cast<int>(inner_edges->size()); ++j) {
        const ConnectedComponent::Edge* edge_ptr = &(inner_edges->at(j));
        Vector2D pos = cc_ptr->GetVertex(edge_ptr->end_vertex_id,
          scale, start_y_pos);
        Vector2D start_pos = cc_ptr->GetVertex(edge_ptr->start_vertex_id,
          scale, start_y_pos);

        Marker marker;
        marker.shape_type = MarkerShapeType::LINE_SEGMENT;
        marker.space_type = opts_.space_type;

        if (opts_.use_non_mask && non_mask->IsInsideMask(pos)) {
          ADEBUG << "the marker with end point ("
                 << pos(0) << ", "
                 << pos(1) << ") is filtered by non_mask.";
          continue;
        }
        marker.image_pos = pos;
        if (!projector_->UvToXy(static_cast<ScalarType>(pos(0)),
                                static_cast<ScalarType>(pos(1)),
                                &(marker.pos))) {
          ADEBUG << "the marker with end point ("
                 << pos(0) << ", "
                 << pos(1) << ") is filtered by projector.";
          continue;
        }
        if (projector_->is_vis()) {
          if (!projector_->UvToXyImagePoint(static_cast<ScalarType>(pos(0)),
                                            static_cast<ScalarType>(pos(1)),
                                            &marker.vis_pos)) {
            return false;
          }
        }

        if (opts_.use_non_mask && non_mask->IsInsideMask(start_pos)) {
          ADEBUG << "the marker with start point ("
                 << start_pos(0) << ", "
                 << start_pos(1) << ") is filtered by non_mask.";
          continue;
        }
        marker.image_start_pos = start_pos;
        if (!projector_->UvToXy(static_cast<ScalarType>(start_pos(0)),
                                static_cast<ScalarType>(start_pos(1)),
                                &(marker.start_pos))) {
          ADEBUG << "the marker with start point ("
                 << start_pos(0) << ", "
                 << start_pos(1) << ") is filtered by projector.";
          continue;
        }
        if (projector_->is_vis()) {
          if (!projector_->UvToXyImagePoint(
                  static_cast<ScalarType>(start_pos(0)),
                  static_cast<ScalarType>(start_pos(1)),
                  &marker.vis_start_pos)) {
            return false;
          }
        }

        marker.orie = marker.pos - marker.start_pos;
        marker.angle = std::atan2(marker.orie(1), marker.orie(0));
        if (marker.angle < -static_cast<ScalarType>(M_PI) ||
            marker.angle > static_cast<ScalarType>(M_PI)) {
          AERROR << "marker.angle is out range of [-pi, pi]: " << marker.angle;
          return false;
        }
        marker.orie(0) = std::cos(marker.angle);
        marker.orie(1) = std::sin(marker.angle);
        marker.original_id = static_cast<int>(markers_.size());
        marker.cc_id = i;
        marker.inner_edge_id = j;
        marker.cc_edge_ascend_id = n++;
        if (n <= 0 || n > static_cast<int>(inner_edges->size())) {
          AERROR << "n is out range of [0, " << inner_edges->size() << "]"
                 << ": " << n;
          return false;
        }

        if (!markers_.empty() && markers_.back().cc_id == i) {
          markers_.back().cc_next_marker_id = static_cast<int>(markers_.size());
        }
        markers_.push_back(marker);

        ADEBUG << "marker " << n << ": "
               << "(" << marker.image_pos(0) << ", " << marker.image_pos(1)
               << ")"
               << " => "
               << "(" << marker.pos(0) << ", " << marker.pos(1) << ")";
      }

      if (n > static_cast<int>(inner_edges->size())) {
        AERROR << "n is larger than inner edge size " << inner_edges->size()
               << ": " << n;
        return false;
      }

      int m = static_cast<int>(markers_.size());
      for (int j = 0; j < n; ++j) {
        markers_[m - 1 - j].cc_edge_descend_id = j;
      }

      bool is_small_cc = false;
      if (n > 0) {
        ScalarType x_min = std::numeric_limits<ScalarType>::max();
        ScalarType x_max = -std::numeric_limits<ScalarType>::max();
        ScalarType y_min = std::numeric_limits<ScalarType>::max();
        ScalarType y_max = -std::numeric_limits<ScalarType>::max();
        for (int j = m - n; j < m; ++j) {
          x_min = std::min(x_min, markers_[j].start_pos(0));
          x_max = std::max(x_max, markers_[j].start_pos(0));
          y_min = std::min(y_min, markers_[j].start_pos(1));
          y_max = std::max(y_max, markers_[j].start_pos(1));

          x_min = std::min(x_min, markers_[j].pos(0));
          x_max = std::max(x_max, markers_[j].pos(0));
          y_min = std::min(y_min, markers_[j].pos(1));
          y_max = std::max(y_max, markers_[j].pos(1));
        }
        if (x_max - x_min < 0.5 && y_max - y_min < 0.5) {
          ADEBUG << "x_min = " << x_min << ", "
                << "x_max = " << x_max << ", "
                << "width = " << x_max - x_min << ", "
                << "y_min = " << y_min << ", "
                << "y_max = " << y_max << ", "
                << "height = " << y_max - y_min;
          ADEBUG << "this cc is too small, ignore it.";
          is_small_cc = true;
        }
      }
      if (is_small_cc) {
        markers_.resize(m - n);
      }
    }
  }

  return true;
}

// compute the candidate edges for each marker
vector<int> LaneFrame::ComputeMarkerEdges(
    vector<unordered_map<int, ScalarType>>* edges) {
  sort(markers_.begin(), markers_.end(), Marker::comp);
  size_t tot_marker_num = markers_.size();

  if (opts_.use_cc) {
    cc_idx_.clear();
    cc_idx_.reserve(tot_marker_num);
    cc_marker_lut_.clear();
    for (size_t i = 0; i < tot_marker_num; ++i) {
      cc_idx_.push_back(markers_[i].cc_id);
      if (cc_marker_lut_.find(markers_[i].cc_id) == cc_marker_lut_.end()) {
        cc_marker_lut_[markers_[i].cc_id] = vector<int>();
      }
      cc_marker_lut_[markers_[i].cc_id].push_back(i);
    }
  }

  // connection status: (-2): to be determined
  //                    (-1): ended
  //                    (id >= 0): determined
  vector<int> connect_status(tot_marker_num, -2);

  edges->clear();  // candidate edges for each marker
  edges->resize(tot_marker_num);
  int tot_num_edges = 0;
  for (size_t i = 0; i < tot_marker_num; ++i) {
    edges->at(i).clear();

    ADEBUG << "marker " << i << " cc " << markers_[i].cc_id << " ("
           << markers_[i].cc_edge_descend_id << ")"
           << " y=" << std::to_string(markers_[i].pos(1));  // y

    bool early_stop_flag = false;
    if (opts_.use_cc) {
      for (int j : cc_marker_lut_[markers_[i].cc_id]) {
        if (markers_[j].cc_edge_ascend_id ==
            markers_[i].cc_edge_ascend_id + 1) {
          (*edges)[i][j] = static_cast<ScalarType>(0);
          connect_status[i] = j;
          ADEBUG << "  dist " << j << " = " << 0.0 << " cc "
                 << markers_[i].cc_id << " ( << "
                 << markers_[j].cc_edge_descend_id << ")";
          break;
        }
      }
      if (markers_[i].cc_edge_descend_id >= opts_.max_cc_marker_match_num) {
        early_stop_flag = true;
      }
    }

    if (early_stop_flag) {
      ADEBUG << " early stopped.";
      continue;
    }

    ScalarType y_thresh = 0;
    switch (opts_.space_type) {
      case SpaceType::IMAGE: {
        y_thresh = markers_[i].pos(1) + opts_.min_y_search_offset;
        break;
      }
      case SpaceType::VEHICLE: {
        y_thresh = markers_[i].pos(0) - opts_.min_y_search_offset;
        break;
      }
      default: { AERROR << "Error: unknown space type " << opts_.space_type; }
    }

    for (size_t j = 0; j < tot_marker_num; ++j) {
      // ignore the markers below marker i or itself
      if (j == i) {
        continue;
      }

      if ((opts_.space_type == SpaceType::IMAGE &&
           markers_[j].start_pos(1) > y_thresh) ||
          (opts_.space_type == SpaceType::VEHICLE &&
           markers_[j].start_pos(0) < y_thresh)) {
        continue;
      }

      // ignore the markers in the same CC or the ones rather than
      // the beginning markers
      if (opts_.use_cc &&
          (markers_[j].cc_id == markers_[i].cc_id ||
           markers_[j].cc_edge_ascend_id >= opts_.max_cc_marker_match_num)) {
        continue;
      }

      ADEBUG << "marker i: y=" << std::to_string(markers_[i].pos(1)) << ", "
             << "marker j: y=" << std::to_string(markers_[j].start_pos(1));

      ScalarType dist = ComputeMarkerPairDistance(markers_[i], markers_[j]);
      ADEBUG << " dist " << j << " = " << std::to_string(dist) << " cc "
             << markers_[j].cc_id << " ( << " << markers_[j].cc_edge_ascend_id
             << ")";

      if (dist >= 0) {
        (*edges)[i][j] = dist;
      }
    }

    if ((*edges)[i].empty()) {
      connect_status[i] = -1;
    }

    tot_num_edges += static_cast<int>(edges->at(i).size());
  }

  return connect_status;
}

// greedy select connections based on marker groups
bool LaneFrame::GreedyGroupConnectAssociation() {
  if (!opts_.use_cc) {
    AERROR << "requires to use CC option.";
    return false;
  }
  if (opts_.group_param.max_group_prediction_marker_num >
      MAX_GROUP_PREDICTION_MARKER_NUM) {
    AERROR << "max_group_prediction_marker_num is larger than "
           << MAX_GROUP_PREDICTION_MARKER_NUM;
    return false;
  }

  if (opts_.orientation_estimation_skip_marker_num >=
      opts_.group_param.max_group_prediction_marker_num) {
    AERROR << "orientation_estimation_skip_marker_num ("
           << opts_.orientation_estimation_skip_marker_num
           << ") should be smaller than max_group_prediction_marker_num ("
           << opts_.group_param.max_group_prediction_marker_num << ")";
    return false;
  }
  opts_.group_param.orientation_estimation_skip_marker_num =
      opts_.orientation_estimation_skip_marker_num;

  ADEBUG << "max_group_prediction_marker_num = "
        << opts_.group_param.max_group_prediction_marker_num;
  ADEBUG << "orientation_estimation_skip_marker_num = "
        << opts_.group_param.orientation_estimation_skip_marker_num;

  // generate marker groups based on CC heuristic
  vector<Group> groups;
  groups.reserve(max_cc_num_);
  unordered_map<int, int> hash_cc_idx(max_cc_num_);
  for (size_t i = 0; i < markers_.size(); ++i) {
    if (hash_cc_idx.find(markers_[i].cc_id) == hash_cc_idx.end()) {
      hash_cc_idx[markers_[i].cc_id] = static_cast<int>(groups.size());
      groups.push_back(Group());
      groups.back().shape_type = markers_[i].shape_type;
      groups.back().space_type = markers_[i].space_type;
      if (groups.back().space_type != opts_.space_type) {
        AERROR << "space type does not match.";
      }

      groups.back().start_marker_idx.resize(
          opts_.group_param.max_group_prediction_marker_num, -1);
      groups.back().end_marker_idx.resize(
          opts_.group_param.max_group_prediction_marker_num, -1);
    }

    int j = hash_cc_idx[markers_[i].cc_id];
    groups[j].marker_idx.push_back(i);

    if (markers_[i].cc_edge_ascend_id <
        opts_.group_param.max_group_prediction_marker_num) {
      groups[j].start_marker_idx[markers_[i].cc_edge_ascend_id] = i;
    }
    if (markers_[i].cc_edge_descend_id <
        opts_.group_param.max_group_prediction_marker_num) {
      groups[j].end_marker_idx[markers_[i].cc_edge_descend_id] = i;
    }
  }
  ADEBUG << "number of marker groups = " << groups.size();

  // compute the orientation of starting and end points for each group
  for (auto it_group = groups.begin(); it_group != groups.end(); ++it_group) {
    int i_start = it_group->start_marker_idx[0];
    if (!IsValidMarker(i_start)) {
      AERROR << "marker " << i_start << " is not valid.";
      return false;
    }
    it_group->start_pos =
        (markers_[i_start].shape_type == MarkerShapeType::POINT)
            ? markers_[i_start].pos
            : markers_[i_start].start_pos;

    int i_end = it_group->end_marker_idx[0];
    if (!IsValidMarker(i_end)) {
      AERROR << "marker " << i_end << " is not valid.";
      return false;
    }
    it_group->end_pos = markers_[i_end].pos;

    int k = it_group->ComputeOrientation(markers_, opts_.group_param);
    it_group->start_marker_idx.resize(k);
    it_group->end_marker_idx.resize(k);
  }

  // compute distance matrix of marker groups
  int n = static_cast<int>(groups.size());
  Eigen::Matrix<ScalarType, Eigen::Dynamic, Eigen::Dynamic> dist_mat =
      Eigen::Matrix<ScalarType, Eigen::Dynamic, Eigen::Dynamic>::Ones(n, n) *
      static_cast<ScalarType>(-1);
  Eigen::Matrix<int, Eigen::Dynamic, Eigen::Dynamic> connect_mat =
      Eigen::Matrix<int, Eigen::Dynamic, Eigen::Dynamic>::Zero(n, n);
  vector<pair<ScalarType, int>> edges;  // (linear id, distance value)
  edges.reserve(size_t(n) * size_t(n));
  vector<unordered_set<int>> to_group_idx(n);
  vector<unordered_set<int>> from_group_idx(n);

  for (int k = 0; k < n; ++k) {
    const Group* cur_group = &(groups[k]);

    ScalarType y_thresh = 0;
    switch (cur_group->space_type) {
      case SpaceType::IMAGE: {
        y_thresh = cur_group->end_pos(1) + opts_.min_y_search_offset;
        break;
      }
      case SpaceType::VEHICLE: {
        y_thresh = cur_group->end_pos(0) - opts_.min_y_search_offset;
        break;
      }
      default: { AERROR << "unknown space type " << cur_group->space_type; }
    }

    to_group_idx[k].reserve(n);
    from_group_idx[k].reserve(n);
    for (int k1 = 0; k1 < n; ++k1) {
      const Group* tar_group = &(groups[k1]);

      if (k1 == k) {
        continue;
      }

      if ((tar_group->space_type == SpaceType::IMAGE &&
           tar_group->start_pos(1) > y_thresh) ||
          (tar_group->space_type == SpaceType::VEHICLE &&
           tar_group->start_pos(0) < y_thresh)) {
        continue;
      }

      dist_mat(k, k1) =
          cur_group->ComputeDistance(*tar_group, opts_.assoc_param);
      if (dist_mat(k, k1) >= 0) {
        connect_mat(k, k1) = 1;  // 1: available to be connected
        edges.push_back(make_pair(dist_mat(k, k1), k * n + k1));
        to_group_idx[k].insert(k1);
        from_group_idx[k1].insert(k);
      }
    }
  }

  // greedy select connections for group association
  sort(edges.begin(), edges.end());
  for (auto it_edge = edges.begin(); it_edge != edges.end(); ++it_edge) {
    // select the lowest-cost one from candidate edges
    int src_group_id = it_edge->second / n;
    int tar_group_id = it_edge->second % n;
    if (std::abs(dist_mat(src_group_id, tar_group_id) - it_edge->first) >=
        kEpsilon) {
      AERROR << "the distance measure from group " << src_group_id
             << " to group " << tar_group_id << " is not equal to "
             << it_edge->first;
      return false;
    }

    if (connect_mat(src_group_id, tar_group_id) == 0) {
      continue;
    }

    // connect source group with target group and
    // delete the candidate edges related to the selected one
    for (int i = 0; i < n; ++i) {
      connect_mat(src_group_id, i) = 0;
      connect_mat(i, tar_group_id) = 0;
    }
    connect_mat(src_group_id, tar_group_id) = 1;

    to_group_idx[src_group_id].clear();
    to_group_idx[src_group_id].insert(tar_group_id);
    from_group_idx[tar_group_id].clear();
    from_group_idx[tar_group_id].insert(src_group_id);
  }

  // generate lane clusters
  vector<int> lut_to_id(n, -1);
  vector<int> lut_from_id(n, -1);
  for (int i = 0; i < n; ++i) {
    for (int j = 0; j < n; ++j) {
      if (connect_mat(i, j) == 1) {
        if (lut_to_id[i] != -1) {
          AERROR << "lut_to_id of group " << i << " is not equal to " << -1;
          return false;
        }
        lut_to_id[i] = j;

        if (lut_from_id[j] != -1) {
          AERROR << "lut_from_id of group " << j << " is not equal to " << -1;
          return false;
        }
        lut_from_id[j] = i;

        break;
      }
    }
  }

  unordered_set<int> hash_group_idx(n);
  unordered_set<int> hash_marker_idx(markers_.size());
  int cur_group_id = -1;
  for (int k = 0; k < n; ++k) {
    if (lut_from_id[k] == -1) {
      // generate a new cluster
      graphs_.push_back(Graph());
      cur_group_id = k;

      while (cur_group_id != -1) {
        hash_group_idx.insert(cur_group_id);

        int n_markers_group =
            static_cast<int>(groups[cur_group_id].marker_idx.size());
        if (lut_to_id[cur_group_id] == -1) {
          if (n_markers_group >=
              opts_.orientation_estimation_skip_marker_num + 2) {
            AddGroupIntoGraph(groups[cur_group_id],
                              opts_.orientation_estimation_skip_marker_num, 0,
                              &(graphs_.back()), &hash_marker_idx);
          } else {
            AddGroupIntoGraph(groups[cur_group_id], &(graphs_.back()),
                              &hash_marker_idx);
          }
        } else {
          if (n_markers_group >=
              opts_.orientation_estimation_skip_marker_num * 2 + 2) {
            AddGroupIntoGraph(groups[cur_group_id],
                              opts_.orientation_estimation_skip_marker_num,
                              opts_.orientation_estimation_skip_marker_num,
                              &(graphs_.back()), &hash_marker_idx);
          } else {
            AddGroupIntoGraph(groups[cur_group_id], &(graphs_.back()),
                              &hash_marker_idx);
          }
        }
        cur_group_id = lut_to_id[cur_group_id];
      }
    }
  }

  if (static_cast<int>(hash_group_idx.size()) != n) {
    AERROR << "total number of groups is not equal to " << n;
    return false;
  }
  if (hash_marker_idx.size() > markers_.size()) {
    AERROR << "total number of markers is larger than " << markers_.size();
    return false;
  }

  return true;
}

int LaneFrame::AddGroupIntoGraph(const Group& group, Graph* graph,
                                 unordered_set<int>* hash_marker_idx) {
  int count_markers = 0;

  int cur_marker_id = group.start_marker_idx[0];
  if (!graph->empty() && graph->back().second == -1) {
    if (cur_marker_id != -1) {
      graph->back().second = cur_marker_id;
    }
  }

  while (cur_marker_id != -1) {
    CHECK(IsValidMarker(cur_marker_id));
    CHECK(hash_marker_idx->find(cur_marker_id) == hash_marker_idx->end());

    int next_marker_id = markers_[cur_marker_id].cc_next_marker_id;
    graph->push_back(make_pair(cur_marker_id, next_marker_id));
    hash_marker_idx->insert(cur_marker_id);

    cur_marker_id = next_marker_id;
    ++count_markers;
  }

  return count_markers;
}

int LaneFrame::AddGroupIntoGraph(const Group& group,
                                 const int& start_marker_ascend_id,
                                 const int& end_marker_descend_id, Graph* graph,
                                 unordered_set<int>* hash_marker_idx) {
  int count_markers = 0;

  int cur_marker_id = group.start_marker_idx.at(start_marker_ascend_id);
  if (!graph->empty() && graph->back().second == -1) {
    if (cur_marker_id != -1) {
      graph->back().second = cur_marker_id;
    }
  }

  while (cur_marker_id != -1 &&
         markers_[cur_marker_id].cc_edge_descend_id >= end_marker_descend_id) {
    CHECK(IsValidMarker(cur_marker_id));
    CHECK(hash_marker_idx->find(cur_marker_id) == hash_marker_idx->end());

    int next_marker_id = markers_[cur_marker_id].cc_next_marker_id;
    graph->push_back(make_pair(cur_marker_id, next_marker_id));
    hash_marker_idx->insert(cur_marker_id);

    cur_marker_id = next_marker_id;
    ++count_markers;
  }
  if (!graph->empty()) {
    graph->back().second = -1;  // set the last node available to be connected
  }

  return count_markers;
}

bool LaneFrame::FitPolyCurve(const int& graph_id, const ScalarType& graph_siz,
                             PolyModel* poly_coef,
                             ScalarType* lateral_distance) const {
  if (poly_coef == nullptr) {
    AERROR << "poly_coef is a null pointer.";
    return false;
  }
  if (lateral_distance == nullptr) {
    AERROR << "lateral_distance is a null pointer.";
    return false;
  }

  if (graph_id < 0 || graph_id >= static_cast<int>(graphs_.size())) {
    AERROR << "graph " << graph_id << "is out range of [0, " << graphs_.size()
           << ").";
    return false;
  }

  const Graph& graph = graphs_[graph_id];

  vector<Vector2D> pos_data;
  pos_data.reserve(graph.size() * 2);
  for (auto it = graph.begin(); it != graph.end(); ++it) {
    int i = it->first;
    if (i < 0 || i >= static_cast<int>(markers_.size())) {
      AERROR << "marker " << i << "is out range of [0, " << markers_.size()
             << ").";
      return false;
    }

    if (markers_[i].shape_type != MarkerShapeType::POINT) {
      pos_data.push_back(markers_[i].start_pos);
    }
    pos_data.push_back(markers_[i].pos);
  }
  pos_data.shrink_to_fit();

  if (pos_data.size() == 1) {
    AERROR << "only one point in graph " << graph_id;
    return false;
  }

  int order = 0;
  if (pos_data.size() < 3 || graph_siz < opts_.max_size_to_fit_straight_line) {
    order = 1;  // fit a 1st-order polynomial curve (straight line)
  } else {
    order = 2;  // fit a 2nd-order polynomial curve
  }

  if (!PolyFit(pos_data, order, poly_coef)) {
    AERROR << "failed to fit " << order << " order polynomial curve.";
    return false;
  }

  *lateral_distance = (*poly_coef)(0);

  ADEBUG << "succeed to fit a " << order << " order polynomial curve: "
         << "lateral distance = " << *lateral_distance;
  return true;
}

bool LaneFrame::Process(LaneInstancesPtr instances) {
  if (instances == NULL) {
    AERROR << "the pointer of input instances is null.";
    return false;
  }

  // do marker association
  switch (opts_.assoc_param.method) {
    case AssociationMethod::GREEDY_GROUP_CONNECT: {
      ADEBUG << "using greedy group connection algorithm "
            << "for marker association ...";
      if (!GreedyGroupConnectAssociation()) {
        AERROR << "failed to do marker association.";
        return false;
      }
      break;
    }
    default: { AERROR << "unknown marker association method."; }
  }
  ADEBUG << "number of lane instance candidates = " << graphs_.size();

  // compute tight bounding box for graphs
  ComputeBbox();

  // generate lane instances
  instances->clear();
  instances->reserve(graphs_.size());
  for (int k = 0; k < GraphNum(); ++k) {
    ScalarType siz = std::max(boxes_.at(k)(2) - boxes_.at(k)(0),
                              boxes_.at(k)(3) - boxes_.at(k)(1));
    // remove too small graphs
    if (siz >= opts_.min_instance_size_prefiltered) {
      instances->push_back(LaneInstance(k, siz, boxes_.at(k)));
    } else {
      ADEBUG << "size of graph " << k << " is too small: " << siz << " ("
             << opts_.min_instance_size_prefiltered << "), "
             << "removed.";
    }
  }

  instances->shrink_to_fit();
  return true;
}

}  // namespace perception
}  // namespace apollo
