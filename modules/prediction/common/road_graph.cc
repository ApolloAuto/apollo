/******************************************************************************
 * Copyright 2017 The Apollo Authors. All Rights Reserved.
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

#include "modules/prediction/common/road_graph.h"

#include <algorithm>
#include <utility>

#include "modules/prediction/common/prediction_gflags.h"
#include "modules/prediction/common/prediction_system_gflags.h"

namespace apollo {
namespace prediction {
namespace {

using apollo::common::ErrorCode;
using apollo::common::Status;
using apollo::hdmap::Lane;
using apollo::hdmap::LaneInfo;

// Custom helper functions for sorting purpose.
bool HeadingIsAtLeft(const std::vector<double>& heading1,
                     const std::vector<double>& heading2, const size_t idx) {
  if (idx >= heading1.size() || idx >= heading2.size()) {
    return true;
  }
  const double angle =
      apollo::common::math::NormalizeAngle(heading1[idx] - heading2[idx]);
  if (angle > 0.0) {
    return true;
  } else if (angle < 0.0) {
    return false;
  }
  return HeadingIsAtLeft(heading1, heading2, idx + 1);
}

int ConvertTurnTypeToDegree(const Lane& lane) {
  // Sanity checks.
  if (!lane.has_turn()) {
    return 0;
  }

  // Assign a number to measure how much it is bent to the left.
  switch (lane.turn()) {
    case Lane::NO_TURN:
      return 0;
    case Lane::LEFT_TURN:
      return 1;
    case Lane::U_TURN:
      return 2;
    default:
      break;
  }

  return -1;
}

bool IsAtLeft(std::shared_ptr<const LaneInfo> lane1,
              std::shared_ptr<const LaneInfo> lane2) {
  if (lane1->lane().has_turn() && lane2->lane().has_turn() &&
      lane1->lane().turn() != lane2->lane().turn()) {
    int degree_to_left_1 = ConvertTurnTypeToDegree(lane1->lane());
    int degree_to_left_2 = ConvertTurnTypeToDegree(lane2->lane());
    return degree_to_left_1 > degree_to_left_2;
  }
  return HeadingIsAtLeft(lane1->headings(), lane2->headings(), 0);
}

}  // namespace

RoadGraph::RoadGraph(const double start_s, const double length,
                     const bool consider_divide,
                     std::shared_ptr<const LaneInfo> lane_info_ptr)
    : start_s_(start_s),
      length_(length),
      consider_divide_(consider_divide),
      lane_info_ptr_(lane_info_ptr) {}

Status RoadGraph::BuildLaneGraph(LaneGraph* const lane_graph_ptr) {
  // Sanity checks.
  if (length_ < 0.0 || lane_info_ptr_ == nullptr) {
    const auto error_msg = common::util::StrCat(
        "Invalid road graph settings. Road graph length = ", length_);
    AERROR << error_msg;
    return Status(ErrorCode::PREDICTION_ERROR, error_msg);
  }
  if (lane_graph_ptr == nullptr) {
    const auto error_msg = "Invalid input lane graph.";
    AERROR << error_msg;
    return Status(ErrorCode::PREDICTION_ERROR, error_msg);
  }

  // Run the recursive function to perform DFS.
  std::vector<LaneSegment> lane_segments;
  double accumulated_s = 0.0;
  ComputeLaneSequence(accumulated_s, start_s_, lane_info_ptr_,
                      FLAGS_road_graph_max_search_horizon, consider_divide_,
                      &lane_segments, lane_graph_ptr);

  return Status::OK();
}

bool RoadGraph::IsOnLaneGraph(std::shared_ptr<const LaneInfo> lane_info_ptr,
                              const LaneGraph& lane_graph) {
  if (!lane_graph.IsInitialized()) {
    return false;
  }

  for (const auto& lane_sequence : lane_graph.lane_sequence()) {
    for (const auto& lane_segment : lane_sequence.lane_segment()) {
      if (lane_segment.has_lane_id() &&
          lane_segment.lane_id() == lane_info_ptr->id().id()) {
        return true;
      }
    }
  }

  return false;
}

void RoadGraph::ComputeLaneSequence(
    const double accumulated_s, const double start_s,
    std::shared_ptr<const LaneInfo> lane_info_ptr,
    const int graph_search_horizon, const bool consider_divide,
    std::vector<LaneSegment>* const lane_segments,
    LaneGraph* const lane_graph_ptr) const {
  // Sanity checks.
  if (lane_info_ptr == nullptr) {
    AERROR << "Invalid lane.";
    return;
  }
  if (graph_search_horizon < 0) {
    AERROR << "The lane search has already reached the limits";
    AERROR << "Possible map error found!";
    return;
  }

  // Create a new lane_segment based on the current lane_info_ptr.
  LaneSegment lane_segment;
  lane_segment.set_lane_id(lane_info_ptr->id().id());
  lane_segment.set_start_s(start_s);
  lane_segment.set_lane_turn_type(
      PredictionMap::LaneTurnType(lane_info_ptr->id().id()));
  if (accumulated_s + lane_info_ptr->total_length() - start_s >= length_) {
    lane_segment.set_end_s(length_ - accumulated_s + start_s);
  } else {
    lane_segment.set_end_s(lane_info_ptr->total_length());
  }
  lane_segment.set_total_length(lane_info_ptr->total_length());
  lane_segments->push_back(std::move(lane_segment));

  if (accumulated_s + lane_info_ptr->total_length() - start_s >= length_ ||
      lane_info_ptr->lane().successor_id_size() == 0) {
    // End condition: if search reached the max. search distance,
    //             or if there is no more successor lane_segment.
    LaneSequence* sequence = lane_graph_ptr->add_lane_sequence();
    *sequence->mutable_lane_segment() = {lane_segments->begin(),
                                         lane_segments->end()};
    sequence->set_label(0);
  } else {
    const double successor_accumulated_s =
        accumulated_s + lane_info_ptr->total_length() - start_s;

    // Sort the successor lane_segments from left to right.
    std::vector<std::shared_ptr<const hdmap::LaneInfo>> successor_lanes;
    for (const auto& successor_lane_id : lane_info_ptr->lane().successor_id()) {
      successor_lanes.push_back(
          PredictionMap::LaneById(successor_lane_id.id()));
    }
    std::sort(successor_lanes.begin(), successor_lanes.end(), IsAtLeft);

    if (!successor_lanes.empty()) {
      if (consider_divide) {
        if (successor_lanes.size() > 1) {
          // Run recursion function to perform DFS.
          for (const auto& successor_lane : successor_lanes) {
            bool consider_divide_further = false;
            if (FLAGS_prediction_offline_mode == 1 ||
                FLAGS_prediction_offline_mode == 2) {
              consider_divide_further = true;
            }
            ComputeLaneSequence(successor_accumulated_s, 0.0,
                                successor_lane, graph_search_horizon - 1,
                                consider_divide_further, lane_segments,
                                lane_graph_ptr);
          }
        } else {
          ComputeLaneSequence(successor_accumulated_s, 0.0, successor_lanes[0],
                              graph_search_horizon - 1, true, lane_segments,
                              lane_graph_ptr);
        }
      } else {
        auto selected_successor_lane =
            LaneWithSmallestAverageCurvature(successor_lanes);
        ComputeLaneSequence(successor_accumulated_s, 0.0,
                            selected_successor_lane, graph_search_horizon - 1,
                            false, lane_segments, lane_graph_ptr);
      }
    }
  }
  lane_segments->pop_back();
}

std::shared_ptr<const hdmap::LaneInfo>
RoadGraph::LaneWithSmallestAverageCurvature(
    const std::vector<std::shared_ptr<const hdmap::LaneInfo>>& lane_infos)
    const {
  CHECK(!lane_infos.empty());
  size_t sample_size = FLAGS_sample_size_for_average_lane_curvature;
  std::shared_ptr<const hdmap::LaneInfo> selected_lane_info = lane_infos[0];
  double smallest_curvature =
      AverageCurvature(selected_lane_info->id().id(), sample_size);
  for (size_t i = 1; i < lane_infos.size(); ++i) {
    std::shared_ptr<const hdmap::LaneInfo> lane_info = lane_infos[i];
    double curvature = AverageCurvature(lane_info->id().id(), sample_size);
    if (curvature < smallest_curvature) {
      smallest_curvature = curvature;
      selected_lane_info = lane_info;
    }
  }
  return selected_lane_info;
}

double RoadGraph::AverageCurvature(const std::string& lane_id,
                                   const size_t sample_size) const {
  CHECK_GT(sample_size, 0);
  std::shared_ptr<const hdmap::LaneInfo> lane_info_ptr =
      PredictionMap::LaneById(lane_id);
  if (lane_info_ptr == nullptr) {
    return 0.0;
  }
  double lane_length = lane_info_ptr->total_length();
  double s_gap = lane_length / static_cast<double>(sample_size);
  double curvature_sum = 0.0;
  for (size_t i = 0; i < sample_size; ++i) {
    double s = s_gap * static_cast<double>(i);
    curvature_sum += std::abs(PredictionMap::CurvatureOnLane(lane_id, s));
  }
  return curvature_sum / static_cast<double>(sample_size);
}

}  // namespace prediction
}  // namespace apollo
