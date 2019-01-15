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

namespace apollo {
namespace prediction {

using apollo::common::ErrorCode;
using apollo::common::Status;
using apollo::hdmap::Lane;
using apollo::hdmap::LaneInfo;
using apollo::common::math::NormalizeAngle;


// Custom helper functions for sorting purpose.
bool HeadingIsAtLeft(std::vector<double> heading1,
                     std::vector<double> heading2,
                     size_t idx);
bool IsAtLeft(std::shared_ptr<const LaneInfo> lane1,
              std::shared_ptr<const LaneInfo> lane2);
int ConvertTurnTypeToDegree(std::shared_ptr<const LaneInfo> lane);


bool HeadingIsAtLeft(std::vector<double> heading1,
                     std::vector<double> heading2,
                     size_t idx) {
  if (heading1.empty() || heading2.empty()) {
    return true;
  }
  if (idx >= heading1.size() || idx >= heading2.size()) {
    return true;
  }
  if (NormalizeAngle(heading1[idx] - heading2[idx]) > 0.0) {
    return true;
  } else if (NormalizeAngle(heading1[idx] - heading2[idx]) < 0.0) {
    return false;
  } else {
    return HeadingIsAtLeft(heading1, heading2, idx + 1);
  }
}
bool IsAtLeft(std::shared_ptr<const LaneInfo> lane1,
              std::shared_ptr<const LaneInfo> lane2) {
  if (lane1->lane().has_turn() && lane2->lane().has_turn() &&
      lane1->lane().turn() != lane2->lane().turn()) {
    int degree_to_left_1 = ConvertTurnTypeToDegree(lane1);
    int degree_to_left_2 = ConvertTurnTypeToDegree(lane2);
    return (degree_to_left_1 > degree_to_left_2);
  } else {
    auto heading1 = lane1->headings();
    auto heading2 = lane2->headings();
    return HeadingIsAtLeft(heading1, heading2, 0);
  }
}
int ConvertTurnTypeToDegree(std::shared_ptr<const LaneInfo> lane) {
  // Sanity checks.
  if (!lane->lane().has_turn()) {
    return 0;
  }

  // Assign a number to measure how much it is bent to the left.
  if (lane->lane().turn() == Lane::NO_TURN) {
    return 0;
  } else if (lane->lane().turn() == Lane::LEFT_TURN) {
    return 1;
  } else if (lane->lane().turn() == Lane::U_TURN) {
    return 2;
  } else {
    return -1;
  }
}

RoadGraph::RoadGraph(const double start_s, const double length,
                     std::shared_ptr<const LaneInfo> lane_info_ptr)
    : start_s_(start_s), length_(length), lane_info_ptr_(lane_info_ptr) {}

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
                      FLAGS_road_graph_max_search_horizon,
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
    const int graph_search_horizon,
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
      successor_lanes.push_back
          (PredictionMap::LaneById(successor_lane_id.id()));
    }
    std::sort(successor_lanes.begin(), successor_lanes.end(), IsAtLeft);

    // Run recursion function to perform DFS.
    for (size_t i = 0; i < successor_lanes.size(); i++) {
      ComputeLaneSequence(successor_accumulated_s, 0.0, successor_lanes[i],
                          graph_search_horizon - 1,
                          lane_segments, lane_graph_ptr);
    }
  }
  lane_segments->pop_back();
}

}  // namespace prediction
}  // namespace apollo
