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

#include <utility>

#include "modules/prediction/common/road_graph.h"
#include "modules/prediction/common/prediction_map.h"

namespace apollo {
namespace prediction {

using apollo::hdmap::LaneInfo;
using apollo::hdmap::Id;
using apollo::common::ErrorCode;

RoadGraph::RoadGraph()
    : start_s_(0.0), length_(-1.0), lane_info_ptr_(nullptr) {
}

RoadGraph::RoadGraph(double start_s, double length,
                     const apollo::hdmap::LaneInfo* lane_info_ptr)
    : start_s_(start_s), length_(length), lane_info_ptr_(lane_info_ptr) {
}

RoadGraph::~RoadGraph() {
  start_s_ = 0.0;
  length_ = -1.0;
  lane_info_ptr_ = nullptr;
}

void RoadGraph::Set(double start_s, double length,
                    apollo::hdmap::LaneInfo* lane_info_ptr) {
  start_s_ = start_s;
  length_ = length;
  lane_info_ptr_ = lane_info_ptr;
}

ErrorCode RoadGraph::BuildLaneGraph(LaneGraph* lane_graph_ptr) {
  if (length_ < 0.0 || lane_info_ptr_ == nullptr) {
    AERROR << "Invalid road graph settings. Road graph length = " << length_;
    return ErrorCode::PREDICTION_ERROR;
  }

  if (lane_graph_ptr == nullptr) {
    AERROR << "Invalid input lane graph.";
    return ErrorCode::PREDICTION_ERROR;
  }

  std::vector<LaneSegment> lane_segments;
  double accumulated_s = 0.0;
  ComputeLaneSequence(accumulated_s, start_s_, lane_info_ptr_,
                      &lane_segments, lane_graph_ptr);

  return ErrorCode::OK;
}

void RoadGraph::ComputeLaneSequence(
    double accumulated_s, double start_s,
    const apollo::hdmap::LaneInfo* lane_info_ptr,
    std::vector<LaneSegment>* lane_segments,
    LaneGraph* lane_graph_ptr) const {
  if (lane_info_ptr == nullptr) {
    AERROR << "Invalid lane.";
    return;
  }
  PredictionMap* map = PredictionMap::instance();
  if (map == nullptr) {
    AERROR << "Missing map.";
    return;
  }
  LaneSegment lane_segment;
  lane_segment.set_lane_id(lane_info_ptr->id().id());
  lane_segment.set_start_s(start_s);
  // lane_segment.set_lane_turn_type(map->lane_turn_type(lane_info_ptr->id()));
  if (accumulated_s + lane_info_ptr->total_length() - start_s >= length_) {
    lane_segment.set_end_s(length_ - accumulated_s + start_s);
  } else {
    lane_segment.set_end_s(lane_info_ptr->total_length());
  }

  lane_segments->push_back(std::move(lane_segment));

  if (accumulated_s + lane_info_ptr->total_length() - start_s >= length_ ||
    lane_info_ptr->lane().successor_id_size() == 0) {
    LaneSequence* sequence = lane_graph_ptr->add_lane_sequence();
    for (auto& lane_segment : *lane_segments) {
      sequence->add_lane_segment()->CopyFrom(lane_segment);
    }
    sequence->set_label(0);
  } else {
    for (const auto& successor_lane_id : lane_info_ptr->lane().successor_id()) {
      double successor_accumulated_s =
          accumulated_s + lane_info_ptr->total_length() - start_s;
      const LaneInfo* successor_lane = map->LaneById(successor_lane_id);
      ComputeLaneSequence(successor_accumulated_s, 0.0, successor_lane,
          lane_segments, lane_graph_ptr);
    }
  }
  lane_segments->pop_back();
}

}  // namespace prediction
}  // namespace apollo
