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

#include "modules/prediction/container/obstacles/obstacle_clusters.h"

#include <queue>

#include "modules/prediction/common/road_graph.h"
#include "modules/prediction/common/prediction_map.h"

namespace apollo {
namespace prediction {

using ::apollo::hdmap::LaneInfo;

std::unordered_map<std::string, LaneGraph> ObstacleClusters::lane_graphs_;
std::unordered_map<std::string, JunctionFeature>
    ObstacleClusters::junction_features_;

void ObstacleClusters::Clear() { lane_graphs_.clear(); }

void ObstacleClusters::Init() { Clear(); }

const LaneGraph& ObstacleClusters::GetLaneGraph(
    const double start_s, const double length,
    std::shared_ptr<const LaneInfo> lane_info_ptr) {
  std::string lane_id = lane_info_ptr->id().id();
  if (lane_graphs_.find(lane_id) != lane_graphs_.end()) {
    LaneGraph* lane_graph = &lane_graphs_[lane_id];
    for (int i = 0; i < lane_graph->lane_sequence_size(); ++i) {
      LaneSequence* lane_seq_ptr = lane_graph->mutable_lane_sequence(i);
      if (lane_seq_ptr->lane_segment_size() == 0) {
        continue;
      }
      LaneSegment* first_lane_seg_ptr = lane_seq_ptr->mutable_lane_segment(0);
      if (first_lane_seg_ptr->lane_id() != lane_id) {
        continue;
      }
      first_lane_seg_ptr->set_start_s(start_s);
    }
    return lane_graphs_[lane_id];
  }
  RoadGraph road_graph(start_s, length, lane_info_ptr);
  LaneGraph lane_graph;
  road_graph.BuildLaneGraph(&lane_graph);
  lane_graphs_[lane_id] = std::move(lane_graph);
  return lane_graphs_[lane_id];
}

const JunctionFeature& ObstacleClusters::GetJunctionFeature(
    const std::string& start_lane_id, const std::string& junction_id) {
  if (junction_features_.find(start_lane_id) != junction_features_.end()) {
    return junction_features_[start_lane_id];
  }
  JunctionFeature junction_feature;
  junction_feature.set_junction_id(junction_id);
  junction_feature.mutable_enter_lane()->set_lane_id(start_lane_id);
  std::queue<std::shared_ptr<const LaneInfo>> lane_queue;
  lane_queue.push(PredictionMap::LaneById(start_lane_id));
  while (!lane_queue.empty()) {
    std::shared_ptr<const LaneInfo> curr_lane = lane_queue.front();
    lane_queue.pop();
    for (auto& succ_lane_id : curr_lane->lane().successor_id()) {
      std::shared_ptr<const LaneInfo> succ_lane =
          PredictionMap::LaneById(succ_lane_id.id());
      if (PredictionMap::IsLaneInJunction(succ_lane, junction_id)) {
        lane_queue.push(succ_lane);
      } else {
        JunctionExit junction_exit = BuildJunctionExit(succ_lane);
        junction_feature.add_junction_exit()->CopyFrom(junction_exit);
      }
    }
  }

  junction_features_[start_lane_id] = std::move(junction_feature);
  return junction_features_[start_lane_id];
}

JunctionExit ObstacleClusters::BuildJunctionExit(
    const std::shared_ptr<const LaneInfo> exit_lane) {
  JunctionExit junction_exit;
  junction_exit.set_exit_lane_id(exit_lane->id().id());
  // TODO(kechxu) set exit position and heading
  double s = 0.5;  // TODO(all) think about this value
  double exit_heading = exit_lane->Heading(s);
  apollo::common::PointENU position = exit_lane->GetSmoothPoint(s);
  junction_exit.set_exit_heading(exit_heading);
  junction_exit.mutable_exit_position()->set_x(position.x());
  junction_exit.mutable_exit_position()->set_y(position.y());
  return junction_exit;
}

}  // namespace prediction
}  // namespace apollo
