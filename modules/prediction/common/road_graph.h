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

#pragma once

#include <list>
#include <memory>
#include <string>
#include <vector>

#include "modules/common/status/status.h"
#include "modules/prediction/common/prediction_map.h"
#include "modules/prediction/proto/lane_graph.pb.h"

namespace apollo {
namespace prediction {

class RoadGraph {
 public:
  /**
   * @brief Constructor
   * @param start_s The starting longitudinal s value.
   * @param length The length to build the road graph.
   * @param If consider all successor lanes after dividing.
   * @param lane_info_ptr The starting lane.
   */
  RoadGraph(const double start_s, const double length,
            const bool consider_divide,
            std::shared_ptr<const hdmap::LaneInfo> lane_info_ptr);

  /**
   * @brief Build the lane graph.
   * @param The built lane graph.
   * @return The status of the road graph building.
   */
  common::Status BuildLaneGraph(LaneGraph* const lane_graph);

  common::Status BuildLaneGraphBidirection(LaneGraph* const lane_graph_ptr);

  /**
   * @brief Check if a lane with an s is on the lane graph
   * @param Lane ID
   * @param Lane s
   * @param The pointer to the given lane graph
   * @return If the given lane ID and lane s is on the lane graph
   */
  bool IsOnLaneGraph(std::shared_ptr<const hdmap::LaneInfo> lane_info_ptr,
                     const LaneGraph& lane_graph);

 private:
  /** @brief Combine the lane-graph of forward direction and that of backward
   *        direction together.
   */
  LaneGraph CombineLaneGraphs(const LaneGraph& lane_graph_predecessor,
                              const LaneGraph& lane_graph_successor);

  /**
   * @brief
   * @param Whether it is searching in the forward direction or backward.
   * @param The accumulated s-distance starting from the obstacle's position
   *        up until the beginning of current lane-segment.
   * @param The s_diff of the current position w.r.t. the start_s of the
   *        current lane-segment, regardless of the search direction.
   * @param The LaneInfo the current lane segment we are looking at.
   * @param The max. number of recursive calls (so that it won't recurse
   *        for too many times when given unreasonable speed info. etc.)
   * @param If we consider all successor lanes after dividing
   * @param The vector of lane_segments visited (DFS).
   * @param The LaneGraph that we need to write in.
   */
  void ConstructLaneSequence(
      const bool search_forward_direction, const double accumulated_s,
      const double curr_lane_seg_s,
      std::shared_ptr<const hdmap::LaneInfo> lane_info_ptr,
      const int graph_search_horizon, const bool consider_lane_split,
      std::list<LaneSegment>* const lane_segments,
      LaneGraph* const lane_graph_ptr) const;

  /** @brief If direction unspecified, by default construct forward direction.
   */
  void ConstructLaneSequence(
      const double accumulated_s, const double curr_lane_seg_s,
      std::shared_ptr<const hdmap::LaneInfo> lane_info_ptr,
      const int graph_search_horizon, const bool consider_lane_split,
      std::list<LaneSegment>* const lane_segments,
      LaneGraph* const lane_graph_ptr) const;

  /**
   * @brief Get the pointer to the lane with the smallest average curvature
   * @param The vector of lane infos
   * @return The pointer to the lane with the smallest average curvature
   */
  std::shared_ptr<const hdmap::LaneInfo> LaneWithSmallestAverageCurvature(
      const std::vector<std::shared_ptr<const hdmap::LaneInfo>>& lane_infos)
      const;

  /**
   * @brief Get the average curvature along a lane with the ID lane_id
   * @param The ID of the lane
   * @param The size of samples alone the lane to compute the average curvature
   * @return The average curvature
   */
  double AverageCurvature(const std::string& lane_id,
                          const size_t sample_size) const;

 private:
  // The s of the obstacle on its own lane_segment.
  double start_s_ = 0;

  // The total length to search for lane_graph.
  double length_ = -1.0;

  // If we consider all successor lanes after dividing
  bool consider_divide_ = false;

  // The lane_info of the lane_segment where the obstacle is on.
  std::shared_ptr<const hdmap::LaneInfo> lane_info_ptr_ = nullptr;
};

}  // namespace prediction
}  // namespace apollo
