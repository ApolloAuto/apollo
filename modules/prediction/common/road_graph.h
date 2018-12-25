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

#include <memory>
#include <vector>

#include "modules/prediction/proto/lane_graph.pb.h"

#include "modules/common/status/status.h"
#include "modules/prediction/common/prediction_map.h"

namespace apollo {
namespace prediction {

class RoadGraph {
 public:
  /**
   * @brief Constructor
   * @param start_s The starting longitudinal s value.
   * @param length The length to build the road graph.
   * @param lane_info_ptr The starting lane.
   */
  RoadGraph(const double start_s, const double length,
            std::shared_ptr<const hdmap::LaneInfo> lane_info_ptr);

  /**
   * @brief Build the lane graph.
   * @param The built lane graph.
   * @return The status of the road graph building.
   */
  common::Status BuildLaneGraph(LaneGraph* const lane_graph);

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
  /**
    * @brief
    * @param The accumulated s starting from the obstacle's position.
    * @param The starting s of the lane_segment to compute lane_sequence,
    *        this should be start_s_ for the first time, and zero for
    *        subsequent recursions.
    * @param The LaneInfo the current lane segment we are looking at.
    * @param The max. number of recursive calls (so that it won't recurse
    *        for too many times when given unreasonable speed info. etc.)
    * @param The vector of lane_segments visited (DFS).
    * @param The LaneGraph that we need to write in.
  */
  void ComputeLaneSequence(const double accumulated_s, const double start_s,
                           std::shared_ptr<const hdmap::LaneInfo> lane_info_ptr,
                           const int graph_search_horizon,
                           std::vector<LaneSegment>* const lane_segments,
                           LaneGraph* const lane_graph_ptr) const;

 private:
  // The s of the obstacle on its own lane_segment.
  double start_s_ = 0;

  // The total length to search for lane_graph.
  double length_ = -1.0;

  // The lane_info of the lane_segment where the obstacle is on.
  std::shared_ptr<const hdmap::LaneInfo> lane_info_ptr_ = nullptr;
};

}  // namespace prediction
}  // namespace apollo
