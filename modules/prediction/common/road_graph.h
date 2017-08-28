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

#ifndef MODULES_PREDICTION_COMMON_ROAD_GRAPH_H_
#define MODULES_PREDICTION_COMMON_ROAD_GRAPH_H_

#include <vector>
#include <memory>

#include "modules/common/status/status.h"
#include "modules/map/hdmap/hdmap_common.h"
#include "modules/prediction/proto/lane_graph.pb.h"

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
  RoadGraph(double start_s, double length,
            std::shared_ptr<const apollo::hdmap::LaneInfo> lane_info_ptr);

  /**
   * @brief Build the lane graph.
   * @param The built lane graph.
   * @return The status of the road graph building.
   */
  apollo::common::Status BuildLaneGraph(LaneGraph* lane_graph);

  /**
   * @brief Check if a lane with an s is on the lane graph
   * @param Lane ID
   * @param Lane s
   * @param The pointer to the given lane graph
   * @return If the given lane ID and lane s is on the lane graph
   */
  bool IsOnLaneGraph(
      std::shared_ptr<const apollo::hdmap::LaneInfo> lane_info_ptr,
      const LaneGraph& lane_graph);

 private:
  void ComputeLaneSequence(
      double accumulated_s,
      double start_s,
      std::shared_ptr<const apollo::hdmap::LaneInfo> lane_info_ptr,
      std::vector<LaneSegment>* lane_segments,
      LaneGraph* lane_graph_ptr) const;

 private:
  double start_s_ = 0;
  double length_ = -1.0;
  std::shared_ptr<const apollo::hdmap::LaneInfo> lane_info_ptr_ = nullptr;
};

}  // namespace prediction
}  // namespace apollo

#endif  // MODULES_PREDICTION_COMMON_ROAD_GRAPH_H_
