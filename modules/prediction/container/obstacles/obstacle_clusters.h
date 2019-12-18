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
#include <string>
#include <unordered_map>
#include <utility>
#include <vector>

#include "modules/prediction/proto/feature.pb.h"

#include "modules/map/hdmap/hdmap_common.h"

namespace apollo {
namespace prediction {

class ObstacleClusters {
 public:
  /**
   * @brief Remove all lane graphs
   */
  static void Init();

  /**
   * @brief Obtain a lane graph given a lane info and s
   * @param lane start s
   * @param lane total length
   * @param if consider lane split ahead
   * @param lane info
   * @return a corresponding lane graph
   */
  static LaneGraph GetLaneGraph(
      const double start_s, const double length, const bool consider_lane_split,
      std::shared_ptr<const apollo::hdmap::LaneInfo> lane_info_ptr);

  /**
   * @brief Obtain a lane graph given a lane info and s, but don't
   *        memorize it.
   * @param lane start s
   * @param lane total length
   * @param if the obstacle is on lane
   * @param lane info
   * @return a corresponding lane graph
   */
  static LaneGraph GetLaneGraphWithoutMemorizing(
      const double start_s, const double length, const bool is_on_lane,
      std::shared_ptr<const apollo::hdmap::LaneInfo> lane_info_ptr);

  /**
   * @brief Get the nearest obstacle on lane sequence at s
   * @param Lane sequence
   * @param s offset in the first lane of the lane sequence
   * @param the forward obstacle on lane
   * @return If the forward obstacle is found
   */
  static bool ForwardNearbyObstacle(const LaneSequence& lane_sequence,
                                    const double s,
                                    LaneObstacle* const lane_obstacle);

  /**
   * @brief Add an obstacle into clusters
   * @param obstacle id
   * @param lane id
   * @param lane s
   * @param lane l
   */
  static void AddObstacle(const int obstacle_id, const std::string& lane_id,
                          const double lane_s, const double lane_l);

  /**
   * @brief Sort lane obstacles by lane s
   */
  static void SortObstacles();

  /**
   * @brief Get the forward nearest obstacle on lane sequence at s
   * @param Lane sequence
   * @param s offset in the first lane of the lane sequence
   * @param the forward obstacle on lane
   * @return If the forward obstacle is found
   */
  static bool ForwardNearbyObstacle(const LaneSequence& lane_sequence,
                                    const int obstacle_id,
                                    const double obstacle_s,
                                    const double obstacle_l,
                                    NearbyObstacle* const nearby_obstacle_ptr);

  /**
   * @brief Get the backward nearest obstacle on lane sequence at s
   * @param Lane sequence
   * @param s offset in the first lane of the lane sequence
   * @param the forward obstacle on lane
   * @return If the backward obstacle is found
   */
  static bool BackwardNearbyObstacle(const LaneSequence& lane_sequence,
                                     const int obstacle_id,
                                     const double obstacle_s,
                                     const double obstacle_l,
                                     NearbyObstacle* const nearby_obstacle_ptr);

  /**
   * @brief Query stop sign by lane ID
   * @param lane ID
   * @return the stop sign
   */
  static StopSign QueryStopSignByLaneId(const std::string& lane_id);

  static std::unordered_map<std::string, std::vector<LaneObstacle>>&
  GetLaneObstacles() {
    return lane_obstacles_;
  }

 private:
  ObstacleClusters() = delete;

  static void Clear();

 private:
  static std::unordered_map<std::string, std::vector<LaneObstacle>>
      lane_obstacles_;
  static std::unordered_map<std::string, StopSign> lane_id_stop_sign_map_;
};

}  // namespace prediction
}  // namespace apollo
