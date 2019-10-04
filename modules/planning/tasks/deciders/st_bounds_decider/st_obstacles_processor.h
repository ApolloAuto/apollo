/******************************************************************************
 * Copyright 2019 The Apollo Authors. All Rights Reserved.
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

/**
 *   @file
 **/

#pragma once

#include <string>
#include <vector>
#include <unordered_map>
#include <utility>

#include "modules/common/configs/proto/vehicle_config.pb.h"
#include "modules/planning/proto/st_bounds_decider_config.pb.h"

#include "modules/common/status/status.h"
#include "modules/planning/common/obstacle.h"
#include "modules/planning/common/path/path_data.h"
#include "modules/planning/common/path_decision.h"
#include "modules/planning/common/speed/st_boundary.h"
#include "modules/planning/common/speed_limit.h"
#include "modules/planning/reference_line/reference_line.h"

namespace apollo {
namespace planning {

class STObstaclesProcessor {
 public:
  STObstaclesProcessor(
      const double planning_distance, const double planning_time,
      const PathData& path_data);

  virtual ~STObstaclesProcessor() = default;

  common::Status MapObstaclesToSTBoundaries(PathDecision* const path_decision);

  std::pair<double, double> GetRegularBoundaryFromObstacles(double t);

  std::pair<double, double> GetFallbackBoundaryFromObstacles(double t);

 private:
  /** @brief Given a single obstacle, compute its ST-boundary.
    * @return If appears on ST-graph, return true; otherwise, false.
    */
  bool ComputeObstacleSTBoundary(const Obstacle& obstacle);

  bool GetOverlappingS(
      const std::vector<common::PathPoint>& adc_path_points,
      const common::math::Box2d& obstacle_instance, const double adc_l_buffer,
      std::pair<double, double>* const overlapping_s);

  /** @brief Over the s-dimension, find the last point that is before the
    * obstacle instance of the first point that is after the obstacle.
    * If there exists no such point within the given range, return -1.
    * @param ADC path points
    * @param The obstacle box
    * @param The s threshold, must be non-negative.
    * @param The direction
    * @param The start-idx
    * @param The end-idx
    * @return Whether there is overlapping or not.
    */
  int GetSBoundingPathPointIndex(
      const std::vector<common::PathPoint>& adc_path_points,
      const common::math::Box2d& obstacle_instance, const double s_thresh,
      const bool is_before, const int start_idx, const int end_idx);

  /** @brief Over the s-dimension, check if the path-point is away
    * from the projected obstacle in the given direction.
    * @param A certain path-point.
    * @param The next path-point indicating path direction.
    * @param The obstacle bounding box.
    * @param The threshold s to tell if path point is far away.
    * @param Direction indicator. True if we want the path-point to be
    *        before the obstacle.
    * @return whether the path-point is away in the indicated direction.
    */
  bool IsPathPointAwayFromObstacle(
      const common::PathPoint& path_point,
      const common::PathPoint& direction_point,
      const common::math::Box2d& obs_box, const double s_thresh,
      const bool is_before);

  bool IsADCOverlappingWithObstacle(
      const common::PathPoint& adc_path_point,
      const common::math::Box2d& obs_box,
      const double l_buffer) const;

 private:
  double planning_time_;
  double planning_distance_;
  const PathData& path_data_;
  const common::VehicleParam& vehicle_param_;

  std::unordered_map<std::string, STBoundary> obs_id_to_st_boundary_;
};

}  // namespace planning
}  // namespace apollo
