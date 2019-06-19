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

/**
 *   @file
 **/

#pragma once

#include <string>
#include <vector>

#include "modules/common/configs/proto/vehicle_config.pb.h"
#include "modules/planning/proto/speed_bounds_decider_config.pb.h"

#include "modules/common/status/status.h"
#include "modules/planning/common/obstacle.h"
#include "modules/planning/common/path/path_data.h"
#include "modules/planning/common/path_decision.h"
#include "modules/planning/common/speed/st_boundary.h"
#include "modules/planning/common/speed_limit.h"
#include "modules/planning/reference_line/reference_line.h"

namespace apollo {
namespace planning {

class STBoundaryMapper {
 public:
  STBoundaryMapper(const SpeedBoundsDeciderConfig& config,
                   const ReferenceLine& reference_line,
                   const PathData& path_data, const double planning_distance,
                   const double planning_time);

  virtual ~STBoundaryMapper() = default;

  common::Status ComputeSTBoundary(PathDecision* path_decision) const;

 private:
  FRIEND_TEST(StBoundaryMapperTest, check_overlap_test);
  bool CheckOverlap(const common::PathPoint& path_point,
                    const common::math::Box2d& obs_box,
                    const double buffer) const;

  /**
   * Creates valid st boundary upper_points and lower_points
   * If return true, upper_points.size() > 1 and
   * upper_points.size() = lower_points.size()
   */
  bool GetOverlapBoundaryPoints(
      const std::vector<common::PathPoint>& path_points,
      const Obstacle& obstacle, std::vector<STPoint>* upper_points,
      std::vector<STPoint>* lower_points) const;

  void ComputeSTBoundary(Obstacle* obstacle) const;

  bool MapStopDecision(Obstacle* stop_obstacle,
                       const ObjectDecisionType& decision) const;

  void ComputeSTBoundaryWithDecision(Obstacle* obstacle,
                                     const ObjectDecisionType& decision) const;

 private:
  const SpeedBoundsDeciderConfig& speed_bounds_config_;
  const ReferenceLine& reference_line_;
  const PathData& path_data_;
  const common::VehicleParam& vehicle_param_;
  const double planning_max_distance_;
  const double planning_max_time_;
};

}  // namespace planning
}  // namespace apollo
