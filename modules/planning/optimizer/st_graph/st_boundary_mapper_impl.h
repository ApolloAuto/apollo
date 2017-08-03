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
*   @file: st_boundary_mapper_impl.h
**/

#ifndef MODULES_PLANNING_OPTIMIZER_ST_GRAPH_ST_BOUNDARY_MAPPER_IMPL_H_
#define MODULES_PLANNING_OPTIMIZER_ST_GRAPH_ST_BOUNDARY_MAPPER_IMPL_H_

#include <vector>

#include "modules/planning/optimizer/st_graph/st_boundary_mapper.h"

#include "modules/common/configs/proto/vehicle_config.pb.h"
#include "modules/planning/proto/st_boundary_config.pb.h"

#include "modules/planning/common/decision_data.h"

namespace apollo {
namespace planning {

class StBoundaryMapperImpl : public StBoundaryMapper {
 public:
  apollo::common::Status GetGraphBoundary(
      const common::TrajectoryPoint& initial_planning_point,
      const DecisionData& decision_data, const PathData& path_data,
      const ReferenceLine& reference_line, const double planning_distance,
      const double planning_time,
      std::vector<StGraphBoundary>* const boundary) const override;

 private:
  apollo::common::Status MapMainDecisionStop(
      const MainStop& main_stop, const ReferenceLine& reference_line,
      const double planning_distance, const double planning_time,
      std::vector<StGraphBoundary>* const boundary) const;

  apollo::common::Status MapMissionComplete(
      const ReferenceLine& reference_line, const double planning_distance,
      const double planning_time,
      std::vector<StGraphBoundary>* const boundary) const;

  apollo::common::Status MapObstacleWithPredictionTrajectory(
      const common::TrajectoryPoint& initial_planning_point,
      const Obstacle& obstacle, const ObjectDecisionType obj_decision,
      const PathData& path_data, const double planning_distance,
      const double planning_time,
      std::vector<StGraphBoundary>* const boundary) const;

  apollo::common::Status MapObstacleWithoutPredictionTrajectory(
      const common::TrajectoryPoint& initial_planning_point,
      const Obstacle& obstacle, const ObjectDecisionType obj_decision,
      const PathData& path_data, const ReferenceLine& reference_line,
      const double planning_distance, const double planning_time,
      std::vector<StGraphBoundary>* const boundary) const;
};

}  // namespace planning
}  // namespace apollo

#endif  // MODULES_PLANNING_OPTIMIZER_ST_GRAPH_ST_BOUNDARY_MAPPER_IMPL_H_
