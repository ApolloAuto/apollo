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
*   @file: dp_st_boundary_mapper.h
**/

#ifndef MODULES_PLANNING_OPTIMIZER_DP_ST_SPEED_DP_ST_BOUNDARY_MAPPER_H_
#define MODULES_PLANNING_OPTIMIZER_DP_ST_SPEED_DP_ST_BOUNDARY_MAPPER_H_

#include <vector>

#include "modules/planning/common/data_center.h"
#include "modules/planning/optimizer/st_graph/st_boundary_mapper.h"
#include "modules/planning/proto/st_boundary_config.pb.h"

namespace apollo {
namespace planning {

class DpStBoundaryMapper : public StBoundaryMapper {
 public:
  DpStBoundaryMapper(const StBoundaryConfig& st_boundary_config,
                     const ::apollo::common::config::VehicleParam& veh_param);

  // TODO: combine two interfaces together to provide a st graph data type
  apollo::common::Status get_graph_boundary(
      const common::TrajectoryPoint& initial_planning_point,
      const DecisionData& decision_data, const PathData& path_data,
      const ReferenceLine& reference_line, const double planning_distance,
      const double planning_time,
      std::vector<StGraphBoundary>* const boundary) const override;

 private:
  apollo::common::Status map_obstacle_with_trajectory(
      const Obstacle& obstacle, const PathData& path_data,
      const double planning_distance, const double planning_time,
      std::vector<StGraphBoundary>* const boundary) const;

  apollo::common::Status map_obstacle_without_trajectory(
      const Obstacle& obstacle, const PathData& path_data,
      const double planning_distance, const double planning_time,
      std::vector<StGraphBoundary>* const boundary) const;
};

}  // namespace planning
}  // namespace apollo

#endif  // MODULES_PLANNING_OPTIMIZER_DP_ST_SPEED_DP_ST_BOUNDARY_MAPPER_H_
