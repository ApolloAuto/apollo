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

#ifndef BAIDU_IDG_HOUSTON_OPTIMIZER_DP_ST_SPEED_OPTIMIZER_DP_ST_BOUNDARY_MAPPER_H_
#define BAIDU_IDG_HOUSTON_OPTIMIZER_DP_ST_SPEED_OPTIMIZER_DP_ST_BOUNDARY_MAPPER_H_

#include "optimizer/st_graph/st_boundary_mapper.h"
#include "util/data_center.h"

namespace apollo {
namespace planning {

class DPSTBoundaryMapper : public StBoundaryMapper {
 public:
  DPSTBoundaryMapper(const STBoundaryConfig& st_boundary_config,
                     const ::adu::common::config::VehicleParam& veh_param);

  // TODO: combine two interfaces together to provide a st graph data type
  virtual ErrorCode get_graph_boundary(
      const DataCenter& data_center, const DecisionData& decision_data,
      const PathData& path_data, const double planning_distance,
      const double planning_time,
      std::vector<STGraphBoundary>* const boundary) const override;

 private:
  ErrorCode map_obstacle_with_trajectory(
      const Obstacle& obstacle, const PathData& path_data,
      const double planning_distance, const double planning_time,
      std::vector<STGraphBoundary>* const boundary) const;

  ErrorCode map_obstacle_without_trajectory(
      const Obstacle& obstacle, const PathData& path_data,
      const double planning_distance, const double planning_time,
      std::vector<STGraphBoundary>* const boundary) const;
};

}  // namespace planning
}  // namespace apollo

#endif  // BAIDU_IDG_HOUSTON_OPTIMIZER_ST_GRAPH_ST_BOUNDARY_MAPPER_H_
