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
*   @file: st_boundary_mapper.h
**/

#ifndef MODULES_PLANNING_OPTIMIZER_ST_GRAPH_ST_BOUNDARY_MAPPER_H_
#define MODULES_PLANNING_OPTIMIZER_ST_GRAPH_ST_BOUNDARY_MAPPER_H_

#include "config.pb.h"
#include "modules/planning/common/data_center.h"
#include "modules/planning/common/decision_data.h"
#include "modules/planning/common/environment.h"
#include "modules/planning/common/path/path_data.h"
#include "modules/planning/common/planning_error.h"
#include "modules/planning/common/speed_limit.h"
#include "modules/planning/optimizer/st_graph/st_boundary_config.h"
#include "modules/planning/optimizer/st_graph/st_graph_boundary.h"
#include "modules/planning/optimizer/st_graph/st_graph_point.h"
#include "modules/planning/reference_line/reference_line.h"

namespace apollo {
namespace planning {

class STBoundaryMapper {
 public:
  STBoundaryMapper(const STBoundaryConfig& st_boundary_config,
                   const ::adu::common::config::VehicleParam& veh_param);

  virtual ~STBoundaryMapper() = default;

  // TODO: combine two interfaces together to provide a st graph data type
  // TODO: data_center -> planning_data or environment, or whatever you need
  virtual ErrorCode get_graph_boundary(
      const DataCenter& data_center, const DecisionData& decision_data,
      const PathData& path_data, const double planning_distance,
      const double planning_time,
      std::vector<STGraphBoundary>* const boundary) const = 0;

  virtual ErrorCode get_speed_limits(const Environment& environment,
                                     const LogicMap& map,
                                     const PathData& path_data,
                                     const double planning_distance,
                                     const std::size_t matrix_dimension_s,
                                     const double default_speed_limit,
                                     SpeedLimit* const speed_limit_data) const;

  const STBoundaryConfig& st_boundary_config() const;
  const ::adu::common::config::VehicleParam& vehicle_param() const;

 protected:
  double get_area(const std::vector<STPoint>& boundary_points) const;

  bool check_overlap(const PathPoint& path_point,
                     const ::adu::common::config::VehicleParam& params,
                     const ::adu::common::math::Box2d& obs_box,
                     const double buffer) const;

 private:
  STBoundaryConfig _st_boundary_config;
  ::adu::common::config::VehicleParam _veh_param;
};

}  // namespace planning
}  // namespace apollo

#endif  // MODULES_PLANNING_OPTIMIZER_ST_GRAPH_ST_BOUNDARY_MAPPER_H_
