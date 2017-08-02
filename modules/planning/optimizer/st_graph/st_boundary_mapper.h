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

#include <string>
#include <vector>

#include "modules/common/proto/pnc_point.pb.h"
#include "modules/common/status/status.h"
#include "modules/localization/proto/pose.pb.h"
#include "modules/planning/proto/st_boundary_config.pb.h"

#include "modules/common/configs/vehicle_config_helper.h"
#include "modules/map/pnc_map/pnc_map.h"
#include "modules/planning/common/decision_data.h"
#include "modules/planning/common/path/path_data.h"
#include "modules/planning/common/speed_limit.h"
#include "modules/planning/optimizer/st_graph/st_graph_boundary.h"
#include "modules/planning/optimizer/st_graph/st_graph_point.h"
#include "modules/planning/reference_line/reference_line.h"

namespace apollo {
namespace planning {

class StBoundaryMapper {
 public:
  StBoundaryMapper() = default;
  virtual ~StBoundaryMapper() = default;

  bool Init(const std::string& config_file);
  virtual apollo::common::Status GetGraphBoundary(
      const common::TrajectoryPoint& initial_planning_point,
      const DecisionData& decision_data, const PathData& path_data,
      const ReferenceLine& reference_line, const double planning_distance,
      const double planning_time,
      std::vector<StGraphBoundary>* const boundary) const = 0;

  virtual apollo::common::Status GetSpeedLimits(
      const ReferenceLine& reference_line, const PathData& path_data,
      SpeedLimit* const speed_limit_data) const;

 protected:
  const StBoundaryConfig& st_boundary_config() const;
  const apollo::common::VehicleParam& vehicle_param() const {
    return common::VehicleConfigHelper::GetConfig().vehicle_param();
  }

  double GetArea(const std::vector<STPoint>& boundary_points) const;
  bool CheckOverlap(const apollo::common::PathPoint& path_point,
                    const apollo::common::VehicleParam& params,
                    const apollo::common::math::Box2d& obs_box,
                    const double buffer) const;

 private:
  StBoundaryConfig st_boundary_config_;
};

}  // namespace planning
}  // namespace apollo

#endif  // MODULES_PLANNING_OPTIMIZER_ST_GRAPH_ST_BOUNDARY_MAPPER_H_
