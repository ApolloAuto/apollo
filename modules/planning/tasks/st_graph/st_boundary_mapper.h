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

#ifndef MODULES_PLANNING_TASKS_ST_GRAPH_ST_BOUNDARY_MAPPER_H_
#define MODULES_PLANNING_TASKS_ST_GRAPH_ST_BOUNDARY_MAPPER_H_

#include <vector>

#include "modules/common/configs/proto/vehicle_config.pb.h"
#include "modules/planning/proto/st_boundary_config.pb.h"

#include "modules/common/status/status.h"
#include "modules/map/pnc_map/pnc_map.h"
#include "modules/planning/common/path/path_data.h"
#include "modules/planning/common/path_decision.h"
#include "modules/planning/common/speed/st_boundary.h"
#include "modules/planning/common/speed_limit.h"
#include "modules/planning/reference_line/reference_line.h"

namespace apollo {
namespace planning {

class StBoundaryMapper {
 public:
  StBoundaryMapper(const SLBoundary& adc_sl_boundary,
                   const StBoundaryConfig& config,
                   const ReferenceLine& reference_line,
                   const PathData& path_data, const double planning_distance,
                   const double planning_time);

  virtual ~StBoundaryMapper() = default;

  apollo::common::Status GetGraphBoundary(PathDecision* path_decision) const;

  virtual apollo::common::Status GetSpeedLimits(
      SpeedLimit* const speed_limit_data) const;

 private:
  FRIEND_TEST(StBoundaryMapperTest, check_overlap_test);
  bool CheckOverlap(const apollo::common::PathPoint& path_point,
                    const apollo::common::math::Box2d& obs_box,
                    const double buffer) const;

  /**
   * Creates valid st boundary upper_points and lower_points
   * If return true, upper_points.size() > 1 and
   * upper_points.size() = lower_points.size()
   */
  bool GetOverlapBoundaryPoints(
      const std::vector<apollo::common::PathPoint>& path_points,
      const Obstacle& obstacle, std::vector<STPoint>* upper_points,
      std::vector<STPoint>* lower_points) const;

  apollo::common::Status MapWithoutDecision(PathObstacle* path_obstacle) const;

  bool MapStopDecision(PathObstacle* stop_obstacle) const;

  apollo::common::Status MapWithPredictionTrajectory(
      PathObstacle* path_obstacle) const;

  double GetCentricAccLimit(const double kappa) const;

  void GetAvgKappa(const std::vector<common::PathPoint>& path_points,
                   std::vector<double>* kappa) const;

 private:
  const SLBoundary& adc_sl_boundary_;
  StBoundaryConfig st_boundary_config_;
  const ReferenceLine& reference_line_;
  const PathData& path_data_;
  const apollo::common::VehicleParam vehicle_param_;
  const double planning_distance_;
  const double planning_time_;
};

}  // namespace planning
}  // namespace apollo

#endif  // MODULES_PLANNING_TASKS_ST_GRAPH_ST_BOUNDARY_MAPPER_H_
