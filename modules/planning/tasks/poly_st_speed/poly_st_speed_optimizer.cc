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
 * @file
 **/

#include "modules/planning/tasks/poly_st_speed/poly_st_speed_optimizer.h"

#include <algorithm>
#include <string>
#include <utility>
#include <vector>

#include "modules/common/proto/pnc_point.pb.h"

#include "modules/common/configs/vehicle_config_helper.h"
#include "modules/common/util/file.h"
#include "modules/common/vehicle_state/vehicle_state_provider.h"
#include "modules/planning/common/planning_gflags.h"
#include "modules/planning/tasks/poly_st_speed/poly_st_graph.h"
#include "modules/planning/tasks/st_graph/speed_limit_decider.h"
#include "modules/planning/tasks/st_graph/st_graph_data.h"

namespace apollo {
namespace planning {

using apollo::common::ErrorCode;
using apollo::common::Status;
using apollo::common::TrajectoryPoint;
using apollo::planning_internal::STGraphDebug;

PolyStSpeedOptimizer::PolyStSpeedOptimizer()
    : SpeedOptimizer("PolyStSpeedOptimizer") {}

bool PolyStSpeedOptimizer::Init(const PlanningConfig& config) {
  if (is_init_) {
    AERROR << "Duplicated Init.";
    return false;
  }
  poly_st_speed_config_ = config.em_planner_config().poly_st_speed_config();
  st_boundary_config_ = poly_st_speed_config_.st_boundary_config();
  is_init_ = true;
  return true;
}

Status PolyStSpeedOptimizer::Process(const SLBoundary& adc_sl_boundary,
                                     const PathData& path_data,
                                     const TrajectoryPoint& init_point,
                                     const ReferenceLine& reference_line,
                                     const SpeedData& reference_speed_data,
                                     PathDecision* const path_decision,
                                     SpeedData* const speed_data) {
  if (reference_line_info_->ReachedDestination()) {
    return Status::OK();
  }
  if (!is_init_) {
    AERROR << "Please call Init() before Process.";
    return Status(ErrorCode::PLANNING_ERROR, "Not init.");
  }

  if (path_data.discretized_path().NumOfPoints() == 0) {
    std::string msg("Empty path data");
    AERROR << msg;
    return Status(ErrorCode::PLANNING_ERROR, msg);
  }

  StBoundaryMapper boundary_mapper(adc_sl_boundary, st_boundary_config_,
                                   reference_line, path_data,
                                   poly_st_speed_config_.total_path_length(),
                                   poly_st_speed_config_.total_time(),
                                   reference_line_info_->IsChangeLanePath());

  for (const auto* path_obstacle : path_decision->path_obstacles().Items()) {
    DCHECK(path_obstacle->HasLongitudinalDecision());
  }
  // step 1 get boundaries
  path_decision->EraseStBoundaries();
  if (boundary_mapper.CreateStBoundary(path_decision).code() ==
      ErrorCode::PLANNING_ERROR) {
    return Status(ErrorCode::PLANNING_ERROR,
                  "Mapping obstacle for qp st speed optimizer failed!");
  }

  for (const auto* obstacle : path_decision->path_obstacles().Items()) {
    auto id = obstacle->Id();
    auto* mutable_obstacle = path_decision->Find(id);

    if (!obstacle->st_boundary().IsEmpty()) {
      mutable_obstacle->SetBlockingObstacle(true);
    } else {
      path_decision->SetStBoundary(
          id, path_decision->Find(id)->reference_line_st_boundary());
    }
  }

  SpeedLimitDecider speed_limit_decider(adc_sl_boundary, st_boundary_config_,
                                        reference_line, path_data);
  SpeedLimit speed_limits;
  if (speed_limit_decider.GetSpeedLimits(path_decision->path_obstacles(),
                                         &speed_limits) != Status::OK()) {
    return Status(ErrorCode::PLANNING_ERROR,
                  "GetSpeedLimits for qp st speed optimizer failed!");
  }

  // step 2 perform graph search
  // make a poly_st_graph and perform search here.
  PolyStGraph poly_st_graph(poly_st_speed_config_, reference_line_info_,
                            speed_limits);
  auto ret = poly_st_graph.FindStTunnel(
      init_point,
      reference_line_info_->path_decision()->path_obstacles().Items(),
      speed_data);
  if (!ret) {
    return Status(ErrorCode::PLANNING_ERROR,
                  "Fail to find st tunnel in PolyStGraph.");
  }
  return Status::OK();
}

}  // namespace planning
}  // namespace apollo
