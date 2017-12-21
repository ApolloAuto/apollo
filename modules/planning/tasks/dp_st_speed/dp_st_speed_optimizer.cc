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
 * @file dp_st_speed_optimizer.cc
 **/

#include "modules/planning/tasks/dp_st_speed/dp_st_speed_optimizer.h"

#include <algorithm>
#include <vector>

#include "modules/planning/proto/planning_internal.pb.h"

#include "modules/common/adapters/adapter_manager.h"
#include "modules/common/configs/vehicle_config_helper.h"
#include "modules/common/vehicle_state/vehicle_state_provider.h"
#include "modules/planning/common/planning_gflags.h"
#include "modules/planning/tasks/dp_st_speed/dp_st_graph.h"
#include "modules/planning/tasks/st_graph/st_graph_data.h"

namespace apollo {
namespace planning {

using apollo::common::ErrorCode;
using apollo::common::Status;
using apollo::common::VehicleConfigHelper;
using apollo::common::adapter::AdapterManager;
using apollo::localization::LocalizationEstimate;
using apollo::planning_internal::STGraphDebug;

DpStSpeedOptimizer::DpStSpeedOptimizer()
    : SpeedOptimizer("DpStSpeedOptimizer") {}

bool DpStSpeedOptimizer::Init(const PlanningConfig& config) {
  dp_st_speed_config_ = config.em_planner_config().dp_st_speed_config();
  st_boundary_config_ = dp_st_speed_config_.st_boundary_config();
  is_init_ = true;
  return true;
}

Status DpStSpeedOptimizer::Process(const SLBoundary& adc_sl_boundary,
                                   const PathData& path_data,
                                   const common::TrajectoryPoint& init_point,
                                   const ReferenceLine& reference_line,
                                   const SpeedData& reference_speed_data,
                                   PathDecision* const path_decision,
                                   SpeedData* const speed_data) {
  if (!is_init_) {
    AERROR << "Please call Init() before process DpStSpeedOptimizer.";
    return Status(ErrorCode::PLANNING_ERROR, "Not inited.");
  }

  if (path_data.discretized_path().NumOfPoints() == 0) {
    std::string msg("Empty path data");
    AERROR << msg;
    return Status(ErrorCode::PLANNING_ERROR, msg);
  }

  StBoundaryMapper boundary_mapper(adc_sl_boundary, st_boundary_config_,
                                   reference_line, path_data,
                                   dp_st_speed_config_.total_path_length(),
                                   dp_st_speed_config_.total_time());

  // step 1 get boundaries
  path_decision->EraseStBoundaries();
  if (boundary_mapper.GetGraphBoundary(path_decision).code() ==
      ErrorCode::PLANNING_ERROR) {
    const std::string msg =
        "Mapping obstacle for dp st speed optimizer failed.";
    AERROR << msg;
    return Status(ErrorCode::PLANNING_ERROR, msg);
  }
  std::vector<const StBoundary*> boundaries;
  for (const auto* obstacle : path_decision->path_obstacles().Items()) {
    if (!obstacle->st_boundary().IsEmpty()) {
      boundaries.push_back(&obstacle->st_boundary());
    }
  }

  // step 2 perform graph search
  SpeedLimit speed_limit;
  if (!boundary_mapper
           .GetSpeedLimits(path_decision->path_obstacles(), &speed_limit)
           .ok()) {
    const std::string msg =
        "Getting speed limits for dp st speed optimizer failed!";
    AERROR << msg;
    return Status(ErrorCode::PLANNING_ERROR, msg);
  }

  const double path_length = path_data.discretized_path().Length();
  StGraphData st_graph_data(boundaries, init_point, speed_limit, path_length);

  DpStGraph st_graph(reference_line, st_graph_data, dp_st_speed_config_,
                     path_data, adc_sl_boundary);
  auto* debug = reference_line_info_->mutable_debug();
  STGraphDebug* st_graph_debug = debug->mutable_planning_data()->add_st_graph();

  if (!st_graph.Search(path_decision, speed_data).ok()) {
    const std::string msg(Name() +
                          ":Failed to search graph with dynamic programming.");
    AERROR << msg;
    RecordSTGraphDebug(st_graph_data, st_graph_debug);
    return Status(ErrorCode::PLANNING_ERROR, msg);
  }

  RecordSTGraphDebug(st_graph_data, st_graph_debug);

  return Status::OK();
}

}  // namespace planning
}  // namespace apollo
