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
 * @file path_time_heuristic_optimizer.cc
 **/

#include "modules/planning/tasks/optimizers/path_time_heuristic/path_time_heuristic_optimizer.h"

#include "modules/common/configs/vehicle_config_helper.h"
#include "modules/common/vehicle_state/vehicle_state_provider.h"
#include "modules/planning/common/planning_gflags.h"
#include "modules/planning/common/st_graph_data.h"
#include "modules/planning/proto/planning_internal.pb.h"
#include "modules/planning/tasks/optimizers/path_time_heuristic/gridded_path_time_graph.h"

namespace apollo {
namespace planning {

using apollo::common::ErrorCode;
using apollo::common::Status;

PathTimeHeuristicOptimizer::PathTimeHeuristicOptimizer(const TaskConfig& config)
    : SpeedOptimizer(config) {
  ACHECK(config.has_speed_heuristic_config());
  speed_heuristic_config_ = config.speed_heuristic_config();
}

bool PathTimeHeuristicOptimizer::SearchPathTimeGraph(
    SpeedData* speed_data) const {
  GriddedPathTimeGraph st_graph(
      reference_line_info_->st_graph_data(), dp_st_speed_config_,
      reference_line_info_->path_decision()->obstacles().Items(), init_point_);

  if (!st_graph.Search(speed_data).ok()) {
    AERROR << "failed to search graph with dynamic programming.";
    return false;
  }
  return true;
}

Status PathTimeHeuristicOptimizer::Process(
    const PathData& path_data, const common::TrajectoryPoint& init_point,
    SpeedData* const speed_data) {
  init_point_ = init_point;

  dp_st_speed_config_ = reference_line_info_->IsChangeLanePath()
                            ? speed_heuristic_config_.lane_change_speed_config()
                            : speed_heuristic_config_.default_speed_config();

  if (path_data.discretized_path().empty()) {
    std::string msg("Empty path data");
    AERROR << msg;
    return Status(ErrorCode::PLANNING_ERROR, msg);
  }

  if (!SearchPathTimeGraph(speed_data)) {
    const std::string msg(Name() +
                          ":Failed to search graph with dynamic programming.");
    AERROR << msg;
    RecordDebugInfo(*speed_data, reference_line_info_->mutable_st_graph_data()
                                     ->mutable_st_graph_debug());
    return Status(ErrorCode::PLANNING_ERROR, msg);
  }
  RecordDebugInfo(
      *speed_data,
      reference_line_info_->mutable_st_graph_data()->mutable_st_graph_debug());
  return Status::OK();
}

}  // namespace planning
}  // namespace apollo
