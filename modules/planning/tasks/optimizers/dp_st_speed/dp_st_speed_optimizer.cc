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

#include "modules/planning/tasks/optimizers/dp_st_speed/dp_st_speed_optimizer.h"

#include <vector>

#include "modules/planning/proto/planning_internal.pb.h"

#include "modules/common/configs/vehicle_config_helper.h"
#include "modules/common/vehicle_state/vehicle_state_provider.h"
#include "modules/planning/common/planning_gflags.h"
#include "modules/planning/tasks/optimizers/dp_st_speed/dp_st_graph.h"
#include "modules/planning/tasks/optimizers/st_graph/st_graph_data.h"

namespace apollo {
namespace planning {

using apollo::common::ErrorCode;
using apollo::common::Status;
using apollo::common::TrajectoryPoint;
using apollo::planning_internal::STGraphDebug;

DpStSpeedOptimizer::DpStSpeedOptimizer(const TaskConfig& config)
    : SpeedOptimizer(config) {
  CHECK(config.has_dp_st_speed_config());
  dp_st_speed_config_ = config.dp_st_speed_config();
  st_boundary_config_ = dp_st_speed_config_.st_boundary_config();
  SetName("DpStSpeedOptimizer");
}

bool DpStSpeedOptimizer::SearchStGraph(
    const StBoundaryMapper& boundary_mapper,
    const SpeedLimitDecider& speed_limit_decider, const PathData& path_data,
    SpeedData* speed_data, PathDecision* path_decision,
    STGraphDebug* st_graph_debug) const {
  std::vector<const StBoundary*> boundaries;
  for (auto* obstacle : path_decision->obstacles().Items()) {
    auto id = obstacle->Id();
    if (!obstacle->st_boundary().IsEmpty()) {
      if (obstacle->st_boundary().boundary_type() ==
          StBoundary::BoundaryType::KEEP_CLEAR) {
        path_decision->Find(id)->SetBlockingObstacle(false);
      } else {
        path_decision->Find(id)->SetBlockingObstacle(true);
      }
      boundaries.push_back(&obstacle->st_boundary());
    }
  }

  // step 2 perform graph search
  SpeedLimit speed_limit;
  if (!speed_limit_decider
           .GetSpeedLimits(path_decision->obstacles(), &speed_limit)
           .ok()) {
    AERROR << "Getting speed limits for dp st speed optimizer failed!";
    return false;
  }

  const double path_length = path_data.discretized_path().Length();
  StGraphData st_graph_data(boundaries, init_point_, speed_limit, path_length);

  DpStGraph st_graph(st_graph_data, dp_st_speed_config_,
                     reference_line_info_->path_decision()->obstacles().Items(),
                     init_point_, adc_sl_boundary_);

  if (!st_graph.Search(speed_data).ok()) {
    AERROR << "failed to search graph with dynamic programming.";
    RecordSTGraphDebug(st_graph_data, st_graph_debug);
    return false;
  }
  RecordSTGraphDebug(st_graph_data, st_graph_debug);
  return true;
}

Status DpStSpeedOptimizer::Process(const SLBoundary& adc_sl_boundary,
                                   const PathData& path_data,
                                   const TrajectoryPoint& init_point,
                                   const ReferenceLine& reference_line,
                                   const SpeedData& reference_speed_data,
                                   PathDecision* const path_decision,
                                   SpeedData* const speed_data) {
  init_point_ = init_point;
  adc_sl_boundary_ = adc_sl_boundary;
  reference_line_ = &reference_line;

  if (path_data.discretized_path().empty()) {
    std::string msg("Empty path data");
    AERROR << msg;
    return Status(ErrorCode::PLANNING_ERROR, msg);
  }

  StBoundaryMapper boundary_mapper(
      adc_sl_boundary, st_boundary_config_, *reference_line_, path_data,
      dp_st_speed_config_.total_path_length(), dp_st_speed_config_.total_time(),
      reference_line_info_->IsChangeLanePath());

  auto* debug = reference_line_info_->mutable_debug();
  STGraphDebug* st_graph_debug = debug->mutable_planning_data()->add_st_graph();

  path_decision->EraseStBoundaries();
  if (boundary_mapper.CreateStBoundary(path_decision).code() ==
      ErrorCode::PLANNING_ERROR) {
    const std::string msg =
        "Mapping obstacle for dp st speed optimizer failed.";
    AERROR << msg;
    return Status(ErrorCode::PLANNING_ERROR, msg);
  }

  SpeedLimitDecider speed_limit_decider(adc_sl_boundary, st_boundary_config_,
                                        *reference_line_, path_data);

  if (!SearchStGraph(boundary_mapper, speed_limit_decider, path_data,
                     speed_data, path_decision, st_graph_debug)) {
    const std::string msg(Name() +
                          ":Failed to search graph with dynamic programming.");
    AERROR << msg;
    return Status(ErrorCode::PLANNING_ERROR, msg);
  }
  return Status::OK();
}

}  // namespace planning
}  // namespace apollo
