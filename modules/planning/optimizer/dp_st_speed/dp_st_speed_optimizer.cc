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

#include "modules/planning/optimizer/dp_st_speed/dp_st_speed_optimizer.h"

#include <algorithm>
#include <vector>

#include "modules/common/adapters/adapter_manager.h"
#include "modules/common/configs/vehicle_config_helper.h"
#include "modules/common/vehicle_state/vehicle_state.h"
#include "modules/planning/common/planning_gflags.h"
#include "modules/planning/optimizer/dp_st_speed/dp_st_graph.h"
#include "modules/planning/optimizer/st_graph/st_graph_data.h"

namespace apollo {
namespace planning {

using apollo::common::ErrorCode;
using apollo::common::Status;
using apollo::common::VehicleConfigHelper;
using apollo::common::adapter::AdapterManager;
using apollo::localization::LocalizationEstimate;

DpStSpeedOptimizer::DpStSpeedOptimizer(const std::string& name)
    : SpeedOptimizer(name) {}

bool DpStSpeedOptimizer::Init() {
  if (!common::util::GetProtoFromFile(FLAGS_st_boundary_config_file,
                                      &st_boundary_config_)) {
    AERROR << "failed to load config file " << FLAGS_st_boundary_config_file;
    return false;
  }
  if (!common::util::GetProtoFromFile(FLAGS_dp_st_speed_config_file,
                                      &dp_st_speed_config_)) {
    AERROR << "failed to load config file " << FLAGS_dp_st_speed_config_file;
    return false;
  }
  is_init_ = true;
  return true;
}

Status DpStSpeedOptimizer::Process(const PathData& path_data,
                                   const common::TrajectoryPoint& init_point,
                                   const ReferenceLine& reference_line,
                                   PathDecision* const path_decision,
                                   SpeedData* const speed_data) {
  if (!is_init_) {
    AERROR << "Please call Init() before process DpStSpeedOptimizer.";
    return Status(ErrorCode::PLANNING_ERROR, "Not inited.");
  }

  StBoundaryMapper boundary_mapper(st_boundary_config_, reference_line,
                                   path_data,
                                   dp_st_speed_config_.total_path_length(),
                                   dp_st_speed_config_.total_time());

  // step 1 get boundaries
  std::vector<StGraphBoundary> boundaries;
  if (!boundary_mapper.GetGraphBoundary(*path_decision, &boundaries).ok()) {
    const std::string msg =
        "Mapping obstacle for dp st speed optimizer failed.";
    AERROR << msg;
    return Status(ErrorCode::PLANNING_ERROR, msg);
  }

  // step 2 perform graph search
  SpeedLimit speed_limit;
  if (!boundary_mapper.GetSpeedLimits(&speed_limit).ok()) {
    const std::string msg =
        "Getting speed limits for dp st speed optimizer failed!";
    AERROR << msg;
    return Status(ErrorCode::PLANNING_ERROR, msg);
  }

  const double path_length = path_data.discretized_path().length();
  StGraphData st_graph_data(boundaries, init_point, speed_limit, path_length);

  DpStGraph st_graph(dp_st_speed_config_, path_data);
  if (!st_graph.Search(st_graph_data, path_decision, speed_data).ok()) {
    const std::string msg = "Failed to search graph with dynamic programming.";
    AERROR << msg;
    return Status(ErrorCode::PLANNING_ERROR, msg);
  }
  return Status::OK();
}

}  // namespace planning
}  // namespace apollo
