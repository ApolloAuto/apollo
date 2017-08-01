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
 * @file qp_spline_st_speed_optimizer.cc
 **/

#include "modules/planning/optimizer/qp_spline_st_speed/qp_spline_st_speed_optimizer.h"

#include <algorithm>
#include <vector>

#include "modules/common/configs/vehicle_config_helper.h"
#include "modules/common/util/file.h"
#include "modules/common/vehicle_state/vehicle_state.h"
#include "modules/planning/common/data_center.h"
#include "modules/planning/common/planning_gflags.h"
#include "modules/planning/optimizer/qp_spline_st_speed/qp_spline_st_graph.h"
#include "modules/planning/optimizer/st_graph/st_graph_data.h"

namespace apollo {
namespace planning {

using Status = apollo::common::Status;
using ErrorCode = apollo::common::ErrorCode;
using TrajectoryPoint = apollo::common::TrajectoryPoint;

QpSplineStSpeedOptimizer::QpSplineStSpeedOptimizer(const std::string& name)
    : SpeedOptimizer(name) {}

bool QpSplineStSpeedOptimizer::Init() {
  // load boundary mapper
  if (!boundary_mapper_.Init(FLAGS_st_boundary_config_file)) {
    AERROR << "Failed to load config file: " << FLAGS_st_boundary_config_file;
    return false;
  }

  // load qp_spline_st_speed_config_
  if (!common::util::GetProtoFromFile(FLAGS_qp_spline_st_speed_config_file,
                                      &qp_spline_st_speed_config_)) {
    AERROR << "Failed to load config file: "
           << FLAGS_qp_spline_st_speed_config_file;
    return false;
  }
  is_init_ = true;
  return true;
}

Status QpSplineStSpeedOptimizer::Process(const PathData& path_data,
                                         const TrajectoryPoint& init_point,
                                         const ReferenceLine& reference_line,
                                         DecisionData* const decision_data,
                                         SpeedData* const speed_data) {
  if (!is_init_) {
    AERROR << "Please call Init() before Process.";
    return Status(ErrorCode::PLANNING_ERROR, "Not init.");
  }

  double total_length = std::min(qp_spline_st_speed_config_.total_path_length(),
                                 path_data.discretized_path().length());

  // step 1 get boundaries
  std::vector<StGraphBoundary> boundaries;
  if (!boundary_mapper_
           .get_graph_boundary(
               init_point, *decision_data, path_data, reference_line,
               qp_spline_st_speed_config_.total_path_length(),
               qp_spline_st_speed_config_.total_time(), &boundaries)
           .ok()) {
    return Status(ErrorCode::PLANNING_ERROR,
                  "Mapping obstacle for dp st speed optimizer failed!");
  }

  SpeedLimit speed_limits;
  const auto& pose = apollo::common::VehicleState::instance()->pose();
  const auto& hdmap = apollo::planning::DataCenter::instance()->map();
  if (boundary_mapper_.get_speed_limits(pose, hdmap, path_data, total_length,
                                        qp_spline_st_speed_config_.total_time(),
                                        qp_spline_st_speed_config_.max_speed(),
                                        &speed_limits) != Status::OK()) {
    return Status(ErrorCode::PLANNING_ERROR,
                  "Mapping obstacle for dp st speed optimizer failed!");
  }

  // step 2 perform graph search
  const auto& veh_param =
      common::VehicleConfigHelper::GetConfig().vehicle_param();
  QpSplineStGraph st_graph(qp_spline_st_speed_config_, veh_param);

  StGraphData st_graph_data(boundaries, init_point, speed_limits,
                            path_data.discretized_path().length());
  if (st_graph.search(st_graph_data, path_data, speed_data) != Status::OK()) {
    return Status(ErrorCode::PLANNING_ERROR,
                  "Failed to search graph with dynamic programming!");
  }
  return Status::OK();
}

}  // namespace planning
}  // namespace apollo
