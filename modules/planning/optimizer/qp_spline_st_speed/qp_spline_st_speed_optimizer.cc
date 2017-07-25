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

#include "modules/planning/optimizer/qp_spline_st_speed/qp_spline_st_boundary_mapper.h"
#include "modules/planning/optimizer/qp_spline_st_speed/qp_spline_st_graph.h"
#include "modules/planning/optimizer/st_graph/st_graph_data.h"

namespace apollo {
namespace planning {

using Status = apollo::common::Status;
using ErrorCode = apollo::common::ErrorCode;
using TrajectoryPoint = apollo::common::TrajectoryPoint;

QpSplineStSpeedOptimizer::QpSplineStSpeedOptimizer(
    const std::string& name,
    const QpSplineStSpeedConfig& qp_spline_st_speed_config,
    const apollo::localization::Pose& pose, const apollo::hdmap::HDMap& hdmap,
    const apollo::common::config::VehicleParam& veh_param)
    : SpeedOptimizer(name),
      _qp_spline_st_speed_config(qp_spline_st_speed_config),
      _pose(pose),
      _hdmap(hdmap),
      veh_param_(veh_param) {}

Status QpSplineStSpeedOptimizer::process(const PathData& path_data,
                                         const TrajectoryPoint& init_point,
                                         DecisionData* const decision_data,
                                         SpeedData* const speed_data) const {
  double total_length = std::min(_qp_spline_st_speed_config.total_path_length(),
                                 path_data.path().param_length());

  // TODO: load boundary mapper and st graph configuration
  StBoundaryConfig st_boundary_config;

  // step 1 get boundaries
  QpSplineStBoundaryMapper st_mapper(st_boundary_config, veh_param_);
  std::vector<StGraphBoundary> boundaries;
  if (st_mapper.get_graph_boundary(
          init_point, *decision_data, path_data,
          _qp_spline_st_speed_config.total_path_length(),
          _qp_spline_st_speed_config.total_time(),
          &boundaries) != Status::OK()) {
    return Status(ErrorCode::PLANNING_ERROR_FAILED,
                  "Mapping obstacle for dp st speed optimizer failed!");
  }

  SpeedLimit speed_limits;
  if (st_mapper.get_speed_limits(_pose, _hdmap, path_data, total_length,
                                 _qp_spline_st_speed_config.total_time(),
                                 _qp_spline_st_speed_config.max_speed(),
                                 &speed_limits) != Status::OK()) {
    return Status(ErrorCode::PLANNING_ERROR_FAILED,
                  "Mapping obstacle for dp st speed optimizer failed!");
  }

  // step 2 perform graph search
  QpSplineStGraph st_graph(_qp_spline_st_speed_config, veh_param_);

  StGraphData st_graph_data(boundaries, init_point, speed_limits,
                            path_data.path().param_length());
  if (st_graph.search(st_graph_data, path_data, speed_data) != Status::OK()) {
    return Status(ErrorCode::PLANNING_ERROR_FAILED,
                  "Failed to search graph with dynamic programming!");
  }
  return Status::OK();
}

}  // namespace planning
}  // namespace apollo
