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
 * @file dp_st_speed_optimizer.cpp
 **/

#include "optimizer/dp_st_speed_optimizer.h"
#include "optimizer/dp_st_speed_optimizer/dp_st_boundary_mapper.h"
#include "optimizer/dp_st_speed_optimizer/dp_st_configuration.h"
#include "optimizer/dp_st_speed_optimizer/dp_st_graph.h"
#include "optimizer/st_graph/st_graph_data.h"

namespace apollo {
namespace planning {

DpStSpeedOptimizer::DpStSpeedOptimizer(
    const std::string& name, const ::boost::property_tree::ptree& ptree)
    : SpeedOptimizer(name) {}

ErrorCode DpStSpeedOptimizer::optimize(
    const DataCenter& data_center, const PathData& path_data,
    const ::adu::planning::TrajectoryPoint& init_point,
    DecisionData* const decision_data, SpeedData* const speed_data) const {
  ::adu::common::config::VehicleParam veh_param =
      data_center.current_frame()->environment().config().vehicle_param();

  // TODO: load boundary mapper and st graph configuration
  STBoundaryConfig st_boundary_config;
  DpStConfiguration dp_st_configuration;

  // step 1 get boundaries
  DPSTBoundaryMapper st_mapper(st_boundary_config, veh_param);
  std::vector<STGraphBoundary> boundaries;

  QUIT_IF(st_mapper.get_graph_boundary(data_center, *decision_data, path_data,
                                       dp_st_configuration.total_path_length(),
                                       dp_st_configuration.total_time(),
                                       &boundaries) != ErrorCode::PLANNING_OK,
          ErrorCode::PLANNING_ERROR_FAILED, Level::ERROR,
          "Mapping obstacle for dp st speed optimizer failed!");

  // step 2 perform graph search
  // TODO: here change the speed limit
  double speed_limit = 20.0;
  STGraphData st_graph_data(boundaries, init_point, speed_limit,
                            path_data.path().param_length());

  DPSTGraph st_graph(dp_st_configuration, veh_param);

  QUIT_IF(st_graph.search(st_graph_data, decision_data, speed_data) !=
              ErrorCode::PLANNING_OK,
          ErrorCode::PLANNING_ERROR_FAILED, Level::ERROR,
          "Failed to search graph with dynamic programming!");

  return ErrorCode::PLANNING_OK;
}

}  // namespace planning
}  // namespace apollo
