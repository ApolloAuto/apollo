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
 * @file: st_graph_data.cc
 **/

#include "modules/planning/optimizer/st_graph/st_graph_data.h"

namespace apollo {
namespace planning {

using TrajectoryPoint = apollo::common::TrajectoryPoint;

StGraphData::StGraphData(const std::vector<StGraphBoundary>& obs_boundary,
                         const TrajectoryPoint& init_point,
                         const SpeedLimit& speed_limit,
                         const double path_data_length)
    : _obs_boundary(obs_boundary),
      _init_point(init_point),
      _speed_limit(speed_limit),
      _path_data_length(path_data_length) {}

const std::vector<StGraphBoundary>& StGraphData::obs_boundary() const {
  return _obs_boundary;
}

const TrajectoryPoint& StGraphData::init_point() const { return _init_point; }

const SpeedLimit StGraphData::speed_limit() const { return _speed_limit; }

double StGraphData::path_data_length() const { return _path_data_length; }

void StGraphData::set_speed_limit(const SpeedLimit& speed_limit) {
  _speed_limit = speed_limit;
}

void StGraphData::set_obs_boundary(
    const std::vector<StGraphBoundary>& obs_boundary) {
  _obs_boundary = obs_boundary;
}

void StGraphData::set_init_point(const TrajectoryPoint& init_point) {
  _init_point = init_point;
}

void StGraphData::set_path_data_length(const double path_data_length) {
  _path_data_length = path_data_length;
}

}  // namespace planning
}  // namespace apollo
