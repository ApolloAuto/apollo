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

#include "modules/planning/tasks/st_graph/st_graph_data.h"

namespace apollo {
namespace planning {

using apollo::common::TrajectoryPoint;

StGraphData::StGraphData(const std::vector<const StBoundary*>& st_boundaries,
                         const TrajectoryPoint& init_point,
                         const SpeedLimit& speed_limit,
                         const double path_data_length)
    : st_boundaries_(st_boundaries),
      init_point_(init_point),
      speed_limit_(speed_limit),
      path_data_length_(path_data_length) {}

const std::vector<const StBoundary*>& StGraphData::st_boundaries() const {
  return st_boundaries_;
}

const TrajectoryPoint& StGraphData::init_point() const { return init_point_; }

const SpeedLimit& StGraphData::speed_limit() const { return speed_limit_; }

double StGraphData::path_data_length() const { return path_data_length_; }

}  // namespace planning
}  // namespace apollo
