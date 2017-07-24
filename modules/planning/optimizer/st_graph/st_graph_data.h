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
 * @file: st_map_data.h
 * @brief: data with map info and obstacle info
 **/

#ifndef MODULES_PLANNING_OPTIMIZER_ST_GRAPH_ST_GRAPH_DATA_H_
#define MODULES_PLANNING_OPTIMIZER_ST_GRAPH_ST_GRAPH_DATA_H_

#include <vector>

#include "modules/planning/optimizer/st_graph/st_graph_boundary.h"

#include "modules/common/proto/path_point.pb.h"

namespace apollo {
namespace planning {

class STGraphData {
 public:
  STGraphData() = default;

  explicit STGraphData(const std::vector<STGraphBoundary>& obs_boundary,
                       const apollo::common::TrajectoryPoint& init_point,
                       const double speed_limit, const double path_data_length);

  const std::vector<STGraphBoundary>& obs_boundary() const;

  const apollo::common::TrajectoryPoint& init_point() const;

  double speed_limit() const;

  double path_data_length() const;

  void set_speed_limit(const double speed_limit);

  void set_obs_boundary(const std::vector<STGraphBoundary>& obs_boundary);

  void set_init_point(const apollo::common::TrajectoryPoint& init_point);

  void set_path_data_length(const double path_data_length);

 private:
  std::vector<STGraphBoundary> _obs_boundary;
  apollo::common::TrajectoryPoint _init_point;

  double _speed_limit = 0.0;
  double _path_data_length = 0.0;
};

}  // namespace planning
}  // namespace apollo

#endif  // MODULES_PLANNING_OPTIMIZER_ST_GRAPH_ST_GRAPH_DATA_H_
