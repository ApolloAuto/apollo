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
 * @file: st_graph_data.h
 * @brief: data with map info and obstacle info
 **/

#ifndef MODULES_PLANNING_TASKS_ST_GRAPH_ST_GRAPH_DATA_H_
#define MODULES_PLANNING_TASKS_ST_GRAPH_ST_GRAPH_DATA_H_

#include <vector>

#include "modules/common/proto/pnc_point.pb.h"

#include "modules/planning/common/speed/st_boundary.h"
#include "modules/planning/common/speed_limit.h"

namespace apollo {
namespace planning {

class StGraphData {
 public:
  StGraphData(const std::vector<const StBoundary*>& st_boundaries,
              const apollo::common::TrajectoryPoint& init_point,
              const SpeedLimit& speed_limit, const double path_data_length);
  StGraphData() = default;

  const std::vector<const StBoundary*>& st_boundaries() const;

  const apollo::common::TrajectoryPoint& init_point() const;

  const SpeedLimit& speed_limit() const;

  double path_data_length() const;

 private:
  std::vector<const StBoundary*> st_boundaries_;
  apollo::common::TrajectoryPoint init_point_;

  SpeedLimit speed_limit_;
  double path_data_length_ = 0.0;
};

}  // namespace planning
}  // namespace apollo

#endif  // MODULES_PLANNING_TASKS_ST_GRAPH_ST_GRAPH_DATA_H_
