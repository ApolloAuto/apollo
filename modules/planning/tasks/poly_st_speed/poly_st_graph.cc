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
 * @file
 **/

#include "modules/planning/tasks/poly_st_speed/poly_st_graph.h"

#include <algorithm>
#include <unordered_map>

#include "modules/common/proto/error_code.pb.h"
#include "modules/common/proto/pnc_point.pb.h"
#include "modules/planning/proto/planning_internal.pb.h"

#include "modules/common/configs/vehicle_config_helper.h"
#include "modules/common/log.h"
#include "modules/common/util/util.h"
#include "modules/planning/common/planning_gflags.h"
#include "modules/planning/math/curve1d/quintic_polynomial_curve1d.h"

namespace apollo {
namespace planning {

using apollo::common::ErrorCode;
using apollo::common::Status;

PolyStGraph::PolyStGraph(const PolyStSpeedConfig &config,
                         const ReferenceLineInfo *reference_line_info)
    : config_(config),
      reference_line_info_(reference_line_info),
      reference_line_(reference_line_info->reference_line()) {}

bool PolyStGraph::FindStTunnel(
    const common::TrajectoryPoint &init_point,
    const std::vector<const PathObstacle *> &obstacles,
    SpeedData *const speed_data) {
  CHECK_NOTNULL(speed_data);

  init_point_ = init_point;
  std::vector<PolyStGraphNode> min_cost_path;
  if (!GenerateMinCostSpeedProfile(obstacles, &min_cost_path)) {
    AERROR << "Fail to generate graph!";
    return false;
  }
  // TODO(All): implement this function
  return true;
}

bool PolyStGraph::GenerateMinCostSpeedProfile(
    const std::vector<const PathObstacle *> &obstacles,
    std::vector<PolyStGraphNode> *min_cost_path) {
  CHECK(min_cost_path != nullptr);
  // TODO(All): implement this function
  return true;
}

bool PolyStGraph::SampleStPoints(
    std::vector<std::vector<STPoint>> *const points) {
  CHECK_NOTNULL(points);
  constexpr double start_t = 5.0;
  constexpr double start_s = 0.0;
  for (double t = start_t; t < planning_time_; t += unit_t_) {
    std::vector<STPoint> level_points;
    for (double s = start_s; s < planning_distance_; s += unit_s_) {
      level_points.emplace_back(t, s);
    }
    points->push_back(std::move(level_points));
  }
  return true;
}

}  // namespace planning
}  // namespace apollo
