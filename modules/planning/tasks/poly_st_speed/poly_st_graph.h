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

#ifndef MODULES_PLANNING_TASKS_POLY_ST_SPEED_POLY_ST_GRAPH_H_
#define MODULES_PLANNING_TASKS_POLY_ST_SPEED_POLY_ST_GRAPH_H_

#include <limits>
#include <string>
#include <utility>
#include <vector>

#include "modules/common/proto/pnc_point.pb.h"
#include "modules/planning/proto/poly_st_speed_config.pb.h"

#include "modules/common/status/status.h"
#include "modules/planning/common/path/path_data.h"
#include "modules/planning/common/path_decision.h"
#include "modules/planning/common/path_obstacle.h"
#include "modules/planning/common/reference_line_info.h"
#include "modules/planning/common/speed/speed_data.h"
#include "modules/planning/common/speed_limit.h"
#include "modules/planning/common/trajectory/discretized_trajectory.h"
#include "modules/planning/math/curve1d/quartic_polynomial_curve1d.h"
#include "modules/planning/reference_line/reference_point.h"

namespace apollo {
namespace planning {

class PolyStGraph {
 public:
  explicit PolyStGraph(const PolyStSpeedConfig &config,
                       const ReferenceLineInfo *reference_line_info,
                       const SpeedLimit &speed_limit);

  ~PolyStGraph() = default;

  bool FindStTunnel(const common::TrajectoryPoint &init_point,
                    const std::vector<const PathObstacle *> &obstacles,
                    SpeedData *const speed_data);

 private:
  struct PolyStGraphNode {
   public:
    PolyStGraphNode() = default;

    PolyStGraphNode(const STPoint &point_st, const double speed,
                    const double accel)
        : st_point(point_st), speed(speed), accel(accel) {}

    STPoint st_point;
    double speed = 0.0;
    double accel = 0.0;
    QuarticPolynomialCurve1d speed_profile;
  };

  bool GenerateMinCostSpeedProfile(
      const std::vector<std::vector<STPoint>> &points,
      const std::vector<const PathObstacle *> &obstacles,
      PolyStGraphNode *const min_cost_node);

  bool SampleStPoints(std::vector<std::vector<STPoint>> *const points);

 private:
  PolyStSpeedConfig config_;
  common::TrajectoryPoint init_point_;
  const ReferenceLineInfo *reference_line_info_ = nullptr;
  const ReferenceLine &reference_line_;
  const SpeedLimit &speed_limit_;

  double unit_t_ = 1.0;
  double unit_s_ = 5.0;
  double planning_distance_ = 100.0;
  double planning_time_ = 6.0;
};

}  // namespace planning
}  // namespace apollo

#endif  // MODULES_PLANNING_TASKS_POLY_ST_SPEED_POLY_ST_GRAPH_H_
