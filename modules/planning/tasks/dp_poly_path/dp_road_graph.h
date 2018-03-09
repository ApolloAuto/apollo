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
 * @file dp_road_graph.h
 **/

#ifndef MODULES_PLANNING_TASKS_DP_POLY_PATH_DP_ROAD_GRAPH_H_
#define MODULES_PLANNING_TASKS_DP_POLY_PATH_DP_ROAD_GRAPH_H_

#include <limits>
#include <list>
#include <vector>

#include "modules/common/proto/pnc_point.pb.h"
#include "modules/planning/proto/dp_poly_path_config.pb.h"

#include "modules/common/status/status.h"
#include "modules/planning/common/path/path_data.h"
#include "modules/planning/common/path_decision.h"
#include "modules/planning/common/path_obstacle.h"
#include "modules/planning/common/reference_line_info.h"
#include "modules/planning/common/speed/speed_data.h"
#include "modules/planning/common/trajectory/discretized_trajectory.h"
#include "modules/planning/math/curve1d/quintic_polynomial_curve1d.h"
#include "modules/planning/reference_line/reference_point.h"
#include "modules/planning/tasks/dp_poly_path/trajectory_cost.h"

namespace apollo {
namespace planning {

class DPRoadGraph {
 public:
  explicit DPRoadGraph(const DpPolyPathConfig &config,
                       const ReferenceLineInfo &reference_line_info,
                       const SpeedData &speed_data);

  ~DPRoadGraph() = default;

  bool FindPathTunnel(const common::TrajectoryPoint &init_point,
                      const std::vector<const PathObstacle *> &obstacles,
                      PathData *const path_data);

  void SetDebugLogger(apollo::planning_internal::Debug *debug) {
    planning_debug_ = debug;
  }

 private:
  /**
   * an private inner struct for the dp algorithm
   */
  struct DPRoadGraphNode {
   public:
    DPRoadGraphNode() = default;

    DPRoadGraphNode(const common::SLPoint point_sl,
                    const DPRoadGraphNode *node_prev)
        : sl_point(point_sl), min_cost_prev_node(node_prev) {}

    DPRoadGraphNode(const common::SLPoint point_sl,
                    const DPRoadGraphNode *node_prev,
                    const ComparableCost &cost)
        : sl_point(point_sl), min_cost_prev_node(node_prev), min_cost(cost) {}

    void UpdateCost(const DPRoadGraphNode *node_prev,
                    const QuinticPolynomialCurve1d &curve,
                    const ComparableCost &cost) {
      if (cost <= min_cost) {
        min_cost = cost;
        min_cost_prev_node = node_prev;
        min_cost_curve = curve;
      }
    }

    common::SLPoint sl_point;
    const DPRoadGraphNode *min_cost_prev_node = nullptr;
    ComparableCost min_cost = {true, true, true,
                               std::numeric_limits<double>::infinity(),
                               std::numeric_limits<double>::infinity()};
    QuinticPolynomialCurve1d min_cost_curve;
  };

  bool GenerateMinCostPath(const std::vector<const PathObstacle *> &obstacles,
                           std::vector<DPRoadGraphNode> *min_cost_path);

  bool SamplePathWaypoints(
      const common::TrajectoryPoint &init_point,
      std::vector<std::vector<common::SLPoint>> *const points);

  bool CalculateFrenetPoint(const common::TrajectoryPoint &traj_point,
                            common::FrenetFramePoint *const frenet_frame_point);
  bool IsSafeForLaneChange();

  bool IsValidCurve(const QuinticPolynomialCurve1d &curve) const;

  void GetCurveCost(TrajectoryCost trajectory_cost,
                    const QuinticPolynomialCurve1d &curve, const double start_s,
                    const double end_s, const uint32_t curr_level,
                    const uint32_t total_level, ComparableCost *cost);

  void UpdateNode(const std::list<DPRoadGraphNode> &prev_nodes,
                  const uint32_t level, const uint32_t total_level,
                  TrajectoryCost *trajectory_cost, DPRoadGraphNode *front,
                  DPRoadGraphNode *cur_node);
  bool HasSidepass();

 private:
  DpPolyPathConfig config_;
  common::TrajectoryPoint init_point_;
  const ReferenceLineInfo &reference_line_info_;
  const ReferenceLine &reference_line_;
  SpeedData speed_data_;
  common::SLPoint init_sl_point_;
  common::FrenetFramePoint init_frenet_frame_point_;
  apollo::planning_internal::Debug *planning_debug_ = nullptr;

  ObjectSidePass sidepass_;
};

}  // namespace planning
}  // namespace apollo

#endif  // MODULES_PLANNING_TASKS_DP_POLY_PATH_DP_ROAD_GRAPH_H_
