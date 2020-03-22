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

#pragma once

#include "modules/common/proto/pnc_point.pb.h"
#include "modules/planning/proto/dp_poly_path_config.pb.h"

#include "modules/common/status/status.h"
#include "modules/planning/common/obstacle.h"
#include "modules/planning/common/path/path_data.h"
#include "modules/planning/common/path_decision.h"
#include "modules/planning/common/reference_line_info.h"
#include "modules/planning/common/speed/speed_data.h"
#include "modules/planning/common/trajectory/discretized_trajectory.h"
#include "modules/planning/math/curve1d/quintic_polynomial_curve1d.h"
#include "modules/planning/reference_line/reference_point.h"
#include "modules/planning/tasks/optimizers/road_graph/trajectory_cost.h"
#include "modules/planning/tasks/optimizers/road_graph/waypoint_sampler.h"

namespace apollo {
namespace planning {

class DpRoadGraph {
 public:
  DpRoadGraph(const DpPolyPathConfig &config,
              const ReferenceLineInfo &reference_line_info,
              const SpeedData &speed_data);

  ~DpRoadGraph() = default;

  bool FindPathTunnel(const common::TrajectoryPoint &init_point,
                      const std::vector<const Obstacle *> &obstacles,
                      PathData *const path_data);

  void SetDebugLogger(apollo::planning_internal::Debug *debug) {
    planning_debug_ = debug;
  }

  void SetWaypointSampler(WaypointSampler *waypoint_sampler) {
    waypoint_sampler_.reset(waypoint_sampler);
  }

 private:
  /**
   * an private inner struct for the dp algorithm
   */
  struct DpRoadGraphNode {
   public:
    DpRoadGraphNode() = default;

    DpRoadGraphNode(const common::SLPoint point_sl,
                    const DpRoadGraphNode *node_prev)
        : sl_point(point_sl), min_cost_prev_node(node_prev) {}

    DpRoadGraphNode(const common::SLPoint point_sl,
                    const DpRoadGraphNode *node_prev,
                    const ComparableCost &cost)
        : sl_point(point_sl), min_cost_prev_node(node_prev), min_cost(cost) {}

    void UpdateCost(const DpRoadGraphNode *node_prev,
                    const QuinticPolynomialCurve1d &curve,
                    const ComparableCost &cost) {
      if (cost <= min_cost) {
        min_cost = cost;
        min_cost_prev_node = node_prev;
        min_cost_curve = curve;
      }
    }

    common::SLPoint sl_point;
    const DpRoadGraphNode *min_cost_prev_node = nullptr;
    ComparableCost min_cost = {true, true, true,
                               std::numeric_limits<double>::infinity(),
                               std::numeric_limits<double>::infinity()};
    QuinticPolynomialCurve1d min_cost_curve;
  };
  bool GenerateMinCostPath(const std::vector<const Obstacle *> &obstacles,
                           std::vector<DpRoadGraphNode> *min_cost_path);

  bool IsValidCurve(const QuinticPolynomialCurve1d &curve) const;

  void GetCurveCost(TrajectoryCost trajectory_cost,
                    const QuinticPolynomialCurve1d &curve, const double start_s,
                    const double end_s, const uint32_t curr_level,
                    const uint32_t total_level, ComparableCost *cost);

  struct RoadGraphMessage {
    RoadGraphMessage(const std::list<DpRoadGraphNode> &_prev_nodes,
                     const uint32_t _level, const uint32_t _total_level,
                     TrajectoryCost *_trajectory_cost, DpRoadGraphNode *_front,
                     DpRoadGraphNode *_cur_node)
        : prev_nodes(_prev_nodes),
          level(_level),
          total_level(_total_level),
          trajectory_cost(_trajectory_cost),
          front(_front),
          cur_node(_cur_node) {}
    const std::list<DpRoadGraphNode> &prev_nodes;
    const uint32_t level;
    const uint32_t total_level;
    TrajectoryCost *trajectory_cost = nullptr;
    DpRoadGraphNode *front = nullptr;
    DpRoadGraphNode *cur_node = nullptr;
  };
  void UpdateNode(const std::shared_ptr<RoadGraphMessage> &msg);

 private:
  DpPolyPathConfig config_;
  common::TrajectoryPoint init_point_;
  const ReferenceLineInfo &reference_line_info_;
  const ReferenceLine &reference_line_;
  SpeedData speed_data_;
  common::SLPoint init_sl_point_;
  common::FrenetFramePoint init_frenet_frame_point_;
  apollo::planning_internal::Debug *planning_debug_ = nullptr;

  std::unique_ptr<WaypointSampler> waypoint_sampler_;
};

}  // namespace planning
}  // namespace apollo
