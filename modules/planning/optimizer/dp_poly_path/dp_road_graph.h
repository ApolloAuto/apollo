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

#ifndef MODULES_PLANNING_OPTIMIZER_DP_POLY_PATH_DP_ROAD_GRAPH_H
#define MODULES_PLANNING_OPTIMIZER_DP_POLY_PATH_DP_ROAD_GRAPH_H

#include <vector>

#include "modules/common/proto/path_point.pb.h"

#include "modules/common/status/status.h"
#include "modules/planning/common/decision_data.h"
#include "modules/planning/common/path/path_data.h"
#include "modules/planning/common/speed/speed_data.h"
#include "modules/planning/common/trajectory/discretized_trajectory.h"
#include "modules/planning/optimizer/dp_poly_path/graph_edge.h"
#include "modules/planning/optimizer/dp_poly_path/graph_vertex.h"
#include "modules/planning/proto/dp_poly_path_config.pb.h"
#include "modules/planning/reference_line/reference_line.h"
#include "modules/planning/reference_line/reference_point.h"

namespace apollo {
namespace planning {

class DpRoadGraph {
 public:
  explicit DpRoadGraph(const DpPolyPathConfig &config,
                       const ::apollo::common::TrajectoryPoint &init_point,
                       const SpeedData &speed_data);
  ~DpRoadGraph() = default;
  bool find_tunnel(const ReferenceLine &reference_line,
                   DecisionData *const decision_data,
                   PathData *const path_data);

 private:
  bool init(const ReferenceLine &reference_line);
  ::apollo::common::Status generate_graph(const ReferenceLine &reference_line);
  ::apollo::common::Status find_best_trajectory(
      const ReferenceLine &reference_line, const DecisionData &decision_data,
      std::vector<uint32_t> *const min_cost_edges);
  bool add_vertex(const common::SLPoint &sl_point,
                  const ReferencePoint &reference_point, const uint32_t level);
  bool connect_vertex(const uint32_t start, const uint32_t end);

 private:
  DpPolyPathConfig _config;
  ::apollo::common::TrajectoryPoint _init_point;
  SpeedData _heuristic_speed_data;
  std::vector<GraphVertex> _vertices;
  std::vector<GraphEdge> _edges;
  common::SLPoint _init_sl_point;
};

}  // namespace planning
}  // namespace apollo

#endif  // MODULES_PLANNING_OPTIMIZER_DP_POLY_PATH_DP_ROAD_GRAPH_H
