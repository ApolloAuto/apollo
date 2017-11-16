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

#include "modules/planning/tasks/dp_poly_path/dp_road_graph.h"

#include <algorithm>
#include <limits>
#include <string>
#include <unordered_map>
#include <utility>

#include "modules/common/proto/error_code.pb.h"
#include "modules/common/proto/pnc_point.pb.h"

#include "modules/common/configs/vehicle_config_helper.h"
#include "modules/common/log.h"
#include "modules/common/util/util.h"
#include "modules/planning/common/path/frenet_frame_path.h"
#include "modules/planning/common/planning_gflags.h"
#include "modules/planning/math/curve1d/quintic_polynomial_curve1d.h"
#include "modules/planning/tasks/dp_poly_path/trajectory_cost.h"

namespace apollo {
namespace planning {

using apollo::common::ErrorCode;
using apollo::common::Status;

DPRoadGraph::DPRoadGraph(const DpPolyPathConfig &config,
                         const ReferenceLineInfo &reference_line_info,
                         const SpeedData &speed_data)
    : config_(config),
      reference_line_info_(reference_line_info),
      reference_line_(reference_line_info.reference_line()),
      speed_data_(speed_data) {}

bool DPRoadGraph::FindPathTunnel(
    const common::TrajectoryPoint &init_point,
    const std::vector<const PathObstacle *> &obstacles,
    PathData *const path_data) {
  CHECK_NOTNULL(path_data);
  init_point_ = init_point;
  if (!reference_line_.XYToSL(
          {init_point_.path_point().x(), init_point_.path_point().y()},
          &init_sl_point_)) {
    AERROR << "Fail to create init_sl_point from : "
           << init_point.DebugString();
    return false;
  }
  std::vector<DPRoadGraphNode> min_cost_path;
  if (!GenerateMinCostPath(obstacles, &min_cost_path)) {
    AERROR << "Fail to generate graph!";
    return false;
  }
  std::vector<common::FrenetFramePoint> frenet_path;
  double accumulated_s = init_sl_point_.s();
  const double path_resolution = config_.path_resolution();

  for (std::size_t i = 1; i < min_cost_path.size(); ++i) {
    const auto &prev_node = min_cost_path[i - 1];
    const auto &cur_node = min_cost_path[i];

    const double path_length = cur_node.sl_point.s() - prev_node.sl_point.s();
    double current_s = 0.0;
    const auto &curve = cur_node.min_cost_curve;
    while (current_s + path_resolution / 2.0 < path_length) {
      const double l = curve.Evaluate(0, current_s);
      const double dl = curve.Evaluate(1, current_s);
      const double ddl = curve.Evaluate(2, current_s);
      common::FrenetFramePoint frenet_frame_point;
      frenet_frame_point.set_s(accumulated_s + current_s);
      frenet_frame_point.set_l(l);
      frenet_frame_point.set_dl(dl);
      frenet_frame_point.set_ddl(ddl);
      frenet_path.push_back(std::move(frenet_frame_point));
      current_s += path_resolution;
    }
    if (i == min_cost_path.size() - 1) {
      accumulated_s += current_s;
    } else {
      accumulated_s += path_length;
    }
  }
  FrenetFramePath tunnel(frenet_path);
  path_data->SetReferenceLine(&reference_line_);
  path_data->SetFrenetPath(tunnel);
  return true;
}

bool DPRoadGraph::GenerateMinCostPath(
    const std::vector<const PathObstacle *> &obstacles,
    std::vector<DPRoadGraphNode> *min_cost_path) {
  CHECK(min_cost_path != nullptr);

  std::vector<std::vector<common::SLPoint>> path_waypoints;
  if (!SamplePathWaypoints(init_point_, &path_waypoints)) {
    AERROR << "Fail to sample path waypoints!";
    return false;
  }
  path_waypoints.insert(path_waypoints.begin(),
                        std::vector<common::SLPoint>{init_sl_point_});

  const auto &vehicle_config =
      common::VehicleConfigHelper::instance()->GetConfig();

  TrajectoryCost trajectory_cost(config_, reference_line_, obstacles,
                                 vehicle_config.vehicle_param(), speed_data_);

  std::vector<std::vector<DPRoadGraphNode>> graph_nodes(path_waypoints.size());
  graph_nodes[0].emplace_back(init_sl_point_, nullptr, 0.0);

  for (std::size_t level = 1; level < path_waypoints.size(); ++level) {
    const auto &prev_dp_nodes = graph_nodes[level - 1];
    const auto &level_points = path_waypoints[level];
    for (const auto &cur_point : level_points) {
      graph_nodes[level].emplace_back(cur_point, nullptr);
      auto &cur_node = graph_nodes[level].back();
      for (const auto &prev_dp_node : prev_dp_nodes) {
        const auto &prev_sl_point = prev_dp_node.sl_point;
        QuinticPolynomialCurve1d curve(prev_sl_point.l(), 0.0, 0.0,
                                       cur_point.l(), 0.0, 0.0,
                                       cur_point.s() - prev_sl_point.s());
        const double cost =
            trajectory_cost.Calculate(curve, prev_sl_point.s(), cur_point.s()) +
            prev_dp_node.min_cost;
        cur_node.UpdateCost(&prev_dp_node, curve, cost);
      }
    }
  }

  // find best path
  DPRoadGraphNode fake_head;
  for (const auto &cur_dp_node : graph_nodes.back()) {
    fake_head.UpdateCost(&cur_dp_node, cur_dp_node.min_cost_curve,
                         cur_dp_node.min_cost);
  }

  const auto *min_cost_node = &fake_head;
  while (min_cost_node->min_cost_prev_node) {
    min_cost_node = min_cost_node->min_cost_prev_node;
    min_cost_path->push_back(*min_cost_node);
  }
  std::reverse(min_cost_path->begin(), min_cost_path->end());
  return true;
}

bool DPRoadGraph::SamplePathWaypoints(
    const common::TrajectoryPoint &init_point,
    std::vector<std::vector<common::SLPoint>> *const points) {
  constexpr double kSamplePointLookForwardTimeChangeLane = 3.0;
  constexpr double kSamplePointLookForwardTimeNoChangeLane = 1.0;
  CHECK(points != nullptr);

  const double reference_line_length = reference_line_.Length();

  double level_distance = 0.0;
  if (reference_line_info_.IsChangeLanePath()) {
    level_distance =
        std::max(config_.step_length_min(),
                 init_point.v() * kSamplePointLookForwardTimeChangeLane);
  } else {
    level_distance = common::math::Clamp(
        init_point.v() * kSamplePointLookForwardTimeNoChangeLane,
        config_.step_length_min(), config_.step_length_max());
  }
  double accumulated_s = init_sl_point_.s();
  for (std::size_t i = 0; accumulated_s < reference_line_length; ++i) {
    std::vector<common::SLPoint> level_points;
    accumulated_s += level_distance;
    const double s = std::fmin(accumulated_s, reference_line_length);

    double left_width = 0.0;
    double right_width = 0.0;
    reference_line_.GetLaneWidth(s, &left_width, &right_width);
    std::vector<double> sample_l;
    common::util::uniform_slice(-right_width, left_width,
                                config_.sample_points_num_each_level() - 1,
                                &sample_l);
    for (double l : sample_l) {
      level_points.emplace_back(common::util::MakeSLPoint(s, l));
    }
    if (!level_points.empty()) {
      points->emplace_back(level_points);
    }
  }
  return true;
}

}  // namespace planning
}  // namespace apollo
