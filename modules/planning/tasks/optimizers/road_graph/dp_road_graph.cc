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
 * @file dp_road_graph.cc
 **/

#include "modules/planning/tasks/optimizers/road_graph/dp_road_graph.h"

#include "cyber/task/task.h"

#include "modules/common/proto/error_code.pb.h"
#include "modules/planning/proto/planning_internal.pb.h"
#include "modules/planning/proto/planning_status.pb.h"

#include "cyber/common/log.h"
#include "modules/common/configs/vehicle_config_helper.h"
#include "modules/common/math/cartesian_frenet_conversion.h"
#include "modules/common/util/util.h"
#include "modules/map/hdmap/hdmap_util.h"
#include "modules/planning/common/path/frenet_frame_path.h"
#include "modules/planning/common/planning_context.h"
#include "modules/planning/common/planning_gflags.h"
#include "modules/planning/math/curve1d/quintic_polynomial_curve1d.h"

namespace apollo {
namespace planning {

DpRoadGraph::DpRoadGraph(const DpPolyPathConfig &config,
                         const ReferenceLineInfo &reference_line_info,
                         const SpeedData &speed_data)
    : config_(config),
      reference_line_info_(reference_line_info),
      reference_line_(reference_line_info.reference_line()),
      speed_data_(speed_data) {}

bool DpRoadGraph::FindPathTunnel(const common::TrajectoryPoint &init_point,
                                 const std::vector<const Obstacle *> &obstacles,
                                 PathData *const path_data) {
  CHECK_NOTNULL(path_data);

  init_point_ = init_point;
  if (!reference_line_.XYToSL(init_point_.path_point(), &init_sl_point_)) {
    AERROR << "Fail to create init_sl_point from : "
           << init_point.DebugString();
    return false;
  }
  init_frenet_frame_point_ =
      reference_line_.GetFrenetPoint(init_point_.path_point());

  waypoint_sampler_->Init(&reference_line_info_, init_sl_point_,
                          init_frenet_frame_point_);
  waypoint_sampler_->SetDebugLogger(planning_debug_);

  std::vector<DpRoadGraphNode> min_cost_path;
  if (!GenerateMinCostPath(obstacles, &min_cost_path)) {
    AERROR << "Fail to generate graph!";
    return false;
  }
  std::vector<common::FrenetFramePoint> frenet_path;
  double accumulated_s = min_cost_path.front().sl_point.s();
  const double path_resolution = config_.path_resolution();

  for (size_t i = 1; i < min_cost_path.size(); ++i) {
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
  path_data->SetReferenceLine(&reference_line_);
  path_data->SetFrenetPath(FrenetFramePath(std::move(frenet_path)));
  return true;
}

bool DpRoadGraph::GenerateMinCostPath(
    const std::vector<const Obstacle *> &obstacles,
    std::vector<DpRoadGraphNode> *min_cost_path) {
  ACHECK(min_cost_path != nullptr);

  std::vector<std::vector<common::SLPoint>> path_waypoints;
  if (!waypoint_sampler_->SamplePathWaypoints(init_point_, &path_waypoints) ||
      path_waypoints.size() < 1) {
    AERROR << "Fail to sample path waypoints! reference_line_length = "
           << reference_line_.Length();
    return false;
  }
  const auto &vehicle_config =
      common::VehicleConfigHelper::Instance()->GetConfig();

  TrajectoryCost trajectory_cost(
      config_, reference_line_, reference_line_info_.IsChangeLanePath(),
      obstacles, vehicle_config.vehicle_param(), speed_data_, init_sl_point_,
      reference_line_info_.AdcSlBoundary());

  std::list<std::list<DpRoadGraphNode>> graph_nodes;

  // find one point from first row
  const auto &first_row = path_waypoints.front();
  size_t nearest_i = 0;
  for (size_t i = 1; i < first_row.size(); ++i) {
    if (std::fabs(first_row[i].l() - init_sl_point_.l()) <
        std::fabs(first_row[nearest_i].l() - init_sl_point_.l())) {
      nearest_i = i;
    }
  }
  graph_nodes.emplace_back();
  graph_nodes.back().emplace_back(first_row[nearest_i], nullptr,
                                  ComparableCost());
  auto &front = graph_nodes.front().front();
  size_t total_level = path_waypoints.size();

  for (size_t level = 1; level < path_waypoints.size(); ++level) {
    const auto &prev_dp_nodes = graph_nodes.back();
    const auto &level_points = path_waypoints[level];

    graph_nodes.emplace_back();
    std::vector<std::future<void>> results;

    for (size_t i = 0; i < level_points.size(); ++i) {
      const auto &cur_point = level_points[i];

      graph_nodes.back().emplace_back(cur_point, nullptr);

      auto msg = std::make_shared<RoadGraphMessage>(
          prev_dp_nodes, level, total_level, &trajectory_cost, &front,
          &(graph_nodes.back().back()));

      if (FLAGS_enable_multi_thread_in_dp_poly_path) {
        results.emplace_back(cyber::Async(&DpRoadGraph::UpdateNode, this, msg));
      } else {
        UpdateNode(msg);
      }
    }
    if (FLAGS_enable_multi_thread_in_dp_poly_path) {
      for (auto &result : results) {
        result.get();
      }
    }
  }

  // find best path
  DpRoadGraphNode fake_head;
  for (const auto &cur_dp_node : graph_nodes.back()) {
    fake_head.UpdateCost(&cur_dp_node, cur_dp_node.min_cost_curve,
                         cur_dp_node.min_cost);
  }

  const auto *min_cost_node = &fake_head;
  while (min_cost_node->min_cost_prev_node) {
    min_cost_node = min_cost_node->min_cost_prev_node;
    min_cost_path->push_back(*min_cost_node);
  }
  if (min_cost_node != &graph_nodes.front().front()) {
    return false;
  }

  std::reverse(min_cost_path->begin(), min_cost_path->end());

  for (const auto &node : *min_cost_path) {
    ADEBUG << "min_cost_path: " << node.sl_point.ShortDebugString();
    planning_debug_->mutable_planning_data()
        ->mutable_dp_poly_graph()
        ->add_min_cost_point()
        ->CopyFrom(node.sl_point);
  }
  return true;
}

void DpRoadGraph::UpdateNode(const std::shared_ptr<RoadGraphMessage> &msg) {
  CHECK_NOTNULL(msg);
  CHECK_NOTNULL(msg->trajectory_cost);
  CHECK_NOTNULL(msg->front);
  CHECK_NOTNULL(msg->cur_node);
  for (const auto &prev_dp_node : msg->prev_nodes) {
    const auto &prev_sl_point = prev_dp_node.sl_point;
    const auto &cur_point = msg->cur_node->sl_point;
    double init_dl = 0.0;
    double init_ddl = 0.0;
    if (msg->level == 1) {
      init_dl = init_frenet_frame_point_.dl();
      init_ddl = init_frenet_frame_point_.ddl();
    }
    QuinticPolynomialCurve1d curve(prev_sl_point.l(), init_dl, init_ddl,
                                   cur_point.l(), 0.0, 0.0,
                                   cur_point.s() - prev_sl_point.s());

    if (!IsValidCurve(curve)) {
      continue;
    }
    const auto cost =
        msg->trajectory_cost->Calculate(curve, prev_sl_point.s(), cur_point.s(),
                                        msg->level, msg->total_level) +
        prev_dp_node.min_cost;

    msg->cur_node->UpdateCost(&prev_dp_node, curve, cost);
  }

  // try to connect the current point with the first point directly
  if (reference_line_info_.IsChangeLanePath() && msg->level >= 2) {
    const double init_dl = init_frenet_frame_point_.dl();
    const double init_ddl = init_frenet_frame_point_.ddl();
    QuinticPolynomialCurve1d curve(
        init_sl_point_.l(), init_dl, init_ddl, msg->cur_node->sl_point.l(), 0.0,
        0.0, msg->cur_node->sl_point.s() - init_sl_point_.s());
    if (!IsValidCurve(curve)) {
      return;
    }
    const auto cost = msg->trajectory_cost->Calculate(
        curve, init_sl_point_.s(), msg->cur_node->sl_point.s(), msg->level,
        msg->total_level);
    msg->cur_node->UpdateCost(msg->front, curve, cost);
  }
}

bool DpRoadGraph::IsValidCurve(const QuinticPolynomialCurve1d &curve) const {
  static constexpr double kMaxLateralDistance = 20.0;
  for (double s = 0.0; s < curve.ParamLength(); s += 2.0) {
    const double l = curve.Evaluate(0, s);
    if (std::fabs(l) > kMaxLateralDistance) {
      return false;
    }
  }
  return true;
}

void DpRoadGraph::GetCurveCost(TrajectoryCost trajectory_cost,
                               const QuinticPolynomialCurve1d &curve,
                               const double start_s, const double end_s,
                               const uint32_t curr_level,
                               const uint32_t total_level,
                               ComparableCost *cost) {
  *cost =
      trajectory_cost.Calculate(curve, start_s, end_s, curr_level, total_level);
}

}  // namespace planning
}  // namespace apollo
