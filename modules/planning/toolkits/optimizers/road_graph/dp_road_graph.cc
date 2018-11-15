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

#include "modules/planning/toolkits/optimizers/road_graph/dp_road_graph.h"

#include <algorithm>
#include <utility>

#include "modules/common/proto/error_code.pb.h"
#include "modules/planning/proto/planning_internal.pb.h"
#include "modules/planning/proto/planning_status.pb.h"

#include "modules/common/configs/vehicle_config_helper.h"
#include "modules/common/log.h"
#include "modules/common/math/cartesian_frenet_conversion.h"
#include "modules/common/util/thread_pool.h"
#include "modules/common/util/util.h"
#include "modules/map/hdmap/hdmap_util.h"
#include "modules/planning/common/path/frenet_frame_path.h"
#include "modules/planning/common/planning_context.h"
#include "modules/planning/common/planning_gflags.h"
#include "modules/planning/math/curve1d/quintic_polynomial_curve1d.h"

namespace apollo {
namespace planning {

using apollo::common::ErrorCode;
using apollo::common::SLPoint;
using apollo::common::Status;
using apollo::common::math::CartesianFrenetConverter;
using apollo::common::util::MakeSLPoint;
using apollo::common::util::ThreadPool;

DpRoadGraph::DpRoadGraph(const DpPolyPathConfig &config,
                         const ReferenceLineInfo &reference_line_info,
                         const SpeedData &speed_data)
    : config_(config),
      reference_line_info_(reference_line_info),
      reference_line_(reference_line_info.reference_line()),
      speed_data_(speed_data) {}

bool DpRoadGraph::FindPathTunnel(
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

  if (!CalculateFrenetPoint(init_point_, &init_frenet_frame_point_)) {
    AERROR << "Fail to create init_frenet_frame_point_ from : "
           << init_point_.DebugString();
    return false;
  }

  waypoint_sampler_->Init(&reference_line_info_, init_sl_point_,
                          init_frenet_frame_point_);
  waypoint_sampler_->SetDebugLogger(planning_debug_);

  std::vector<DpRoadGraphNode> min_cost_path;
  if (!GenerateMinCostPath(obstacles, &min_cost_path)) {
    AERROR << "Fail to generate graph!";
    return false;
  }
  std::vector<common::FrenetFramePoint> frenet_path;
  float accumulated_s = init_sl_point_.s();
  const float path_resolution = config_.path_resolution();

  for (std::size_t i = 1; i < min_cost_path.size(); ++i) {
    const auto &prev_node = min_cost_path[i - 1];
    const auto &cur_node = min_cost_path[i];

    const float path_length = cur_node.sl_point.s() - prev_node.sl_point.s();
    float current_s = 0.0;
    const auto &curve = cur_node.min_cost_curve;
    while (current_s + path_resolution / 2.0 < path_length) {
      const float l = curve.Evaluate(0, current_s);
      const float dl = curve.Evaluate(1, current_s);
      const float ddl = curve.Evaluate(2, current_s);
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

bool DpRoadGraph::GenerateMinCostPath(
    const std::vector<const PathObstacle *> &obstacles,
    std::vector<DpRoadGraphNode> *min_cost_path) {
  CHECK(min_cost_path != nullptr);

  std::vector<std::vector<common::SLPoint>> path_waypoints;
  if (!waypoint_sampler_->SamplePathWaypoints(init_point_, &path_waypoints) ||
      path_waypoints.size() < 1) {
    AERROR << "Fail to sample path waypoints! reference_line_length = "
           << reference_line_.Length();
    return false;
  }
  path_waypoints.insert(path_waypoints.begin(),
                        std::vector<common::SLPoint>{init_sl_point_});
  const auto &vehicle_config =
      common::VehicleConfigHelper::instance()->GetConfig();

  TrajectoryCost trajectory_cost(
      config_, reference_line_, reference_line_info_.IsChangeLanePath(),
      obstacles, vehicle_config.vehicle_param(), speed_data_, init_sl_point_);

  std::list<std::list<DpRoadGraphNode>> graph_nodes;
  graph_nodes.emplace_back();
  graph_nodes.back().emplace_back(init_sl_point_, nullptr, ComparableCost());
  auto &front = graph_nodes.front().front();
  size_t total_level = path_waypoints.size();

  for (std::size_t level = 1; level < path_waypoints.size(); ++level) {
    const auto &prev_dp_nodes = graph_nodes.back();
    const auto &level_points = path_waypoints[level];

    graph_nodes.emplace_back();

    std::vector<std::future<void>> futures;

    for (size_t i = 0; i < level_points.size(); ++i) {
      const auto &cur_point = level_points[i];

      graph_nodes.back().emplace_back(cur_point, nullptr);
      auto &cur_node = graph_nodes.back().back();
      if (FLAGS_enable_multi_thread_in_dp_poly_path) {
        futures.push_back(ThreadPool::pool()->push(std::bind(
            &DpRoadGraph::UpdateNode, this, std::ref(prev_dp_nodes), level,
            total_level, &trajectory_cost, &(front), &(cur_node))));

      } else {
        UpdateNode(prev_dp_nodes, level, total_level, &trajectory_cost, &front,
                   &cur_node);
      }
    }

    for (const auto &f : futures) {
      f.wait();
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

void DpRoadGraph::UpdateNode(const std::list<DpRoadGraphNode> &prev_nodes,
                             const uint32_t level, const uint32_t total_level,
                             TrajectoryCost *trajectory_cost,
                             DpRoadGraphNode *front,
                             DpRoadGraphNode *cur_node) {
  DCHECK_NOTNULL(trajectory_cost);
  DCHECK_NOTNULL(front);
  DCHECK_NOTNULL(cur_node);
  for (const auto &prev_dp_node : prev_nodes) {
    const auto &prev_sl_point = prev_dp_node.sl_point;
    const auto &cur_point = cur_node->sl_point;
    float init_dl = 0.0;
    float init_ddl = 0.0;
    if (level == 1) {
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
        trajectory_cost->Calculate(curve, prev_sl_point.s(), cur_point.s(),
                                   level, total_level) +
        prev_dp_node.min_cost;

    cur_node->UpdateCost(&prev_dp_node, curve, cost);
  }

  // try to connect the current point with the first point directly
  if (level >= 2) {
    const float init_dl = init_frenet_frame_point_.dl();
    const float init_ddl = init_frenet_frame_point_.ddl();
    QuinticPolynomialCurve1d curve(init_sl_point_.l(), init_dl, init_ddl,
                                   cur_node->sl_point.l(), 0.0, 0.0,
                                   cur_node->sl_point.s() - init_sl_point_.s());
    if (!IsValidCurve(curve)) {
      return;
    }
    const auto cost = trajectory_cost->Calculate(
        curve, init_sl_point_.s(), cur_node->sl_point.s(), level, total_level);
    cur_node->UpdateCost(front, curve, cost);
  }
}

bool DpRoadGraph::CalculateFrenetPoint(
    const common::TrajectoryPoint &traj_point,
    common::FrenetFramePoint *const frenet_frame_point) {
  common::SLPoint sl_point;
  if (!reference_line_.XYToSL(
          {traj_point.path_point().x(), traj_point.path_point().y()},
          &sl_point)) {
    return false;
  }
  frenet_frame_point->set_s(sl_point.s());
  frenet_frame_point->set_l(sl_point.l());

  const float theta = traj_point.path_point().theta();
  const float kappa = traj_point.path_point().kappa();
  const float l = frenet_frame_point->l();

  ReferencePoint ref_point;
  ref_point = reference_line_.GetReferencePoint(frenet_frame_point->s());

  const float theta_ref = ref_point.heading();
  const float kappa_ref = ref_point.kappa();
  const float dkappa_ref = ref_point.dkappa();

  const float dl = CartesianFrenetConverter::CalculateLateralDerivative(
      theta_ref, theta, l, kappa_ref);
  const float ddl =
      CartesianFrenetConverter::CalculateSecondOrderLateralDerivative(
          theta_ref, theta, kappa_ref, kappa, dkappa_ref, l);
  frenet_frame_point->set_dl(dl);
  frenet_frame_point->set_ddl(ddl);
  return true;
}

bool DpRoadGraph::IsValidCurve(const QuinticPolynomialCurve1d &curve) const {
  constexpr float kMaxLateralDistance = 20.0;
  for (float s = 0.0; s < curve.ParamLength(); s += 2.0) {
    const float l = curve.Evaluate(0, s);
    if (std::fabs(l) > kMaxLateralDistance) {
      return false;
    }
  }
  return true;
}

void DpRoadGraph::GetCurveCost(TrajectoryCost trajectory_cost,
                               const QuinticPolynomialCurve1d &curve,
                               const float start_s, const float end_s,
                               const uint32_t curr_level,
                               const uint32_t total_level,
                               ComparableCost *cost) {
  *cost =
      trajectory_cost.Calculate(curve, start_s, end_s, curr_level, total_level);
}

}  // namespace planning
}  // namespace apollo
