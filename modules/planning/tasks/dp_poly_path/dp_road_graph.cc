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
#include "modules/planning/math/frame_conversion/cartesian_frenet_conversion.h"

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

  if (!CalculateFrenetPoint(init_point_, &init_frenet_frame_point_)) {
    AERROR << "Fail to create init_frenet_frame_point_ from : "
           << init_point_.DebugString();
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
  if (!SamplePathWaypoints(init_point_, &path_waypoints) ||
      path_waypoints.size() < 1) {
    AERROR << "Fail to sample path waypoints! reference_line_length = "
           << reference_line_.Length();
    return false;
  }
  path_waypoints.insert(path_waypoints.begin(),
                        std::vector<common::SLPoint>{init_sl_point_});

  for (uint32_t i = 0; i < path_waypoints.size(); ++i) {
    const auto &level_waypoints = path_waypoints.at(i);
    for (uint32_t j = 0; j < level_waypoints.size(); ++j) {
      ADEBUG << "level[" << i << "], "
             << level_waypoints.at(j).ShortDebugString();
    }
  }

  const auto &vehicle_config =
      common::VehicleConfigHelper::instance()->GetConfig();

  TrajectoryCost trajectory_cost(
      config_, reference_line_, reference_line_info_.IsChangeLanePath(),
      obstacles, vehicle_config.vehicle_param(), speed_data_, init_sl_point_);

  std::vector<std::vector<DPRoadGraphNode>> graph_nodes(path_waypoints.size());
  if (graph_nodes.size() < 2) {
    AERROR << "Too few graph_nodes.";
    return false;
  }
  graph_nodes[0].emplace_back(init_sl_point_, nullptr, ComparableCost());

  for (std::size_t level = 1; level < path_waypoints.size(); ++level) {
    const auto &prev_dp_nodes = graph_nodes[level - 1];
    const auto &level_points = path_waypoints[level];
    for (const auto &cur_point : level_points) {
      graph_nodes[level].emplace_back(cur_point, nullptr);
      auto &cur_node = graph_nodes[level].back();
      for (const auto &prev_dp_node : prev_dp_nodes) {
        const auto &prev_sl_point = prev_dp_node.sl_point;
        double init_dl = 0.0;
        double init_ddl = 0.0;
        if (level == 1) {
          init_dl = init_frenet_frame_point_.dl();
          init_ddl = init_frenet_frame_point_.ddl();
        }
        QuinticPolynomialCurve1d curve(prev_sl_point.l(), init_dl, init_ddl,
                                       cur_point.l(), 0.0, 0.0,
                                       cur_point.s() - prev_sl_point.s());
        const auto cost =
            trajectory_cost.Calculate(curve, prev_sl_point.s(), cur_point.s(),
                                      level, path_waypoints.size()) +
            prev_dp_node.min_cost;
        cur_node.UpdateCost(&prev_dp_node, curve, cost);

        // try to connect the current point with the first point directly
        // only do this at lane change
        if (reference_line_info_.IsChangeLanePath() && level >= 2) {
          init_dl = init_frenet_frame_point_.dl();
          init_ddl = init_frenet_frame_point_.ddl();
          QuinticPolynomialCurve1d curve(init_sl_point_.l(), init_dl, init_ddl,
                                         cur_point.l(), 0.0, 0.0,
                                         cur_point.s() - init_sl_point_.s());
          const auto cost = trajectory_cost.Calculate(curve, init_sl_point_.s(),
                                                      cur_point.s(), level,
                                                      path_waypoints.size());

          cur_node.UpdateCost(&(graph_nodes.front().front()), curve, cost);
        }
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

  for (const auto &node : *min_cost_path) {
    ADEBUG << "min_cost_path: " << node.sl_point.ShortDebugString();
  }
  return true;
}

bool DPRoadGraph::SamplePathWaypoints(
    const common::TrajectoryPoint &init_point,
    std::vector<std::vector<common::SLPoint>> *const points) {
  CHECK_NOTNULL(points);

  const double kMinSampleDistance = 40.0;
  const double total_length = std::fmin(
      init_sl_point_.s() + std::fmax(init_point.v() * 8.0, kMinSampleDistance),
      reference_line_.Length());

  constexpr double kSamplePointLookForwardTime = 4.0;
  const double level_distance =
      common::math::Clamp(init_point.v() * kSamplePointLookForwardTime,
                          config_.step_length_min(), config_.step_length_max());
  double accumulated_s = init_sl_point_.s();
  double prev_s = accumulated_s;
  for (std::size_t i = 0; accumulated_s < total_length; ++i) {
    std::vector<common::SLPoint> level_points;
    accumulated_s += level_distance;
    const double s = std::fmin(accumulated_s, total_length);
    constexpr double kMinAllowedSampleStep = 1.0;
    if (std::fabs(s - prev_s) < kMinAllowedSampleStep) {
      continue;
    }
    prev_s = s;

    double left_width = 0.0;
    double right_width = 0.0;
    reference_line_.GetLaneWidth(s, &left_width, &right_width);

    constexpr double kBoundaryBuff = 0.10;
    const auto &vehicle_config =
        common::VehicleConfigHelper::instance()->GetConfig();
    const double half_adc_width = vehicle_config.vehicle_param().width() / 2.0;
    const double eff_right_width = right_width - half_adc_width - kBoundaryBuff;
    const double eff_left_width = left_width - half_adc_width - kBoundaryBuff;

    const double kDeafultUnitL = 0.30;
    const double sample_l_range =
        kDeafultUnitL * (config_.sample_points_num_each_level() - 1);
    double sample_right_boundary =
        std::fmin(-eff_right_width, init_sl_point_.l());
    double sample_left_boundary = std::fmax(eff_left_width, init_sl_point_.l());

    if (reference_line_info_.IsChangeLanePath() &&
        init_sl_point_.l() > eff_left_width) {
      sample_right_boundary =
          std::fmax(sample_right_boundary, init_sl_point_.l() - sample_l_range);
    }
    if (reference_line_info_.IsChangeLanePath() &&
        init_sl_point_.l() < eff_right_width) {
      sample_left_boundary =
          std::fmin(sample_left_boundary, init_sl_point_.l() + sample_l_range);
    }

    std::vector<double> sample_l;
    common::util::uniform_slice(sample_right_boundary, sample_left_boundary,
                                config_.sample_points_num_each_level() - 1,
                                &sample_l);
    for (uint8_t j = 0; j < sample_l.size(); ++j) {
      const double l = sample_l[j];
      common::SLPoint sl;
      if (j % 2 == 0 || total_length - accumulated_s < level_distance) {
        sl = common::util::MakeSLPoint(s, l);
      } else {
        constexpr double kResonateDistance = 2.0;
        sl = common::util::MakeSLPoint(
            std::fmin(total_length, s + kResonateDistance), l);
      }
      level_points.push_back(std::move(sl));
    }
    if (!reference_line_info_.IsChangeLanePath()) {
      level_points.push_back(common::util::MakeSLPoint(s, 0.0));
    }

    if (!level_points.empty()) {
      points->emplace_back(level_points);
    }
  }
  return true;
}

bool DPRoadGraph::CalculateFrenetPoint(
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

  const double theta = traj_point.path_point().theta();
  const double kappa = traj_point.path_point().kappa();
  const double l = frenet_frame_point->l();

  ReferencePoint ref_point;
  ref_point = reference_line_.GetReferencePoint(frenet_frame_point->s());

  const double theta_ref = ref_point.heading();
  const double kappa_ref = ref_point.kappa();
  const double dkappa_ref = ref_point.dkappa();

  const double dl = CartesianFrenetConverter::CalculateLateralDerivative(
      theta_ref, theta, l, kappa_ref);
  const double ddl =
      CartesianFrenetConverter::CalculateSecondOrderLateralDerivative(
          theta_ref, theta, kappa_ref, kappa, dkappa_ref, l);
  frenet_frame_point->set_dl(dl);
  frenet_frame_point->set_ddl(ddl);
  return true;
}

}  // namespace planning
}  // namespace apollo
