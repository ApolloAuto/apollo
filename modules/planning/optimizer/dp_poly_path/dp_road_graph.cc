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

#include "modules/planning/optimizer/dp_poly_path/dp_road_graph.h"

#include <algorithm>
#include <limits>
#include <string>
#include <unordered_map>
#include <utility>

#include "modules/common/configs/vehicle_config_helper.h"
#include "modules/common/log.h"
#include "modules/common/proto/error_code.pb.h"
#include "modules/common/proto/path_point.pb.h"
#include "modules/planning/common/path/frenet_frame_path.h"
#include "modules/planning/math/curve1d/quintic_polynomial_curve1d.h"
#include "modules/planning/math/double.h"
#include "modules/planning/math/sl_analytic_transformation.h"
#include "modules/planning/optimizer/dp_poly_path/path_sampler.h"
#include "modules/planning/optimizer/dp_poly_path/trajectory_cost.h"

namespace apollo {
namespace planning {

using apollo::common::ErrorCode;
using apollo::common::Status;

DpRoadGraph::DpRoadGraph(const DpPolyPathConfig &config,
                         const ::apollo::common::TrajectoryPoint &init_point,
                         const SpeedData &speed_data)
    : _config(config), _init_point(init_point), _speed_data(speed_data) {}

bool DpRoadGraph::find_tunnel(const ReferenceLine &reference_line,
                              DecisionData *const decision_data,
                              PathData *const path_data) {
  CHECK_NOTNULL(decision_data);
  CHECK_NOTNULL(path_data);
  if (!init(reference_line)) {
    AERROR << "Fail to init dp road graph!";
    return false;
  }
  std::vector<DpNode> min_cost_path;
  if (!generate_graph(reference_line, decision_data, &min_cost_path)) {
    AERROR << "Fail to generate graph!";
    return false;
  }
  std::vector<common::FrenetFramePoint> frenet_path;
  double accumulated_s = _init_point.path_point().s();
  const double path_resolution = _config.path_resolution();
  for (size_t i = 1; i < min_cost_path.size(); ++i) {
    const auto &prev_node = min_cost_path[i - 1];
    const auto &cur_node = min_cost_path[i];
    const double path_length = cur_node.sl_point.s() - prev_node.sl_point.s();
    double current_s = 0.0;
    const auto &curve = cur_node.min_cost_curve;
    while (Double::compare(current_s, path_length) < 0.0) {
      const double l = curve.evaluate(0, current_s);
      const double dl = curve.evaluate(1, current_s);
      const double ddl = curve.evaluate(2, current_s);
      common::FrenetFramePoint frenet_frame_point;
      frenet_frame_point.set_s(accumulated_s + current_s);
      frenet_frame_point.set_l(l);
      frenet_frame_point.set_dl(dl);
      frenet_frame_point.set_ddl(ddl);
      frenet_path.push_back(frenet_frame_point);
      current_s += path_resolution;
    }
    accumulated_s += path_length;
  }
  FrenetFramePath tunnel(frenet_path);
  path_data->set_frenet_path(tunnel);
  // convert frenet path to cartesian path by reference line
  std::vector<::apollo::common::PathPoint> path_points;
  for (const common::FrenetFramePoint &frenet_point : frenet_path) {
    common::SLPoint sl_point;
    common::math::Vec2d cartesian_point;
    sl_point.set_s(frenet_point.s());
    sl_point.set_l(frenet_point.l());
    if (!reference_line.get_point_in_Cartesian_frame(sl_point,
                                                     &cartesian_point)) {
      AERROR << "Fail to convert sl point to xy point";
      return false;
    }
    ReferencePoint ref_point =
        reference_line.get_reference_point(frenet_point.s());
    double theta = SLAnalyticTransformation::calculate_theta(
        ref_point.heading(), ref_point.kappa(), frenet_point.l(),
        frenet_point.dl());
    double kappa = SLAnalyticTransformation::calculate_kappa(
        ref_point.kappa(), ref_point.dkappa(), frenet_point.l(),
        frenet_point.dl(), frenet_point.ddl());
    ::apollo::common::PathPoint path_point;
    path_point.set_x(cartesian_point.x());
    path_point.set_y(cartesian_point.y());
    path_point.set_z(0.0);
    path_point.set_theta(theta);
    path_point.set_kappa(kappa);
    path_point.set_dkappa(0.0);
    path_point.set_ddkappa(0.0);
    path_point.set_s(0.0);

    if (path_points.size() != 0) {
      common::math::Vec2d last(path_points.back().x(), path_points.back().y());
      common::math::Vec2d current(path_point.x(), path_point.y());
      double distance = (last - current).Length();
      path_point.set_s(path_points.back().s() + distance);
    }
    path_points.push_back(std::move(path_point));
  }
  DiscretizedPath discretized_path(path_points);
  path_data->set_path(discretized_path);
  return true;
}

bool DpRoadGraph::init(const ReferenceLine &reference_line) {
  common::math::Vec2d xy_point(_init_point.path_point().x(),
                               _init_point.path_point().y());

  if (!reference_line.get_point_in_frenet_frame(xy_point, &_init_sl_point)) {
    AERROR << "Fail to map init point to sl coordinate!";
    return false;
  }

  ReferencePoint reference_point =
      reference_line.get_reference_point(_init_sl_point.s());

  double init_dl = SLAnalyticTransformation::calculate_lateral_derivative(
      reference_point.heading(), _init_point.path_point().theta(),
      _init_sl_point.l(), reference_point.kappa());
  double init_ddl =
      SLAnalyticTransformation::calculate_second_order_lateral_derivative(
          reference_point.heading(), _init_point.path_point().theta(),
          reference_point.kappa(), _init_point.path_point().kappa(),
          reference_point.dkappa(), _init_sl_point.l());
  common::FrenetFramePoint init_frenet_frame_point;
  init_frenet_frame_point.set_s(_init_sl_point.s());
  init_frenet_frame_point.set_l(_init_sl_point.l());
  init_frenet_frame_point.set_dl(init_dl);
  init_frenet_frame_point.set_ddl(init_ddl);

  return true;
}

bool DpRoadGraph::generate_graph(const ReferenceLine &reference_line,
                                 DecisionData *const decision_data,
                                 std::vector<DpNode> *min_cost_path) {
  if (!min_cost_path) {
    AERROR << "the provided min_cost_path is null";
    return false;
  }
  std::vector<std::vector<common::SLPoint>> points;
  PathSampler path_sampler(_config);
  if (!path_sampler.sample(reference_line, _init_point, _init_sl_point,
                           &points)) {
    AERROR << "Fail to sampling point with path sampler!";
    return false;
  }
  points.insert(points.begin(), std::vector<common::SLPoint>{_init_sl_point});
  const auto &vehicle_config =
      common::VehicleConfigHelper::instance()->GetConfig();
  TrajectoryCost trajectory_cost(_config, reference_line,
                                 vehicle_config.vehicle_param(), _speed_data,
                                 *decision_data);
  std::vector<std::vector<DpNode>> dp_nodes(points.size());
  dp_nodes[0].push_back(DpNode(_init_sl_point, nullptr, 0.0));
  const double zero_dl = 0.0;   // always use 0 dl
  const double zero_ddl = 0.0;  // always use 0 ddl
  for (std::size_t level = 1; level < points.size(); ++level) {
    const auto &prev_dp_nodes = dp_nodes[level - 1];
    const auto &level_points = points[level];
    for (std::size_t i = 0; i < level_points.size(); ++i) {
      const auto cur_sl_point = level_points[i];
      dp_nodes[level].push_back(DpNode(cur_sl_point, nullptr));
      auto *cur_node = &dp_nodes[level].back();
      for (std::size_t j = 0; j < prev_dp_nodes.size(); ++j) {
        const auto *prev_dp_node = &prev_dp_nodes[j];
        const auto prev_sl_point = prev_dp_node->sl_point;
        QuinticPolynomialCurve1d curve(prev_sl_point.l(), zero_dl, zero_ddl,
                                       cur_sl_point.l(), zero_dl, zero_ddl,
                                       cur_sl_point.s() - prev_sl_point.s());
        const double cost = trajectory_cost.calculate(curve, prev_sl_point.s(),
                                                      cur_sl_point.s()) +
                            prev_dp_node->min_cost;
        cur_node->update_cost(prev_dp_node, curve, cost);
      }
    }
  }

  // find best path
  DpNode fake_head;
  const auto &last_dp_nodes = dp_nodes.back();
  for (std::size_t i = 0; i < last_dp_nodes.size(); ++i) {
    const auto &cur_dp_node = last_dp_nodes[i];
    fake_head.update_cost(&cur_dp_node, cur_dp_node.min_cost_curve,
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

}  // namespace planning
}  // namespace apollo
