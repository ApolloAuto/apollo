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
    : _config(config),
      _init_point(init_point),
      _heuristic_speed_data(speed_data) {}

Status DpRoadGraph::find_tunnel(const ReferenceLine &reference_line,
                                DecisionData *const decision_data,
                                PathData *const path_data) {
  CHECK_NOTNULL(path_data);
  if (!init(reference_line)) {
    const std::string msg = "Fail to init dp road graph!";
    AERROR << msg;
    return Status(ErrorCode::PLANNING_ERROR, msg);
  }
  if (!generate_graph(reference_line).ok()) {
    const std::string msg = "Fail to generate graph!";
    AERROR << msg;
    return Status(ErrorCode::PLANNING_ERROR, msg);
  }
  std::vector<uint32_t> min_cost_edges;
  if (!find_best_trajectory(reference_line, *decision_data, &min_cost_edges)
           .ok()) {
    const std::string msg = "Fail to find best trajectory!";
    AERROR << msg;
    return Status(ErrorCode::PLANNING_ERROR, msg);
  }
  FrenetFramePath tunnel;
  std::vector<common::FrenetFramePoint> frenet_path;
  frenet_path.push_back(_vertices[0].frame_point());
  double accumulated_s = 0.0;
  for (uint32_t i = 0; i < min_cost_edges.size(); ++i) {
    GraphEdge edge = _edges[min_cost_edges[i]];
    GraphVertex end_vertex = _vertices[edge.to_vertex()];
    double step = 0.1;  // TODO(yifei): get from config.
    double current_s = step;
    while (Double::compare(current_s, end_vertex.frame_point().s()) < 0.0) {
      current_s += step;
      const double l = edge.poly_path().evaluate(1, current_s);
      const double dl = edge.poly_path().evaluate(2, current_s);
      const double ddl = edge.poly_path().evaluate(3, current_s);
      common::FrenetFramePoint frenet_frame_point;
      frenet_frame_point.set_s(accumulated_s + current_s);
      frenet_frame_point.set_l(l);
      frenet_frame_point.set_dl(dl);
      frenet_frame_point.set_ddl(ddl);
      frenet_path.push_back(frenet_frame_point);
    }
    frenet_path.push_back(end_vertex.frame_point());
    accumulated_s += end_vertex.frame_point().s();
  }
  tunnel.set_frenet_points(frenet_path);
  path_data->set_frenet_path(tunnel);
  // convert frenet path to path by reference line
  std::vector<::apollo::common::PathPoint> path_points;
  for (const common::FrenetFramePoint &frenet_point : frenet_path) {
    common::math::Vec2d xy_point;
    common::SLPoint sl_point;
    sl_point.set_s(frenet_point.s());
    sl_point.set_l(frenet_point.l());

    if (!reference_line.get_point_in_Cartesian_frame(sl_point, &xy_point)) {
      const std::string msg = "Fail to convert sl point to xy point";
      AERROR << msg;
      return Status(ErrorCode::PLANNING_ERROR, msg);
    }
    ReferencePoint ref_point =
        reference_line.get_reference_point(frenet_point.s());
    double theta = SLAnalyticTransformation::calculate_theta(
        ref_point.heading(), ref_point.kappa(), frenet_point.l(),
        frenet_point.dl());
    // TODO(yifei) comment out unused variable
    // double kappa =
    // SLAnalyticTransformation::calculate_kappa(ref_point.kappa(),
    //                                           ref_point.dkappa(),
    //                                           frenet_point.l(),
    //                                           frenet_point.dl(),
    //                                           frenet_point.ddl());

    ::apollo::common::PathPoint
        path_point;  // (xy_point, theta, kappa, 0.0, 0.0, 0.0);
    path_point.set_x(xy_point.x());
    path_point.set_y(xy_point.y());
    path_point.set_theta(theta);
    path_point.set_dkappa(0.0);
    path_point.set_ddkappa(0.0);
    path_point.set_s(0.0);
    path_point.set_z(0.0);

    if (path_points.size() != 0) {
      common::math::Vec2d last(path_points.back().x(), path_points.back().y());
      common::math::Vec2d current(path_point.x(), path_point.y());
      double distance = (last - current).Length();
      path_point.set_s(path_points.back().s() + distance);
    }
    path_points.push_back(std::move(path_point));
  }
  *(path_data->mutable_path()->mutable_path_points()) = path_points;
  return Status::OK();
}

bool DpRoadGraph::init(const ReferenceLine &reference_line) {
  _vertices.clear();
  _edges.clear();
  common::math::Vec2d xy_point(_init_point.path_point().x(),
                               _init_point.path_point().y());

  if (!reference_line.get_point_in_Frenet_frame(xy_point, &_init_sl_point)) {
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

  _vertices.emplace_back(init_frenet_frame_point, 0, 0);
  _vertices.back().set_type(GraphVertex::Type::GRAPH_HEAD);
  _vertices.back().set_accumulated_cost(0.0);
  return true;
}

Status DpRoadGraph::generate_graph(const ReferenceLine &reference_line) {
  std::vector<std::vector<common::SLPoint>> points;
  PathSampler path_sampler(_config);
  if (!path_sampler.sample(reference_line, _init_point, _init_sl_point, &points)
           .ok()) {
    const std::string msg = "Fail to sampling point with path sampler!";
    AERROR << msg;
    return Status(ErrorCode::PLANNING_ERROR, msg);
  }

  int vertex_num_previous_level = 1;
  uint32_t accumulated_prev_level_size = 0;
  uint32_t accumulated_index = 0;
  for (uint32_t i = 0; i < points.size(); ++i) {
    int vertex_num_current_level = 0;
    ReferencePoint reference_point =
        reference_line.get_reference_point(points[i][0].s());
    for (uint32_t j = 0; j < points[i].size(); ++j) {
      if (!add_vertex(points[i][j], reference_point, i + 1)) {
        continue;
      }
      bool is_connected = false;
      for (int n = 0; n < vertex_num_previous_level; ++n) {
        const uint32_t index_start = accumulated_prev_level_size + n;
        const uint32_t index_end = accumulated_prev_level_size +
                                   vertex_num_previous_level +
                                   vertex_num_current_level;
        if (connect_vertex(index_start, index_end)) {
          is_connected = true;
        }
      }
      if (is_connected) {
        ++vertex_num_current_level;
        if (i + 1 == points.size()) {
          _vertices.back().set_type(GraphVertex::Type::DEAD_END);
        }
        ++accumulated_index;
      } else {
        _vertices.pop_back();
      }
      if (vertex_num_current_level == 0) {
        return Status(ErrorCode::PLANNING_ERROR, "DpRoadGraph::generate_graph");
      }
    }
    accumulated_prev_level_size += vertex_num_previous_level;
    vertex_num_previous_level = vertex_num_current_level;
  }
  return Status::OK();
}

Status DpRoadGraph::find_best_trajectory(
    const ReferenceLine &reference_line, const DecisionData &decision_data,
    std::vector<uint32_t> *const min_cost_edges) {
  CHECK_NOTNULL(min_cost_edges);
  std::unordered_map<uint32_t, uint32_t> vertex_connect_table;
  GraphVertex &head = _vertices[0];
  head.set_accumulated_cost(0.0);
  uint32_t best_trajectory_end_index = 0;
  const auto &vehicle_config =
      common::VehicleConfigHelper::instance()->GetConfig();
  double min_trajectory_cost = std::numeric_limits<double>::max();
  TrajectoryCost trajectory_cost(_config, vehicle_config.vehicle_param(),
                                 _heuristic_speed_data, decision_data);

  for (uint32_t i = 0; i < _vertices.size(); ++i) {
    const GraphVertex &vertex = _vertices[i];
    const std::vector<uint32_t> edges = vertex.edges_out();
    for (uint32_t j = 0; j < edges.size(); ++j) {
      double start_s = vertex.frame_point().s();
      double end_s = _vertices[_edges[j].to_vertex()].frame_point().s();
      double cost = trajectory_cost.calculate(_edges[j].poly_path(), start_s,
                                              end_s, reference_line);
      GraphVertex &vertex_end = _vertices[_edges[j].to_vertex()];
      if (vertex_connect_table.find(vertex_end.index()) ==
          vertex_connect_table.end()) {
        vertex_end.set_accumulated_cost(vertex.accumulated_cost() + cost);
        vertex_connect_table[vertex_end.index()] = vertex.index();
      } else {
        if (Double::compare(vertex.accumulated_cost() + cost,
                            vertex_end.accumulated_cost()) < 0) {
          vertex_end.set_accumulated_cost(vertex.accumulated_cost() + cost);
          vertex_connect_table[vertex_end.index()] = vertex.index();
        }
      }
    }
    if (vertex.is_dead_end() &&
        Double::compare(vertex.accumulated_cost(), min_trajectory_cost) <= 0) {
      best_trajectory_end_index = vertex.index();
      min_trajectory_cost = vertex.accumulated_cost();
    }
  }
  while (best_trajectory_end_index != 0) {
    min_cost_edges->push_back(best_trajectory_end_index);
    best_trajectory_end_index = vertex_connect_table[best_trajectory_end_index];
  }
  min_cost_edges->push_back(0);
  std::reverse(min_cost_edges->begin(), min_cost_edges->end());
  return Status::OK();
}

bool DpRoadGraph::add_vertex(const common::SLPoint &sl_point,
                             const ReferencePoint &reference_point,
                             const uint32_t level) {
  double kappa = reference_point.kappa();
  double kappa_range_upper = 0.23;
  double kappa_range_lower = -kappa_range_upper;

  if (Double::compare(kappa * sl_point.l(), 1.0) == 0) {
    kappa = 0.0;
  } else {
    kappa = kappa / (1 - kappa * sl_point.l());
    if (kappa < kappa_range_lower || kappa > kappa_range_upper) {
      // Invalid sample point
      return false;
    }
    kappa = std::max(kappa_range_lower, kappa);
    kappa = std::min(kappa_range_upper, kappa);
  }

  common::FrenetFramePoint frenet_frame_point;
  frenet_frame_point.set_s(sl_point.s());
  frenet_frame_point.set_l(sl_point.l());
  frenet_frame_point.set_dl(0.0);
  frenet_frame_point.set_ddl(0.0);

  _vertices.emplace_back(frenet_frame_point, _vertices.size(), level);
  return true;
}

bool DpRoadGraph::connect_vertex(const uint32_t start, const uint32_t end) {
  GraphVertex &v_start = _vertices[start];
  GraphVertex &v_end = _vertices[end];
  QuinticPolynomialCurve1d curve(
      v_start.frame_point().l(), v_start.frame_point().dl(),
      v_start.frame_point().ddl(), v_end.frame_point().l(),
      v_end.frame_point().dl(), v_end.frame_point().ddl(),
      v_end.frame_point().s() - v_start.frame_point().s());
  v_start.add_out_vertex(end);
  v_start.add_out_edge(_edges.size());

  v_end.add_in_vertex(start);
  v_end.add_in_edge(_edges.size());

  _edges.emplace_back(start, end, v_start.level());
  _edges.back().set_edge_index(_edges.size() - 1);
  _edges.back().set_poly_path(curve);
  return true;
}

}  // namespace planning
}  // namespace apollo
