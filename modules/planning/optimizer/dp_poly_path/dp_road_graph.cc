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
#include "modules/planning/common/planning_gflags.h"
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
    : config_(config), init_point_(init_point), speed_data_(speed_data) {}

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
  double accumulated_s = init_sl_point_.s();
  const double path_resolution = config_.path_resolution();
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

  // compute decision data
  if (compute_decision_from_path(tunnel, speed_data_, reference_line,
                                 decision_data)) {
    AINFO << "Computing decision_data in dp path success";
  } else {
    AINFO << "Computing decision_data in dp path fail";
  }

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
  path_data->set_discretized_path(path_points);
  return true;
}

bool DpRoadGraph::init(const ReferenceLine &reference_line) {
  common::math::Vec2d xy_point(init_point_.path_point().x(),
                               init_point_.path_point().y());

  if (!reference_line.get_point_in_frenet_frame(xy_point, &init_sl_point_)) {
    AERROR << "Fail to map init point to sl coordinate!";
    return false;
  }

  ReferencePoint reference_point =
      reference_line.get_reference_point(init_sl_point_.s());

  double init_dl = SLAnalyticTransformation::calculate_lateral_derivative(
      reference_point.heading(), init_point_.path_point().theta(),
      init_sl_point_.l(), reference_point.kappa());
  double init_ddl =
      SLAnalyticTransformation::calculate_second_order_lateral_derivative(
          reference_point.heading(), init_point_.path_point().theta(),
          reference_point.kappa(), init_point_.path_point().kappa(),
          reference_point.dkappa(), init_sl_point_.l());
  common::FrenetFramePoint init_frenet_frame_point;
  init_frenet_frame_point.set_s(init_sl_point_.s());
  init_frenet_frame_point.set_l(init_sl_point_.l());
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
  PathSampler path_sampler(config_);
  if (!path_sampler.sample(reference_line, init_point_, &points)) {
    AERROR << "Fail to sampling point with path sampler!";
    return false;
  }
  points.insert(points.begin(), std::vector<common::SLPoint>{init_sl_point_});
  const auto &vehicleconfig_ =
      common::VehicleConfigHelper::instance()->GetConfig();
  TrajectoryCost trajectory_cost(config_, reference_line,
                                 vehicleconfig_.vehicle_param(), speed_data_,
                                 *decision_data);
  std::vector<std::vector<DpNode>> dp_nodes(points.size());
  dp_nodes[0].push_back(DpNode(init_sl_point_, nullptr, 0.0));
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

bool DpRoadGraph::compute_decision_from_path(
    const FrenetFramePath &tunnel, const SpeedData &heuristic_speed_data,
    const ReferenceLine &reference_line, DecisionData *const decision_data) {
  CHECK_NOTNULL(decision_data);
  const std::vector<common::FrenetFramePoint> &frenet_frame_points =
      tunnel.points();

  std::vector<Obstacle *> static_obstacles =
      decision_data->MutableStaticObstacles();

  // Compute static obstacle decision
  for (size_t i = 0; i < static_obstacles.size(); ++i) {
    // Attention: assume that there is only 1 trajectory for static obstacle AND
    // There is only one point for predicted static obstacles
    TrajectoryPoint traj_point =
        static_obstacles[i]->prediction_trajectories()[0].evaluate(0.0);
    ::apollo::common::math::Vec2d static_center_point = {
        traj_point.path_point().x(), traj_point.path_point().y()};
    ::apollo::common::math::Box2d static_obstacle_box = {
        static_center_point, traj_point.path_point().theta(),
        static_obstacles[i]->BoundingBox().length(),
        static_obstacles[i]->BoundingBox().width()};

    common::SLPoint static_obstacle_sl_point;
    reference_line.get_point_in_frenet_frame(
        {traj_point.path_point().x(), traj_point.path_point().y()},
        &static_obstacle_sl_point);
    double static_obstacle_s = static_obstacle_sl_point.s();
    double static_obstacle_l = static_obstacle_sl_point.l();
    double static_obstacle_s_min = std::numeric_limits<double>::max();
    double static_obstacle_s_max = std::numeric_limits<double>::min();

    reference_line.get_s_range_from_box2d(
        static_obstacle_box, &static_obstacle_s_max, &static_obstacle_s_min);

    const auto &vehicle_config =
        common::VehicleConfigHelper::instance()->GetConfig();

    static_obstacle_s_max += vehicle_config.vehicle_param().length();
    static_obstacle_s_min -= vehicle_config.vehicle_param().length();

    bool is_nudge = true;
    double diff_s = std::numeric_limits<double>::max();
    double diff_l = 0.0;
    for (size_t j = 0; j <= frenet_frame_points.size(); ++j) {
      // From each frame, get master frenet frame point corresponding xy-Box
      const common::FrenetFramePoint &frenet_frame_point =
          frenet_frame_points[j];
      if (frenet_frame_point.s() < static_obstacle_s_min ||
          frenet_frame_point.s() > static_obstacle_s_max) {
        continue;
      }

      common::math::Vec2d ego_position_cartesian;
      common::SLPoint sl_point;
      sl_point.set_s(frenet_frame_point.s());
      sl_point.set_l(frenet_frame_point.l());
      reference_line.get_point_in_Cartesian_frame(sl_point,
                                                  &ego_position_cartesian);
      ReferencePoint reference_point =
          reference_line.get_reference_point(frenet_frame_point.s());

      double one_minus_kappa_r_d =
          1 - reference_point.kappa() * frenet_frame_point.l();
      double delta_theta =
          std::atan2(frenet_frame_point.dl(), one_minus_kappa_r_d);
      double theta = ::apollo::common::math::NormalizeAngle(
          delta_theta + reference_point.heading());

      ::apollo::common::math::Box2d ego_bounding_box = {
          {ego_position_cartesian.x(), ego_position_cartesian.y()},
          theta,
          vehicle_config.vehicle_param().length(),
          vehicle_config.vehicle_param().width()};

      if (ego_bounding_box.DistanceTo(static_obstacle_box) <
          vehicle_config.vehicle_param().width() +
              FLAGS_static_decision_stop_buffer) {
        // Potentially a STOP, not NUDGE or IGNORE
        break;
      }

      // compute decision
      if (ego_bounding_box.DistanceTo(static_obstacle_box) <
          FLAGS_static_decision_ignore_range) {
        is_nudge = false;
      }

      if (std::fabs(frenet_frame_point.s() - static_obstacle_s) < diff_s) {
        diff_s = std::fabs(frenet_frame_point.s() - static_obstacle_s);
        diff_l = frenet_frame_point.l() - static_obstacle_l;
      }
    }

    if (is_nudge) {
      ObjectDecisionType object_nudge;
      ObjectNudge *object_nudge_ptr = object_nudge.mutable_nudge();
      if (diff_l > 0.0) {
        // Left nudge to be distinguished
        object_nudge_ptr->set_type(ObjectNudge::LEFT_NUDGE);
        object_nudge_ptr->set_distance_l(FLAGS_dp_path_decision_buffer);
        static_obstacles[i]->MutableDecisions()->push_back(object_nudge);
      } else {
        // Right nudge to be distinguished
        object_nudge_ptr->set_type(ObjectNudge::RIGHT_NUDGE);
        object_nudge_ptr->set_distance_l(FLAGS_dp_path_decision_buffer);
        static_obstacles[i]->MutableDecisions()->push_back(object_nudge);
      }
    } else {
      // Ignore
      ObjectDecisionType object_ignore;
      ObjectIgnore *object_ignore_ptr = object_ignore.mutable_ignore();
      CHECK_NOTNULL(object_ignore_ptr);
      static_obstacles[i]->MutableDecisions()->push_back(object_ignore);
    }
  }

  // Compute dynamic obstacle decision
  // TBD
  std::vector<Obstacle *> dynamic_obstacles =
      decision_data->MutableDynamicObstacles();
  const double total_time =
      std::min(heuristic_speed_data.total_time(), FLAGS_prediction_total_time);
  size_t evaluate_times = static_cast<size_t>(
      std::floor(total_time / config_.eval_time_interval()));
  for (size_t i = 0; i < dynamic_obstacles.size(); ++i) {
    const auto &trajectories = dynamic_obstacles[i]->prediction_trajectories();
    for (size_t j = 0; j < trajectories.size(); ++j) {
      const auto &trajectory = trajectories[j];
      std::vector<::apollo::common::math::Box2d> obstacle_by_time;
      for (size_t time = 0; time <= evaluate_times; ++time) {
        TrajectoryPoint traj_point =
            trajectory.evaluate(time * config_.eval_time_interval());
        ::apollo::common::math::Vec2d center_point = {
            traj_point.path_point().x(), traj_point.path_point().y()};
        ::apollo::common::math::Box2d obstacle_box = {
            center_point, traj_point.path_point().theta(),
            dynamic_obstacles[i]->BoundingBox().length(),
            dynamic_obstacles[i]->BoundingBox().width()};
        obstacle_by_time.push_back(obstacle_box);
      }
    }
  }
  return true;
}

}  // namespace planning
}  // namespace apollo
