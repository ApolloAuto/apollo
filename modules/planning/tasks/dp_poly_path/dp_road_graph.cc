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
#include "modules/planning/math/double.h"
#include "modules/planning/tasks/dp_poly_path/trajectory_cost.h"

namespace apollo {
namespace planning {

using apollo::common::ErrorCode;
using apollo::common::Status;
using IdDecisionList = std::vector<std::pair<std::string, ObjectDecisionType>>;
using ConstPathObstacleList = std::vector<const PathObstacle *>;

DPRoadGraph::DPRoadGraph(const DpPolyPathConfig &config,
                         const ReferenceLine &reference_line,
                         const SpeedData &speed_data)
    : config_(config),
      reference_line_(reference_line),
      speed_data_(speed_data) {}

bool DPRoadGraph::FindPathTunnel(const common::TrajectoryPoint &init_point,
                                 PathData *const path_data) {
  CHECK_NOTNULL(path_data);
  init_point_ = init_point;
  if (!reference_line_.get_point_in_frenet_frame(
          {init_point_.path_point().x(), init_point_.path_point().y()},
          &init_sl_point_)) {
    AERROR << "Fail to create init_sl_point from : "
           << init_point.DebugString();
    return false;
  }
  std::vector<DPRoadGraphNode> min_cost_path;
  if (!GenerateMinCostPath(&min_cost_path)) {
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
    while (Double::Compare(current_s, path_length) < 0.0) {
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
    accumulated_s += path_length;
  }
  FrenetFramePath tunnel(frenet_path);

  path_data->SetReferenceLine(&reference_line_);
  path_data->SetFrenetPath(tunnel);
  return true;
}

bool DPRoadGraph::GenerateMinCostPath(
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

  TrajectoryCost trajectory_cost(config_, reference_line_,
                                 vehicle_config.vehicle_param(), speed_data_);

  std::vector<std::vector<DPRoadGraphNode>> graph_nodes(path_waypoints.size());
  graph_nodes[0].emplace_back(init_sl_point_, nullptr, 0.0);

  for (std::size_t level = 1; level < path_waypoints.size(); ++level) {
    const auto &prev_dp_nodes = graph_nodes[level - 1];
    const auto &level_points = path_waypoints[level];
    for (std::size_t i = 0; i < level_points.size(); ++i) {
      const auto &cur_point = level_points[i];
      graph_nodes[level].emplace_back(cur_point, nullptr);
      auto &cur_node = graph_nodes[level].back();
      for (std::size_t j = 0; j < prev_dp_nodes.size(); ++j) {
        const auto &prev_dp_node = prev_dp_nodes[j];
        const auto &prev_sl_point = prev_dp_node.sl_point;
        QuinticPolynomialCurve1d curve(prev_sl_point.l(), 0.0, 0.0,
                                       cur_point.l(), 0.0, 0.0,
                                       cur_point.s() - prev_sl_point.s());
        const double cost =
            trajectory_cost.calculate(curve, prev_sl_point.s(), cur_point.s()) +
            prev_dp_node.min_cost;
        cur_node.UpdateCost(&prev_dp_node, curve, cost);
      }
    }
  }

  // find best path
  DPRoadGraphNode fake_head;
  const auto &last_dp_nodes = graph_nodes.back();
  for (std::size_t i = 0; i < last_dp_nodes.size(); ++i) {
    const auto &cur_dp_node = last_dp_nodes[i];
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

bool DPRoadGraph::MakeObjectDecision(const PathData &path_data,
                                     PathDecision *const path_decision) {
  DCHECK_NOTNULL(path_decision);
  if (!MakeStaticObstacleDecision(path_data, path_decision)) {
    AERROR << "Failed to make decisions for static obstacles";
    return false;
  }
  return true;
}

bool DPRoadGraph::MakeStaticObstacleDecision(
    const PathData &path_data, PathDecision *const path_decision) {
  DCHECK_NOTNULL(path_decision);
  std::vector<common::SLPoint> adc_sl_points;
  std::vector<common::math::Box2d> adc_bounding_box;
  const auto &vehicle_param =
      common::VehicleConfigHelper::instance()->GetConfig().vehicle_param();
  const double adc_length = vehicle_param.length();
  const double adc_width = vehicle_param.length();
  const double adc_max_edge_to_center_dist =
      std::hypot(std::max(vehicle_param.front_edge_to_center(),
                          vehicle_param.back_edge_to_center()),
                 std::max(vehicle_param.left_edge_to_center(),
                          vehicle_param.right_edge_to_center()));

  for (const common::PathPoint &path_point :
       path_data.discretized_path().path_points()) {
    adc_bounding_box.emplace_back(
        common::math::Vec2d{path_point.x(), path_point.y()}, path_point.theta(),
        adc_length, adc_width);
    common::SLPoint adc_sl;
    if (!reference_line_.get_point_in_frenet_frame(
            {path_point.x(), path_point.y()}, &adc_sl)) {
      AERROR << "get_point_in_Frenet_frame error for ego vehicle "
             << path_point.x() << " " << path_point.y();
      return false;
    } else {
      adc_sl_points.push_back(std::move(adc_sl));
    }
  }

  for (const auto *path_obstacle : path_decision->path_obstacles().Items()) {
    const auto *obstacle = path_obstacle->Obstacle();
    if (obstacle->IsVirtual()) {
      continue;
    }
    if (!obstacle->IsStatic()) {
      continue;
    }

    // IGNORE by default
    ObjectDecisionType object_decision;
    object_decision.mutable_ignore();

    const auto &sl_boundary = path_obstacle->perception_sl_boundary();
    bool has_stop = false;
    for (std::size_t j = 0; j < adc_sl_points.size(); ++j) {
      const auto &adc_sl = adc_sl_points[j];
      if (adc_sl.s() + adc_max_edge_to_center_dist +
                  FLAGS_static_decision_ignore_s_range <
              sl_boundary.start_s() ||
          adc_sl.s() - adc_max_edge_to_center_dist -
                  FLAGS_static_decision_ignore_s_range >
              sl_boundary.end_s()) {
        // ignore: no overlap in s direction
        continue;
      }

      if (adc_sl.l() + adc_max_edge_to_center_dist +
                  FLAGS_static_decision_nudge_l_buffer <
              sl_boundary.start_l() ||
          adc_sl.l() - adc_max_edge_to_center_dist -
                  FLAGS_static_decision_nudge_l_buffer >
              sl_boundary.end_l()) {
        // ignore: no overlap in l direction
        continue;
      }

      // check STOP/NUDGE
      double left_bound;
      double right_bound;
      if (!reference_line_.get_lane_width(adc_sl.s(), &left_bound,
                                          &right_bound)) {
        left_bound = right_bound = FLAGS_default_reference_line_width / 2;
      }
      bool left_nudgable = left_bound - sl_boundary.end_l() >=
                           FLAGS_static_decision_nudge_l_buffer;
      bool right_nudgable = sl_boundary.start_l() - right_bound >=
                            FLAGS_static_decision_nudge_l_buffer;

      if (!left_nudgable && !right_nudgable) {
        // STOP: and break
        ObjectDecisionType stop_decision;
        ObjectStop *object_stop_ptr = stop_decision.mutable_stop();
        object_stop_ptr->set_distance_s(-FLAGS_stop_distance_obstacle);
        object_stop_ptr->set_reason_code(StopReasonCode::STOP_REASON_OBSTACLE);

        auto stop_ref_s = sl_boundary.start_s() - FLAGS_stop_distance_obstacle;
        auto stop_ref_point = reference_line_.get_reference_point(stop_ref_s);
        object_stop_ptr->mutable_stop_point()->set_x(stop_ref_point.x());
        object_stop_ptr->mutable_stop_point()->set_y(stop_ref_point.y());
        object_stop_ptr->set_stop_heading(stop_ref_point.heading());
        path_decision->AddLongitudinalDecision("DpRoadGraph", obstacle->Id(),
                                               stop_decision);

        has_stop = true;
        break;
      } else {
        // NUDGE: and continue to check potential STOP along the ref line
        if (!object_decision.has_nudge()) {
          ObjectNudge *object_nudge_ptr = object_decision.mutable_nudge();
          if (left_nudgable) {
            // LEFT_NUDGE
            object_nudge_ptr->set_type(ObjectNudge::LEFT_NUDGE);
            object_nudge_ptr->set_distance_l(FLAGS_nudge_distance_obstacle);
          } else {
            // RIGHT_NUDGE
            object_nudge_ptr->set_type(ObjectNudge::RIGHT_NUDGE);
            object_nudge_ptr->set_distance_l(-FLAGS_nudge_distance_obstacle);
          }
        }
      }
    }
    if (!has_stop) {
      path_decision->AddLateralDecision("DpRoadGraph", obstacle->Id(),
                                        object_decision);
    }
  }

  return true;
}

bool DPRoadGraph::MakeDynamicObstcleDecision(
    const PathData &path_data, PathDecision *const path_decision) {
  // Compute dynamic obstacle decision
  const double interval = config_.eval_time_interval();
  const double total_time =
      std::min(speed_data_.TotalTime(), FLAGS_prediction_total_time);
  std::size_t evaluate_time_slots =
      static_cast<std::size_t>(std::floor(total_time / interval));
  // list of Box2d for adc given speed profile
  std::vector<common::math::Box2d> adc_by_time;
  if (!ComputeBoundingBoxesForAdc(path_data.frenet_frame_path(),
                                  evaluate_time_slots, &adc_by_time)) {
    AERROR << "fill adc_by_time error";
    return false;
  }

  for (const auto *path_obstacle : path_decision->path_obstacles().Items()) {
    const auto *obstacle = path_obstacle->Obstacle();
    if (obstacle->IsVirtual()) {
      continue;
    }
    if (obstacle->IsStatic()) {
      continue;
    }
    double timestamp = 0.0;
    for (std::size_t k = 0; k < evaluate_time_slots;
         ++k, timestamp += interval) {
      // TODO(all) make nudge decision for dynamic obstacles.
      // const auto obstacle_by_time =
      //     obstacle->GetBoundingBox(obstacle->GetPointAtTime(timestamp));
    }
  }
  return true;
}

bool DPRoadGraph::ComputeBoundingBoxesForAdc(
    const FrenetFramePath &frenet_frame_path,
    const std::size_t evaluate_time_slots,
    std::vector<common::math::Box2d> *adc_boxes) {
  CHECK(adc_boxes != nullptr);

  const auto &vehicle_config =
      common::VehicleConfigHelper::instance()->GetConfig();
  double adc_length = vehicle_config.vehicle_param().length();
  double adc_width = vehicle_config.vehicle_param().length();

  double time_stamp = 0.0;
  const double interval = config_.eval_time_interval();
  for (std::size_t i = 0; i < evaluate_time_slots;
       ++i, time_stamp += interval) {
    common::SpeedPoint speed_point;
    if (!speed_data_.EvaluateByTime(time_stamp, &speed_point)) {
      AINFO << "get_speed_point_with_time for time_stamp[" << time_stamp << "]";
      return false;
    }

    const common::FrenetFramePoint &interpolated_frenet_point =
        frenet_frame_path.EvaluateByS(speed_point.s());
    double s = interpolated_frenet_point.s();
    double l = interpolated_frenet_point.l();
    double dl = interpolated_frenet_point.dl();

    common::math::Vec2d adc_position_cartesian;
    common::SLPoint sl_point;
    sl_point.set_s(s);
    sl_point.set_l(l);
    reference_line_.get_point_in_cartesian_frame(sl_point,
                                                 &adc_position_cartesian);

    ReferencePoint reference_point = reference_line_.get_reference_point(s);

    double one_minus_kappa_r_d = 1 - reference_point.kappa() * l;
    double delta_theta = std::atan2(dl, one_minus_kappa_r_d);
    double theta = ::apollo::common::math::NormalizeAngle(
        delta_theta + reference_point.heading());

    adc_boxes->emplace_back(adc_position_cartesian, theta, adc_length,
                            adc_width);
  }
  return true;
}

bool DPRoadGraph::SamplePathWaypoints(
    const common::TrajectoryPoint &init_point,
    std::vector<std::vector<common::SLPoint>> *const points) {
  CHECK(points != nullptr);

  common::math::Vec2d init_cartesian_point(init_point.path_point().x(),
                                           init_point.path_point().y());
  common::SLPoint init_sl_point;
  if (!reference_line_.get_point_in_frenet_frame(init_cartesian_point,
                                                 &init_sl_point)) {
    AERROR << "Failed to get sl point from point "
           << init_cartesian_point.DebugString();
    return false;
  }

  const double reference_line_length =
      reference_line_.map_path().accumulated_s().back();

  double level_distance =
      std::fmax(config_.step_length_min(),
                std::fmin(init_point.v(), config_.step_length_max()));

  double accumulated_s = init_sl_point.s();
  for (std::size_t i = 0;
       i < config_.sample_level() && accumulated_s < reference_line_length;
       ++i) {
    std::vector<common::SLPoint> level_points;
    accumulated_s += level_distance;
    double s = std::fmin(accumulated_s, reference_line_length);

    int32_t num =
        static_cast<int32_t>(config_.sample_points_num_each_level() / 2);

    for (int32_t j = -num; j < num + 1; ++j) {
      double l = config_.lateral_sample_offset() * j;
      auto sl = common::util::MakeSLPoint(s, l);
      if (reference_line_.is_on_road(sl)) {
        level_points.push_back(sl);
      }
    }
    if (!level_points.empty()) {
      points->push_back(level_points);
    }
  }
  return true;
}

}  // namespace planning
}  // namespace apollo
