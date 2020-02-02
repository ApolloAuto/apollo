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
 * @file
 **/

#include "modules/planning/tasks/optimizers/road_graph/trajectory_cost.h"

#include "modules/common/proto/pnc_point.pb.h"

#include "modules/common/configs/vehicle_config_helper.h"
#include "modules/common/math/vec2d.h"
#include "modules/common/util/point_factory.h"
#include "modules/planning/common/planning_gflags.h"

namespace apollo {
namespace planning {

TrajectoryCost::TrajectoryCost(const DpPolyPathConfig &config,
                               const ReferenceLine &reference_line,
                               const bool is_change_lane_path,
                               const std::vector<const Obstacle *> &obstacles,
                               const common::VehicleParam &vehicle_param,
                               const SpeedData &heuristic_speed_data,
                               const common::SLPoint &init_sl_point,
                               const SLBoundary &adc_sl_boundary)
    : config_(config),
      reference_line_(&reference_line),
      is_change_lane_path_(is_change_lane_path),
      vehicle_param_(vehicle_param),
      heuristic_speed_data_(heuristic_speed_data),
      init_sl_point_(init_sl_point),
      adc_sl_boundary_(adc_sl_boundary) {
  const double total_time =
      std::min(heuristic_speed_data_.TotalTime(), FLAGS_prediction_total_time);

  num_of_time_stamps_ = static_cast<uint32_t>(
      std::floor(total_time / config.eval_time_interval()));

  for (const auto *ptr_obstacle : obstacles) {
    if (ptr_obstacle->IsIgnore()) {
      continue;
    } else if (ptr_obstacle->LongitudinalDecision().has_stop()) {
      continue;
    }
    const auto &sl_boundary = ptr_obstacle->PerceptionSLBoundary();

    const double adc_left_l =
        init_sl_point_.l() + vehicle_param_.left_edge_to_center();
    const double adc_right_l =
        init_sl_point_.l() - vehicle_param_.right_edge_to_center();

    if (adc_left_l + FLAGS_lateral_ignore_buffer < sl_boundary.start_l() ||
        adc_right_l - FLAGS_lateral_ignore_buffer > sl_boundary.end_l()) {
      continue;
    }

    bool is_bycycle_or_pedestrian =
        (ptr_obstacle->Perception().type() ==
             perception::PerceptionObstacle::BICYCLE ||
         ptr_obstacle->Perception().type() ==
             perception::PerceptionObstacle::PEDESTRIAN);

    if (ptr_obstacle->IsVirtual()) {
      // Virtual obstacle
      continue;
    } else if (ptr_obstacle->IsStatic() || is_bycycle_or_pedestrian) {
      static_obstacle_sl_boundaries_.push_back(std::move(sl_boundary));
    } else {
      std::vector<Box2d> box_by_time;
      for (uint32_t t = 0; t <= num_of_time_stamps_; ++t) {
        TrajectoryPoint trajectory_point =
            ptr_obstacle->GetPointAtTime(t * config.eval_time_interval());

        Box2d obstacle_box = ptr_obstacle->GetBoundingBox(trajectory_point);
        static constexpr double kBuff = 0.5;
        Box2d expanded_obstacle_box =
            Box2d(obstacle_box.center(), obstacle_box.heading(),
                  obstacle_box.length() + kBuff, obstacle_box.width() + kBuff);
        box_by_time.push_back(expanded_obstacle_box);
      }
      dynamic_obstacle_boxes_.push_back(std::move(box_by_time));
    }
  }
}

ComparableCost TrajectoryCost::CalculatePathCost(
    const QuinticPolynomialCurve1d &curve, const double start_s,
    const double end_s, const uint32_t curr_level, const uint32_t total_level) {
  ComparableCost cost;
  double path_cost = 0.0;
  std::function<double(const double)> quasi_softmax = [this](const double x) {
    const double l0 = this->config_.path_l_cost_param_l0();
    const double b = this->config_.path_l_cost_param_b();
    const double k = this->config_.path_l_cost_param_k();
    return (b + std::exp(-k * (x - l0))) / (1.0 + std::exp(-k * (x - l0)));
  };

  for (double curve_s = 0.0; curve_s < (end_s - start_s);
       curve_s += config_.path_resolution()) {
    const double l = curve.Evaluate(0, curve_s);

    path_cost += l * l * config_.path_l_cost() * quasi_softmax(std::fabs(l));

    const double dl = std::fabs(curve.Evaluate(1, curve_s));
    if (IsOffRoad(curve_s + start_s, l, dl, is_change_lane_path_)) {
      cost.cost_items[ComparableCost::OUT_OF_BOUNDARY] = true;
    }

    path_cost += dl * dl * config_.path_dl_cost();

    const double ddl = std::fabs(curve.Evaluate(2, curve_s));
    path_cost += ddl * ddl * config_.path_ddl_cost();
  }
  path_cost *= config_.path_resolution();

  if (curr_level == total_level) {
    const double end_l = curve.Evaluate(0, end_s - start_s);
    path_cost +=
        std::sqrt(end_l - init_sl_point_.l() / 2.0) * config_.path_end_l_cost();
  }
  cost.smoothness_cost = path_cost;
  return cost;
}

bool TrajectoryCost::IsOffRoad(const double ref_s, const double l,
                               const double dl,
                               const bool is_change_lane_path) {
  static constexpr double kIgnoreDistance = 5.0;
  if (ref_s - init_sl_point_.s() < kIgnoreDistance) {
    return false;
  }
  Vec2d rear_center(0.0, l);

  const auto &param = common::VehicleConfigHelper::GetConfig().vehicle_param();
  Vec2d vec_to_center(
      (param.front_edge_to_center() - param.back_edge_to_center()) / 2.0,
      (param.left_edge_to_center() - param.right_edge_to_center()) / 2.0);

  Vec2d rear_center_to_center = vec_to_center.rotate(std::atan(dl));
  Vec2d center = rear_center + rear_center_to_center;
  Vec2d front_center = center + rear_center_to_center;

  const double buffer = 0.1;  // in meters
  const double r_w =
      (param.left_edge_to_center() + param.right_edge_to_center()) / 2.0;
  const double r_l = param.back_edge_to_center();
  const double r = std::sqrt(r_w * r_w + r_l * r_l);

  double left_width = 0.0;
  double right_width = 0.0;
  reference_line_->GetLaneWidth(ref_s, &left_width, &right_width);

  double left_bound = std::max(init_sl_point_.l() + r + buffer, left_width);
  double right_bound = std::min(init_sl_point_.l() - r - buffer, -right_width);
  if (rear_center.y() + r + buffer / 2.0 > left_bound ||
      rear_center.y() - r - buffer / 2.0 < right_bound) {
    return true;
  }
  if (front_center.y() + r + buffer / 2.0 > left_bound ||
      front_center.y() - r - buffer / 2.0 < right_bound) {
    return true;
  }

  return false;
}

ComparableCost TrajectoryCost::CalculateStaticObstacleCost(
    const QuinticPolynomialCurve1d &curve, const double start_s,
    const double end_s) {
  ComparableCost obstacle_cost;
  for (double curr_s = start_s; curr_s <= end_s;
       curr_s += config_.path_resolution()) {
    const double curr_l = curve.Evaluate(0, curr_s - start_s);
    for (const auto &obs_sl_boundary : static_obstacle_sl_boundaries_) {
      obstacle_cost += GetCostFromObsSL(curr_s, curr_l, obs_sl_boundary);
    }
  }
  obstacle_cost.safety_cost *= config_.path_resolution();
  return obstacle_cost;
}

ComparableCost TrajectoryCost::CalculateDynamicObstacleCost(
    const QuinticPolynomialCurve1d &curve, const double start_s,
    const double end_s) const {
  ComparableCost obstacle_cost;
  if (dynamic_obstacle_boxes_.empty()) {
    return obstacle_cost;
  }

  double time_stamp = 0.0;
  for (size_t index = 0; index < num_of_time_stamps_;
       ++index, time_stamp += config_.eval_time_interval()) {
    common::SpeedPoint speed_point;
    heuristic_speed_data_.EvaluateByTime(time_stamp, &speed_point);
    double ref_s = speed_point.s() + init_sl_point_.s();
    if (ref_s < start_s) {
      continue;
    }
    if (ref_s > end_s) {
      break;
    }

    const double s = ref_s - start_s;  // s on spline curve
    const double l = curve.Evaluate(0, s);
    const double dl = curve.Evaluate(1, s);

    const common::SLPoint sl = common::util::PointFactory::ToSLPoint(ref_s, l);
    const Box2d ego_box = GetBoxFromSLPoint(sl, dl);
    for (const auto &obstacle_trajectory : dynamic_obstacle_boxes_) {
      obstacle_cost +=
          GetCostBetweenObsBoxes(ego_box, obstacle_trajectory.at(index));
    }
  }
  static constexpr double kDynamicObsWeight = 1e-6;
  obstacle_cost.safety_cost *=
      (config_.eval_time_interval() * kDynamicObsWeight);
  return obstacle_cost;
}

ComparableCost TrajectoryCost::GetCostFromObsSL(
    const double adc_s, const double adc_l, const SLBoundary &obs_sl_boundary) {
  const auto &vehicle_param =
      common::VehicleConfigHelper::Instance()->GetConfig().vehicle_param();

  ComparableCost obstacle_cost;
  if (obs_sl_boundary.start_l() * obs_sl_boundary.end_l() <= 0.0) {
    return obstacle_cost;
  }

  const double adc_front_s = adc_s + vehicle_param.front_edge_to_center();
  const double adc_end_s = adc_s - vehicle_param.back_edge_to_center();
  const double adc_left_l = adc_l + vehicle_param.left_edge_to_center();
  const double adc_right_l = adc_l - vehicle_param.right_edge_to_center();

  if (adc_left_l + FLAGS_lateral_ignore_buffer < obs_sl_boundary.start_l() ||
      adc_right_l - FLAGS_lateral_ignore_buffer > obs_sl_boundary.end_l()) {
    return obstacle_cost;
  }

  bool no_overlap = ((adc_front_s < obs_sl_boundary.start_s() ||
                      adc_end_s > obs_sl_boundary.end_s()) ||  // longitudinal
                     (adc_left_l + 0.1 < obs_sl_boundary.start_l() ||
                      adc_right_l - 0.1 > obs_sl_boundary.end_l()));  // lateral

  if (!no_overlap) {
    obstacle_cost.cost_items[ComparableCost::HAS_COLLISION] = true;
  }

  // if obstacle is behind ADC, ignore its cost contribution.
  if (adc_front_s > obs_sl_boundary.end_s()) {
    return obstacle_cost;
  }

  const double delta_l = std::fmax(adc_right_l - obs_sl_boundary.end_l(),
                                   obs_sl_boundary.start_l() - adc_left_l);
  /*
  AWARN << "adc_s: " << adc_s << "; adc_left_l: " << adc_left_l
        << "; adc_right_l: " << adc_right_l << "; delta_l = " << delta_l;
  AWARN << obs_sl_boundary.ShortDebugString();
  */

  static constexpr double kSafeDistance = 0.6;
  if (delta_l < kSafeDistance) {
    obstacle_cost.safety_cost +=
        config_.obstacle_collision_cost() *
        Sigmoid(config_.obstacle_collision_distance() - delta_l);
  }

  return obstacle_cost;
}

// Simple version: calculate obstacle cost by distance
ComparableCost TrajectoryCost::GetCostBetweenObsBoxes(
    const Box2d &ego_box, const Box2d &obstacle_box) const {
  ComparableCost obstacle_cost;

  const double distance = obstacle_box.DistanceTo(ego_box);
  if (distance > config_.obstacle_ignore_distance()) {
    return obstacle_cost;
  }

  obstacle_cost.safety_cost +=
      config_.obstacle_collision_cost() *
      Sigmoid(config_.obstacle_collision_distance() - distance);
  obstacle_cost.safety_cost +=
      20.0 * Sigmoid(config_.obstacle_risk_distance() - distance);
  return obstacle_cost;
}

Box2d TrajectoryCost::GetBoxFromSLPoint(const common::SLPoint &sl,
                                        const double dl) const {
  Vec2d xy_point;
  reference_line_->SLToXY(sl, &xy_point);

  ReferencePoint reference_point = reference_line_->GetReferencePoint(sl.s());

  const double one_minus_kappa_r_d = 1 - reference_point.kappa() * sl.l();
  const double delta_theta = std::atan2(dl, one_minus_kappa_r_d);
  const double theta =
      common::math::NormalizeAngle(delta_theta + reference_point.heading());
  return Box2d(xy_point, theta, vehicle_param_.length(),
               vehicle_param_.width());
}

// TODO(All): optimize obstacle cost calculation time
ComparableCost TrajectoryCost::Calculate(const QuinticPolynomialCurve1d &curve,
                                         const double start_s,
                                         const double end_s,
                                         const uint32_t curr_level,
                                         const uint32_t total_level) {
  ComparableCost total_cost;
  // path cost
  total_cost +=
      CalculatePathCost(curve, start_s, end_s, curr_level, total_level);

  // static obstacle cost
  total_cost += CalculateStaticObstacleCost(curve, start_s, end_s);

  // dynamic obstacle cost
  total_cost += CalculateDynamicObstacleCost(curve, start_s, end_s);
  return total_cost;
}

}  // namespace planning
}  // namespace apollo
