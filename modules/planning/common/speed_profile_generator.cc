/******************************************************************************
 * Copyright 2018 The Apollo Authors. All Rights Reserved.
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
 * @file speed_profile_generator.cc
 **/

#include "modules/planning/common/speed_profile_generator.h"

#include <algorithm>
#include <limits>
#include <memory>

#include "cyber/common/log.h"
#include "modules/planning/common/ego_info.h"
#include "modules/planning/common/frame.h"
#include "modules/planning/common/planning_gflags.h"
#include "modules/planning/math/piecewise_jerk/piecewise_jerk_speed_problem.h"

namespace apollo {
namespace planning {

using common::SLPoint;
using common::SpeedPoint;
using common::TrajectoryPoint;
using common::math::Vec2d;

SpeedData SpeedProfileGenerator::GenerateFallbackSpeed(
    const double stop_distance) {
  AERROR << "Fallback using piecewise jerk speed!";
  const double init_v = EgoInfo::Instance()->start_point().v();
  const double init_a = EgoInfo::Instance()->start_point().a();
  const auto& veh_param =
      common::VehicleConfigHelper::GetConfig().vehicle_param();

  // if already stopped
  if (init_v <= 0.0 && init_a <= 0.0) {
    AWARN << "Already stopped! Nothing to do in GenerateFallbackSpeed()";
    SpeedData speed_data;
    speed_data.AppendSpeedPoint(0.0, 0.0, 0.0, 0.0, 0.0);
    FillEnoughSpeedPoints(&speed_data);
    return speed_data;
  }

  std::array<double, 3> init_s = {0.0, init_v, init_a};
  std::array<double, 3> end_s = {stop_distance, 0.0, 0.0};

  // TODO(all): dt is too small;
  double delta_t = FLAGS_fallback_time_unit;
  double total_time = FLAGS_fallback_total_time;
  const size_t num_of_knots = static_cast<size_t>(total_time / delta_t) + 1;

  PiecewiseJerkSpeedProblem piecewise_jerk_problem(num_of_knots, delta_t,
                                                   init_s);

  piecewise_jerk_problem.set_end_state_ref({10000.0, 0.0, 0.0}, end_s);

  // TODO(Hongyi): tune the params and move to a config
  piecewise_jerk_problem.set_weight_ddx(1.0);
  piecewise_jerk_problem.set_weight_dddx(0.01);

  piecewise_jerk_problem.set_x_bounds(0.0, std::fmax(stop_distance, 100.0));
  piecewise_jerk_problem.set_dx_bounds(
      0.0, std::fmax(FLAGS_planning_upper_speed_limit, init_v));
  piecewise_jerk_problem.set_ddx_bounds(veh_param.max_deceleration(),
                                        veh_param.max_acceleration());
  // TODO(Hongyi): Set back to vehicle_params when ready
  piecewise_jerk_problem.set_ddx_bounds(-4.4, 2.0);
  piecewise_jerk_problem.set_dddx_bound(FLAGS_longitudinal_jerk_bound);

  // Solve the problem
  if (!piecewise_jerk_problem.Optimize()) {
    AERROR << "Piecewise jerk fallback speed optimizer failed!";
    return GenerateStopProfile(init_v, init_a);
  }

  // Extract output
  const std::vector<double>& s = piecewise_jerk_problem.opt_x();
  const std::vector<double>& ds = piecewise_jerk_problem.opt_dx();
  const std::vector<double>& dds = piecewise_jerk_problem.opt_ddx();

  SpeedData speed_data;
  speed_data.AppendSpeedPoint(s[0], 0.0, ds[0], dds[0], 0.0);
  for (size_t i = 1; i < num_of_knots; ++i) {
    // Avoid the very last points when already stopped
    if (s[i] - s[i - 1] <= 0.0 || ds[i] <= 0.0) {
      break;
    }
    speed_data.AppendSpeedPoint(s[i], delta_t * static_cast<double>(i), ds[i],
                                dds[i], (dds[i] - dds[i - 1]) / delta_t);
  }
  FillEnoughSpeedPoints(&speed_data);
  return speed_data;
}

void SpeedProfileGenerator::FillEnoughSpeedPoints(SpeedData* const speed_data) {
  const SpeedPoint& last_point = speed_data->back();
  if (last_point.t() >= FLAGS_fallback_total_time) {
    return;
  }
  for (double t = last_point.t() + FLAGS_fallback_time_unit;
       t < FLAGS_fallback_total_time; t += FLAGS_fallback_time_unit) {
    speed_data->AppendSpeedPoint(last_point.s(), t, 0.0, 0.0, 0.0);
  }
}

SpeedData SpeedProfileGenerator::GenerateFallbackSpeedProfile() {
  const double init_v = EgoInfo::Instance()->start_point().v();
  const double init_a = EgoInfo::Instance()->start_point().a();
  if (init_v > FLAGS_polynomial_speed_fallback_velocity) {
    auto speed_data = GenerateStopProfileFromPolynomial(init_v, init_a);
    if (!speed_data.empty()) {
      return speed_data;
    }
  }
  return GenerateStopProfile(init_v, init_a);
}

SpeedData SpeedProfileGenerator::GenerateFallbackSpeedProfileWithStopDistance(
    const double stop_distance) {
  const double init_v = EgoInfo::Instance()->start_point().v();
  const double init_a = EgoInfo::Instance()->start_point().a();
  if (init_v > FLAGS_polynomial_speed_fallback_velocity) {
    auto speed_data =
        GenerateStopProfileFromPolynomial(init_v, init_a, stop_distance);
    if (!speed_data.empty()) {
      return speed_data;
    }
  }
  return GenerateStopProfile(init_v, init_a, stop_distance);
}

SpeedData SpeedProfileGenerator::GenerateStopProfile(const double init_speed,
                                                     const double init_acc) {
  AERROR << "Slowing down the car within a constant deceleration with fallback "
            "stopping profile.";
  SpeedData speed_data;

  const double max_t = FLAGS_fallback_total_time;
  const double unit_t = FLAGS_fallback_time_unit;

  double pre_s = 0.0;
  double pre_v = init_speed;
  double acc = FLAGS_slowdown_profile_deceleration;

  for (double t = 0.0; t < max_t; t += unit_t) {
    double s = 0.0;
    double v = 0.0;
    s = std::fmax(pre_s,
                  pre_s + 0.5 * (pre_v + (pre_v + unit_t * acc)) * unit_t);
    v = std::fmax(0.0, pre_v + unit_t * acc);
    speed_data.AppendSpeedPoint(s, t, v, acc, 0.0);
    pre_s = s;
    pre_v = v;
  }
  FillEnoughSpeedPoints(&speed_data);
  return speed_data;
}

SpeedData SpeedProfileGenerator::GenerateStopProfile(
    const double init_speed, const double init_acc,
    const double stop_distance) {
  AERROR << "Slowing down the car within a stop distance with fallback "
            "stopping profile.";
  SpeedData speed_data;

  constexpr double kEpsilon = 1.0e-8;
  const double unit_t = FLAGS_fallback_time_unit;
  double pre_s = 0.0;
  double pre_v = init_speed;
  double buffered_stop_distance =
      stop_distance > 0.0 ? stop_distance : kEpsilon;
  buffered_stop_distance = stop_distance - FLAGS_fallback_distance_buffer > 0.0
                               ? stop_distance - FLAGS_fallback_distance_buffer
                               : stop_distance;
  double acc = -(init_speed * init_speed) / (2.0 * buffered_stop_distance);
  double max_t = std::abs(init_speed / acc);

  for (double t = 0.0; t < max_t; t += unit_t) {
    double s = 0.0;
    double v = 0.0;
    s = std::fmax(pre_s,
                  pre_s + 0.5 * (pre_v + (pre_v + unit_t * acc)) * unit_t);
    v = std::fmax(0.0, pre_v + unit_t * acc);
    speed_data.AppendSpeedPoint(s, t, v, acc, 0.0);
    pre_s = s;
    pre_v = v;
  }
  FillEnoughSpeedPoints(&speed_data);
  return speed_data;
}

SpeedData SpeedProfileGenerator::GenerateStopProfileFromPolynomial(
    const double init_speed, const double init_acc,
    const double stop_distance) {
  AERROR << "Slowing down the car within a stop distance with polynomial.";
  constexpr double kMaxT = 4.0;
  // TODO(Jinyun) reduce or refactor below configuration numbers
  const double max_s = std::min(50.0, stop_distance);
  for (double t = 2.0; t <= kMaxT; t += 0.5) {
    for (double s = 0.0; s < max_s; s += 0.5) {
      QuinticPolynomialCurve1d curve(0.0, init_speed, init_acc, s, 0.0, 0.0, t);
      if (!IsValidProfile(curve)) {
        continue;
      }
      constexpr double kUnitT = 0.02;
      SpeedData speed_data;
      for (double curve_t = 0.0; curve_t <= t; curve_t += kUnitT) {
        const double curve_s = curve.Evaluate(0, curve_t);
        const double curve_v = curve.Evaluate(1, curve_t);
        const double curve_a = curve.Evaluate(2, curve_t);
        const double curve_da = curve.Evaluate(3, curve_t);
        speed_data.AppendSpeedPoint(curve_s, curve_t, curve_v, curve_a,
                                    curve_da);
      }
      FillEnoughSpeedPoints(&speed_data);
      return speed_data;
    }
  }
  return SpeedData();
}

SpeedData SpeedProfileGenerator::GenerateStopProfileFromPolynomial(
    const double init_speed, const double init_acc) {
  AERROR << "Slowing down the car with polynomial.";
  constexpr double kMaxT = 4.0;
  for (double t = 2.0; t <= kMaxT; t += 0.5) {
    for (double s = 0.0;
         s < std::min(50.0, EgoInfo::Instance()->front_clear_distance() - 0.3);
         s += 1.0) {
      QuinticPolynomialCurve1d curve(0.0, init_speed, init_acc, s, 0.0, 0.0, t);
      if (!IsValidProfile(curve)) {
        continue;
      }
      constexpr double kUnitT = 0.02;
      SpeedData speed_data;
      for (double curve_t = 0.0; curve_t <= t; curve_t += kUnitT) {
        const double curve_s = curve.Evaluate(0, curve_t);
        const double curve_v = curve.Evaluate(1, curve_t);
        const double curve_a = curve.Evaluate(2, curve_t);
        const double curve_da = curve.Evaluate(3, curve_t);
        speed_data.AppendSpeedPoint(curve_s, curve_t, curve_v, curve_a,
                                    curve_da);
      }
      FillEnoughSpeedPoints(&speed_data);
      return speed_data;
    }
  }
  return SpeedData();
}

bool SpeedProfileGenerator::IsValidProfile(
    const QuinticPolynomialCurve1d& curve) {
  for (double evaluate_t = 0.1; evaluate_t <= curve.ParamLength();
       evaluate_t += 0.2) {
    const double v = curve.Evaluate(1, evaluate_t);
    const double a = curve.Evaluate(2, evaluate_t);
    constexpr double kEpsilon = 1e-3;
    if (v < -kEpsilon || a < -5.0) {
      return false;
    }
  }
  return true;
}

SpeedData SpeedProfileGenerator::GenerateFixedDistanceCreepProfile(
    const double distance, const double max_speed) {
  constexpr double kConstDeceleration = -0.8;  // (~3sec to fully stop)
  constexpr double kProceedingSpeed = 2.23;    // (5mph proceeding speed)
  const double proceeding_speed = std::fmin(max_speed, kProceedingSpeed);
  const double distance_to_start_deceleration =
      proceeding_speed * proceeding_speed / kConstDeceleration / 2;
  bool is_const_deceleration_mode = distance < distance_to_start_deceleration;

  double a = kConstDeceleration;
  double t = 0.0;
  double s = 0.0;
  double v = proceeding_speed;

  constexpr double kDeltaT = 0.1;

  SpeedData speed_data;
  while (s < distance && v > 0) {
    if (is_const_deceleration_mode) {
      speed_data.AppendSpeedPoint(s, t, v, a, 0.0);
      t += kDeltaT;
      double v_new = std::max(0.0, v + a * t);
      s += kDeltaT * (v + v_new) / 2;
      v = v_new;
    } else {
      speed_data.AppendSpeedPoint(s, t, v, 0.0, 0.0);
      t += kDeltaT;
      s += kDeltaT * v;
      if (distance - s < distance_to_start_deceleration)
        is_const_deceleration_mode = true;
    }
  }

  return speed_data;
}

}  // namespace planning
}  // namespace apollo
