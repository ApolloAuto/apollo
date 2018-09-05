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

#include "modules/common/log.h"
#include "modules/planning/common/frame.h"
#include "modules/planning/common/planning_gflags.h"

namespace apollo {
namespace planning {

using common::SpeedPoint;
using common::SLPoint;
using common::TrajectoryPoint;
using common::math::Vec2d;

std::vector<SpeedPoint> SpeedProfileGenerator::GenerateInitSpeedProfile(
    const TrajectoryPoint& planning_init_point,
    const ReferenceLineInfo* reference_line_info) const {
  std::vector<SpeedPoint> speed_profile;
  const auto* last_frame = FrameHistory::instance()->Latest();
  if (!last_frame) {
    AWARN << "last frame is empty";
    return speed_profile;
  }
  const ReferenceLineInfo* last_reference_line_info =
      last_frame->DriveReferenceLineInfo();
  if (!last_reference_line_info) {
    ADEBUG << "last reference line info is empty";
    return speed_profile;
  }
  if (!reference_line_info->IsStartFrom(*last_reference_line_info)) {
    ADEBUG << "Current reference line is not started previous drived line";
    return speed_profile;
  }
  const auto& last_speed_vector =
      last_reference_line_info->speed_data().speed_vector();

  if (!last_speed_vector.empty()) {
    const auto& last_init_point = last_frame->PlanningStartPoint().path_point();
    Vec2d last_xy_point(last_init_point.x(), last_init_point.y());
    SLPoint last_sl_point;
    if (!last_reference_line_info->reference_line().XYToSL(last_xy_point,
                                                           &last_sl_point)) {
      AERROR << "Fail to transfer xy to sl when init speed profile";
    }

    Vec2d xy_point(planning_init_point.path_point().x(),
                   planning_init_point.path_point().y());
    SLPoint sl_point;
    if (!last_reference_line_info->reference_line().XYToSL(xy_point,
                                                           &sl_point)) {
      AERROR << "Fail to transfer xy to sl when init speed profile";
    }

    double s_diff = sl_point.s() - last_sl_point.s();
    double start_time = 0.0;
    double start_s = 0.0;
    bool is_updated_start = false;
    for (const auto& speed_point : last_speed_vector) {
      if (speed_point.s() < s_diff) {
        continue;
      }
      if (!is_updated_start) {
        start_time = speed_point.t();
        start_s = speed_point.s();
        is_updated_start = true;
      }
      SpeedPoint refined_speed_point;
      refined_speed_point.set_s(speed_point.s() - start_s);
      refined_speed_point.set_t(speed_point.t() - start_time);
      refined_speed_point.set_v(speed_point.v());
      refined_speed_point.set_a(speed_point.a());
      refined_speed_point.set_da(speed_point.da());
      speed_profile.push_back(std::move(refined_speed_point));
    }
  }
  return speed_profile;
}

// a dummy simple hot start
// TODO(All): refine the hotstart speed profile
std::vector<SpeedPoint> SpeedProfileGenerator::GenerateSpeedHotStart(
    const TrajectoryPoint& planning_init_point) const {
  std::vector<SpeedPoint> hot_start_speed_profile;
  double s = 0.0;
  double t = 0.0;
  double v = common::math::Clamp(planning_init_point.v(), 5.0,
                                 FLAGS_planning_upper_speed_limit);
  while (t < FLAGS_trajectory_time_length) {
    SpeedPoint speed_point;
    speed_point.set_s(s);
    speed_point.set_t(t);
    speed_point.set_v(v);

    hot_start_speed_profile.push_back(std::move(speed_point));

    t += FLAGS_trajectory_time_min_interval;
    s += v * FLAGS_trajectory_time_min_interval;
  }
  return hot_start_speed_profile;
}

SpeedData SpeedProfileGenerator::GenerateFallbackSpeedProfile(
    const ReferenceLineInfo& reference_line_info) {
  const double init_v = reference_line_info.AdcPlanningPoint().v();
  const double init_a = reference_line_info.AdcPlanningPoint().a();
  if (init_v > FLAGS_polynomial_speed_fallback_velocity) {
    return GenerateStopProfileFromPolynomial(init_v, init_a);
  } else {
    auto speed_data = GenerateStopProfileFromPolynomial(init_v, init_a);
    if (!speed_data.Empty()) {
      return speed_data;
    }
  }
  return GenerateStopProfile(init_v, init_a);
}

SpeedData SpeedProfileGenerator::GenerateStopProfile(
    const double init_speed, const double init_acc) const {
  AERROR << "Using fallback stopping profile: Slowing down the car!";
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
  return speed_data;
}

SpeedData SpeedProfileGenerator::GenerateStopProfileFromPolynomial(
    const double init_speed, const double init_acc) const {
  AERROR << "Slowing down the car with polynomial.";
  constexpr double kMaxT = 4.0;
  for (double t = 2.0; t <= kMaxT; t += 0.5) {
    for (double s = 0.0; s < 50.0; s += 1.0) {
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
      return speed_data;
    }
  }
  return SpeedData();
}

bool SpeedProfileGenerator::IsValidProfile(
    const QuinticPolynomialCurve1d& curve) const {
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

}  // namespace planning
}  // namespace apollo
