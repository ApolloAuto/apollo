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
 * @file
 * @brief This file provides the implementation of the class "NaviSpeedTsGraph".
 */

#include "modules/planning/navi/decider/navi_speed_ts_graph.h"

#include <algorithm>

#include "cyber/common/log.h"
#include "modules/common/math/math_utils.h"

namespace apollo {
namespace planning {

using apollo::common::ErrorCode;
using apollo::common::Status;
using apollo::common::math::Clamp;

namespace {
constexpr double kDoubleEpsilon = 1.0e-6;
constexpr double kDefaultSStep = 1.0;
constexpr double kDefaultSMax = 2.0;
constexpr double kDefaultSafeDistanceRatio = 1.0;
constexpr double kDefaultSafeDistanceBase = 2.0;
constexpr double kSafeDistanceSmooth = 3.0;
constexpr double kFollowSpeedSmooth = 0.25;
constexpr double kInfinityValue = 1.0e8;
}  // namespace

static void CheckConstraints(const NaviSpeedTsConstraints& constraints) {
  CHECK_GE(constraints.t_min, 0.0);
  CHECK_GE(constraints.v_max, 0.0);
  CHECK_GE(constraints.v_max, constraints.v_preffered);
  CHECK_GE(constraints.a_max, 0.0);
  CHECK_GE(constraints.a_max, constraints.a_preffered);
  CHECK_GE(constraints.b_max, 0.0);
  CHECK_GE(constraints.b_max, constraints.b_preffered);
  CHECK_GE(constraints.da_max, 0.0);
  CHECK_GE(constraints.da_max, constraints.da_preffered);
}

static void CombineConstraints(const NaviSpeedTsConstraints& constraints,
                               NaviSpeedTsConstraints* dst) {
  dst->t_min = std::max(constraints.t_min, dst->t_min);
  dst->v_max = std::min(constraints.v_max, dst->v_max);
  dst->v_preffered = std::min(constraints.v_preffered, dst->v_preffered);
  dst->a_max = std::min(constraints.a_max, dst->a_max);
  dst->a_preffered = std::min(constraints.a_preffered, dst->a_preffered);
  dst->b_max = std::min(constraints.b_max, dst->b_max);
  dst->b_preffered = std::min(constraints.b_preffered, dst->b_preffered);
  dst->da_max = std::min(constraints.da_max, dst->da_max);
  dst->da_preffered = std::min(constraints.da_preffered, dst->da_preffered);
}

NaviSpeedTsGraph::NaviSpeedTsGraph() {
  Reset(kDefaultSStep, kDefaultSMax, 0.0, 0.0, 0.0);
}

void NaviSpeedTsGraph::Reset(double s_step, double s_max, double start_v,
                             double start_a, double start_da) {
  CHECK_GT(s_step, 0.0);
  CHECK_GE(s_max, s_step);
  CHECK_GE(start_v, 0.0);

  s_step_ = s_step;
  start_v_ = start_v;
  start_a_ = start_a;
  start_da_ = start_da;

  auto point_num = (size_t)((s_max + s_step_) / s_step_);
  constraints_.clear();
  constraints_.resize(point_num);
}

void NaviSpeedTsGraph::UpdateConstraints(
    const NaviSpeedTsConstraints& constraints) {
  CheckConstraints(constraints);

  for (auto& pc : constraints_) {
    CombineConstraints(constraints, &pc);
  }
}

void NaviSpeedTsGraph::UpdateRangeConstraints(
    double start_s, double end_s, const NaviSpeedTsConstraints& constraints) {
  CHECK_GE(start_s, 0.0);
  CHECK_GE(end_s, start_s);
  CheckConstraints(constraints);

  auto start_idx = (size_t)(std::floor(start_s / s_step_));
  auto end_idx = (size_t)(std::ceil(end_s / s_step_));
  if (start_idx == end_idx) {
    CombineConstraints(constraints, &constraints_[start_idx]);
  } else {
    for (size_t i = start_idx; i < end_idx && i < constraints_.size(); i++)
      CombineConstraints(constraints, &constraints_[i]);
  }
}

void NaviSpeedTsGraph::UpdateObstacleConstraints(double distance,
                                                 double safe_distance,
                                                 double following_accel_ratio,
                                                 double v,
                                                 double cruise_speed) {
  CHECK_GE(distance, 0.0);
  CHECK_GE(safe_distance, 0.0);
  CHECK_GT(following_accel_ratio, 0.0);

  // smooth obstacle following
  if (distance > safe_distance &&
      distance - safe_distance < kSafeDistanceSmooth &&
      std::abs(v - start_v_) < kFollowSpeedSmooth) {
    distance = safe_distance;
    v = start_v_;
  }

  // TODO(all): if v < 0
  v = std::max(v, 0.0);

  // update t_min
  double s = 0.0;
  for (auto& pc : constraints_) {
    auto t = (s - distance) / v;
    if (t >= 0.0) {
      NaviSpeedTsConstraints constraints;
      constraints.t_min = t;
      CombineConstraints(constraints, &pc);
    }
    s += s_step_;
  }

  // update v_preffered
  auto od = distance - safe_distance;
  auto v0 = start_v_;
  auto v1 = v;
  auto r0 = (distance - safe_distance) / (distance + safe_distance);
  auto vm = std::max(cruise_speed * r0, 0.0) +
            (1.0 + following_accel_ratio * r0) * v1;
  auto ra = (v1 - vm) * (v1 - vm) * (v1 + v0 - 2.0 * vm) / (od * od);
  auto rb = (v1 - vm) * (v1 + 2.0 * v0 - 3.0 * vm) / od;
  auto rc = v0;
  auto t1 = -1.0 * od / (v1 - vm);
  auto s1 = ra * t1 * t1 * t1 + rb * t1 * t1 + rc * t1;

  double t;
  double prev_v;
  bool first = true;
  for (auto& pc : constraints_) {
    NaviSpeedTsConstraints constraints;

    if (first) {
      first = false;
      auto cur_v = rc;
      t = 0.0;
      s = 0.0;
      prev_v = cur_v;
      constraints.v_preffered = cur_v;
    } else if (s <= s1 && t <= t1) {
      auto a = 6.0 * ra * t + 2.0 * rb;
      t += (std::sqrt(prev_v * prev_v + 2.0 * a * s_step_) - prev_v) / a;
      auto cur_v = 3.0 * ra * t * t + 2.0 * rb * t + rc;
      s += s_step_;
      prev_v = cur_v;
      constraints.v_preffered = std::max(cur_v, 0.0);
    } else {
      auto cur_v = v;
      t += 2.0 * s_step_ / (prev_v + cur_v);
      s += s_step_;
      prev_v = cur_v;
      constraints.v_preffered = cur_v;
    }

    CombineConstraints(constraints, &pc);
  }
}

Status NaviSpeedTsGraph::Solve(std::vector<NaviSpeedTsPoint>* output) {
  CHECK_NOTNULL(output);

  // make constraints of the first point
  auto& constraints = constraints_[0];
  constraints.v_max = start_v_;
  constraints.v_preffered = start_v_;
  constraints.a_max = start_a_;
  constraints.a_preffered = start_a_;
  constraints.da_max = start_da_;
  constraints.da_preffered = start_da_;

  // preprocess v_max base on b_max
  for (ssize_t i = constraints_.size() - 2; i >= 0; i--) {
    const auto& next = constraints_[i + 1];
    auto& cur = constraints_[i];
    cur.v_max =
        std::min(std::sqrt(next.v_max * next.v_max + 2 * next.b_max * s_step_),
                 cur.v_max);
    cur.v_preffered = std::min(cur.v_max, cur.v_preffered);
  }

  // preprocess v_max base on a_max
  for (size_t i = 1; i < constraints_.size(); i++) {
    const auto& prev = constraints_[i - 1];
    auto& cur = constraints_[i];
    cur.v_max =
        std::min(std::sqrt(prev.v_max * prev.v_max + 2 * cur.a_max * s_step_),
                 cur.v_max);
    cur.v_preffered = std::min(cur.v_max, cur.v_preffered);
  }

  // preprocess v_preffered base on b_preffered
  for (ssize_t i = constraints_.size() - 2; i >= 0; i--) {
    const auto& next = constraints_[i + 1];
    auto& cur = constraints_[i];
    cur.v_preffered = std::min(std::sqrt(next.v_preffered * next.v_preffered +
                                         2 * next.b_preffered * s_step_),
                               cur.v_preffered);
  }

  // preprocess v_preffered base on a_preffered
  for (size_t i = 1; i < constraints_.size(); i++) {
    const auto& prev = constraints_[i - 1];
    auto& cur = constraints_[i];
    cur.v_preffered = std::min(std::sqrt(prev.v_preffered * prev.v_preffered +
                                         2 * cur.a_preffered * s_step_),
                               cur.v_preffered);
  }

  auto& points = *output;
  points.resize(constraints_.size());

  // compute the first point
  auto& point = points[0];
  point.s = 0.0;
  point.t = 0.0;
  point.v = start_v_;
  point.a = start_a_;
  point.da = start_da_;

  // compute the remaining points
  for (size_t i = 1; i < points.size(); i++) {
    const auto& prev = points[i - 1];
    const auto& constraints = constraints_[i];
    auto& cur = points[i];

    // compute t_min base on v_max
    auto t_min = std::max(prev.t, constraints.t_min);
    auto v_max = constraints.v_max;
    t_min = std::max(prev.t + 2.0 * s_step_ / (prev.v + v_max), t_min);

    // compute t_max base on b_max
    auto t_max = std::numeric_limits<double>::infinity();
    auto b_max = constraints.b_max;
    auto r0 = prev.v * prev.v - 2 * b_max * s_step_;
    if (r0 > 0.0) {
      t_max = prev.t + (prev.v - std::sqrt(r0)) / b_max;
    }
    // if t_max < t_min
    if (t_max < t_min) {
      AERROR << "failure to satisfy the constraints.";
      return Status(ErrorCode::PLANNING_ERROR,
                    "failure to satisfy the constraints.");
    }

    // compute t_preffered base on v_preffered
    auto v_preffered = constraints.v_preffered;
    auto t_preffered = prev.t + 2 * s_step_ / (prev.v + v_preffered);

    cur.s = prev.s + s_step_;
    cur.t = Clamp(t_preffered, t_min, t_max);
    auto dt = cur.t - prev.t;
    cur.v = std::max(2.0 * s_step_ / dt - prev.v, 0.0);

    // if t is infinity
    if (std::isinf(cur.t)) {
      points.resize(i + 1);
      break;
    }
  }

  for (size_t i = 1; i < points.size() - 1; i++) {
    const auto& prev = points[i - 1];
    const auto& next = points[i + 1];
    auto& cur = points[i];
    auto ds = next.s - prev.s;
    auto dt = next.t - prev.t;
    cur.v = ds / dt;
  }

  auto& first = points[0];
  const auto& second = points[1];
  first.a = (second.v - first.v) / (2.0 * (second.t - first.t));

  for (size_t i = 1; i < points.size() - 1; i++) {
    const auto& prev = points[i - 1];
    const auto& next = points[i + 1];
    auto& cur = points[i];
    auto dv = next.v - prev.v;
    auto dt = next.t - prev.t;
    cur.a = dv / dt;
  }

  for (size_t i = 1; i < points.size() - 1; i++) {
    const auto& prev = points[i - 1];
    const auto& next = points[i + 1];
    auto& cur = points[i];
    auto da = next.a - prev.a;
    auto dt = next.t - prev.t;
    cur.da = da / dt;
  }

  for (size_t i = 0; i < points.size(); i++) {
    auto& point = points[i];
    point.s = std::min(kInfinityValue, point.s);
    point.t = std::min(kInfinityValue, point.t);
    point.v = std::min(kInfinityValue, point.v);
    point.a = std::min(kInfinityValue, point.a);
    point.da = std::min(kInfinityValue, point.da);

    if (std::abs(point.t - kInfinityValue) < kDoubleEpsilon)
      points.resize(i + 1);
  }

  return Status::OK();
}

}  // namespace planning
}  // namespace apollo
