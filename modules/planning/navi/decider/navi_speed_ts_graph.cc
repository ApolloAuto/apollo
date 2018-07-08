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
#include <cmath>

#include "glog/logging.h"

#include "modules/common/log.h"
#include "modules/common/math/math_utils.h"

namespace apollo {
namespace planning {

using apollo::common::ErrorCode;
using apollo::common::Status;
using apollo::common::math::Clamp;

namespace {
constexpr double kDefaultSStep = 1.0;
constexpr double kDefaultSMax = 2.0;
constexpr double kDefaultSafeDistanceRatio = 1.0;
constexpr double kDefaultSafeDistanceBase = 2.0;
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
  Reset(kDefaultSStep, kDefaultSMax, [](double v) {
    return kDefaultSafeDistanceRatio * v + kDefaultSafeDistanceBase;
  });
}

void NaviSpeedTsGraph::Reset(
    double s_step, double s_max,
    const std::function<double(double v)>& get_safe_distance) {
  CHECK_GT(s_step, 0.0);
  CHECK_GE(s_max, s_step);

  s_step_ = s_step;
  get_safe_distance_ = get_safe_distance;

  auto point_num = (std::size_t)((s_max + s_step_) / s_step_);
  constraints_.resize(point_num);
}

double NaviSpeedTsGraph::Step() const { return s_step_; }

std::size_t NaviSpeedTsGraph::PointNum() const { return constraints_.size(); }

void NaviSpeedTsGraph::UpdateConstraints(
    const NaviSpeedTsConstraints& constraints) {
  CheckConstraints(constraints);

  for (auto& pc : constraints_) CombineConstraints(constraints, &pc);
}

void NaviSpeedTsGraph::UpdatePointConstraints(
    double s, const NaviSpeedTsConstraints& constraints) {
  CHECK_GE(s, 0.0);
  CheckConstraints(constraints);

  auto idx = (std::size_t)(s / s_step_);
  CombineConstraints(constraints, &constraints_[idx]);
}

void NaviSpeedTsGraph::UpdateObstacleConstraints(double distance, double v) {
  auto s = 0.0;
  for (auto& pc : constraints_) {
    auto t = (s - distance) / v;
    if (t >= 0.0) {
      NaviSpeedTsConstraints constraints;
      constraints.t_min = t;
      CombineConstraints(constraints, &pc);
    }

    s += s_step_;
  }
}

Status NaviSpeedTsGraph::Solve(double start_v, double start_a, double start_da,
                               std::vector<NaviSpeedTsPoint>* points) {
  CHECK_NOTNULL(points);

  points->resize(constraints_.size());

  // compute the first point
  auto& point = (*points)[0];
  point.s = 0.0;
  point.t = 0.0;
  point.v = std::abs(start_v);
  point.a = start_a;
  point.da = start_da;

  // compute the remaining points
  for (size_t i = 1; i < points->size(); i++) {
    const auto& prev = (*points)[i - 1];
    const auto& constraints = constraints_[i];
    auto& point = (*points)[i];

    // compute v_max base on v_max and a_max
    auto v_max = constraints.v_max;
    auto a_max = constraints.a_max;
    v_max = std::min(std::sqrt(prev.v * prev.v + 2 * a_max * s_step_), v_max);

    // compute t_min base on v_max
    auto t_min = std::max(prev.t, constraints.t_min);
    t_min = std::max(prev.t + s_step_ / v_max, t_min);

    // compute t_max base on b_max
    auto t_max = std::numeric_limits<double>::infinity();
    auto b_max = constraints.b_max;
    if (prev.v * prev.v / (2 * b_max) > s_step_)
      t_max =
          prev.t +
          (prev.v - std::sqrt(prev.v * prev.v - 2 * b_max * s_step_)) / b_max;

    // if t_max < t_min
    if (t_max < t_min) {
      AERROR << "failure to satisfy the constraints.";
      points->resize(i);
      return Status(ErrorCode::PLANNING_ERROR,
                    "failure to satisfy the constraints.");
    }

    // compute v_preffered
    auto v_preffered = constraints.v_preffered;
    if (v_preffered > prev.v) {
      auto a_preffered = constraints.a_preffered;
      v_preffered = std::min(
          std::sqrt(prev.v * prev.v + 2 * a_preffered * s_step_), v_preffered);
    } else if (v_preffered < prev.v) {
      auto b_preffered = constraints.b_preffered;
      if ((prev.v * prev.v - v_preffered * v_preffered) >
          2 * b_preffered * s_step_)
        v_preffered = std::sqrt(prev.v * prev.v - 2 * b_preffered * s_step_);
    }

    // compute t_preffered base on v_preffered and safe distance
    auto t_preffered = prev.t + s_step_ / v_preffered;
    auto distance = get_safe_distance_(prev.v);
    auto s = prev.s + s_step_;
    auto d_idx = (s + distance) / s_step_;
    if (d_idx < constraints_.size()) {
      const auto& d_constraints = constraints_[d_idx];
      t_preffered = std::max(d_constraints.t_min, t_preffered);
    } else {
      t_preffered = std::numeric_limits<double>::infinity();
    }

    point.s = s;
    point.t = Clamp(t_preffered, t_min, t_max);
    auto dt = point.t - prev.t;
    // TODO(all): if v < 0
    point.v = std::max(2 * s_step_ / dt - prev.v, 0.0);
    point.a = (point.v - prev.v) / dt;
    point.da = (point.a - prev.a) / dt;

    // if t is infinity
    if (std::isinf(point.t)) {
      points->resize(i + 1);
      break;
    }
  }

  return Status::OK();
}

}  // namespace planning
}  // namespace apollo
