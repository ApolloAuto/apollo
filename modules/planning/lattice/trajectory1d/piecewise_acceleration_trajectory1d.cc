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

#include "modules/planning/lattice/trajectory1d/piecewise_acceleration_trajectory1d.h"

#include <algorithm>

#include "modules/common/log.h"
#include "modules/common/math/linear_interpolation.h"
#include "modules/common/util/string_util.h"
#include "modules/planning/common/planning_gflags.h"

namespace apollo {
namespace planning {

ConstantAccelerationTrajectory1d::ConstantAccelerationTrajectory1d(
    const double start_s, const double start_v) {
  s_.push_back(start_s);
  v_.push_back(start_v);
  a_.push_back(0.0);
  t_.push_back(0.0);
}

void ConstantAccelerationTrajectory1d::AppendSegment(
    const double a, const double t_duration) {
  double s0 = s_.back();
  double v0 = v_.back();
  double t0 = t_.back();

  double v1 = v0 + a * t_duration;
  CHECK(v1 >= -FLAGS_lattice_epsilon);

  double delta_s = (v0 + v1) * t_duration * 0.5;
  double s1 = s0 + delta_s;
  double t1 = t0 + t_duration;

  CHECK(s1 >= s0 - FLAGS_lattice_epsilon);
  s1 = std::max(s1, s0);
  s_.push_back(s1);
  v_.push_back(v1);
  a_.push_back(a);
  t_.push_back(t1);
}

void ConstantAccelerationTrajectory1d::PopSegment() {
  if (a_.size() > 0) {
    s_.pop_back();
    v_.pop_back();
    a_.pop_back();
    t_.pop_back();
  }
}

double ConstantAccelerationTrajectory1d::ParamLength() const {
  CHECK_GT(t_.size(), 1);
  return t_.back() - t_.front();
}

std::string ConstantAccelerationTrajectory1d::ToString() const {
  return apollo::common::util::StrCat(apollo::common::util::PrintIter(s_, "\t"),
                                      apollo::common::util::PrintIter(t_, "\t"),
                                      apollo::common::util::PrintIter(v_, "\t"),
                                      apollo::common::util::PrintIter(a_, "\t"),
                                      "\n");
}

double ConstantAccelerationTrajectory1d::Evaluate(const std::uint32_t order,
                                                  const double param) const {
  CHECK_GT(t_.size(), 1);
  CHECK(t_.front() <= param && param <= t_.back());

  switch (order) {
    case 0:
      return Evaluate_s(param);
    case 1:
      return Evaluate_v(param);
    case 2:
      return Evaluate_a(param);
    case 3:
      return Evaluate_j(param);
  }
  return 0.0;
}

double ConstantAccelerationTrajectory1d::Evaluate_s(const double t) const {
  auto it_lower = std::lower_bound(t_.begin(), t_.end(), t);
  auto index = std::distance(t_.begin(), it_lower);

  double s0 = s_[index - 1];
  double v0 = v_[index - 1];
  double t0 = t_[index - 1];

  double v1 = v_.back();
  double t1 = t_.back();

  double v = common::math::lerp(v0, t0, v1, t1, t);
  double s = (v0 + v) * (t - t0) * 0.5 + s0;
  return s;
}

double ConstantAccelerationTrajectory1d::Evaluate_v(const double t) const {
  auto it_lower = std::lower_bound(t_.begin(), t_.end(), t);
  auto index = std::distance(t_.begin(), it_lower);

  double v0 = v_[index - 1];
  double t0 = t_[index - 1];

  double v1 = v_.back();
  double t1 = t_.back();

  double v = apollo::common::math::lerp(v0, t0, v1, t1, t);
  return v;
}

double ConstantAccelerationTrajectory1d::Evaluate_a(const double t) const {
  auto it_lower = std::lower_bound(t_.begin(), t_.end(), t);
  auto index = std::distance(t_.begin(), it_lower);
  return a_[index - 1];
}

double ConstantAccelerationTrajectory1d::Evaluate_j(const double t) const {
  return 0.0;
}

std::array<double, 4> ConstantAccelerationTrajectory1d::Evaluate(
    const double t) const {
  CHECK_GT(t_.size(), 1);
  CHECK(t_.front() <= t && t <= t_.back());

  auto it_lower = std::lower_bound(t_.begin(), t_.end(), t);
  auto index = std::distance(t_.begin(), it_lower);

  double s0 = s_[index - 1];
  double v0 = v_[index - 1];
  double t0 = t_[index - 1];

  double v1 = v_.back();
  double t1 = t_.back();

  double v = common::math::lerp(v0, t0, v1, t1, t);
  double s = (v0 + v) * (t - t0) * 0.5 + s0;

  double a = a_[index - 1];
  double j = 0.0;

  return {{s, v, a, j}};
}

}  // namespace planning
}  // namespace apollo
