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

#include "modules/planning/common/trajectory1d/constant_deceleration_trajectory1d.h"

#include <cmath>

#include "cyber/common/log.h"
#include "modules/planning/common/planning_gflags.h"

namespace apollo {
namespace planning {

ConstantDecelerationTrajectory1d::ConstantDecelerationTrajectory1d(
    const double init_s, const double init_v, const double a)
    : init_s_(init_s), init_v_(init_v), deceleration_(-a) {
  if (init_v_ < -FLAGS_numerical_epsilon) {
    AERROR << "negative init v = " << init_v_;
  }
  init_v_ = std::fabs(init_v_);
  CHECK(deceleration_ > 0.0);
  end_t_ = init_v_ / deceleration_;
  end_s_ = init_v_ * init_v_ / (2.0 * deceleration_) + init_s_;
}

double ConstantDecelerationTrajectory1d::Evaluate_s(const double t) const {
  if (t < end_t_) {
    double curr_v = init_v_ - deceleration_ * t;
    double delta_s = (curr_v + init_v_) * t * 0.5;
    return init_s_ + delta_s;
  } else {
    return end_s_;
  }
}

double ConstantDecelerationTrajectory1d::Evaluate_v(const double t) const {
  if (t < end_t_) {
    return init_v_ - deceleration_ * t;
  } else {
    return 0.0;
  }
}

double ConstantDecelerationTrajectory1d::Evaluate_a(const double t) const {
  if (t < end_t_) {
    return -deceleration_;
  } else {
    return 0.0;
  }
}

double ConstantDecelerationTrajectory1d::Evaluate_j(const double t) const {
  return 0.0;
}

double ConstantDecelerationTrajectory1d::ParamLength() const { return end_t_; }

std::string ConstantDecelerationTrajectory1d::ToString() const { return ""; }

double ConstantDecelerationTrajectory1d::Evaluate(const std::uint32_t order,
                                                  const double param) const {
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

}  // namespace planning
}  // namespace apollo
