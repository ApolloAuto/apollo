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

#include "modules/planning/common/trajectory1d/standing_still_trajectory1d.h"

namespace apollo {
namespace planning {

StandingStillTrajectory1d::StandingStillTrajectory1d(const double p,
                                                     const double duration)
    : fixed_position_(p), duration_(duration) {}

double StandingStillTrajectory1d::ParamLength() const { return duration_; }

std::string StandingStillTrajectory1d::ToString() const { return ""; }

double StandingStillTrajectory1d::Evaluate(const std::uint32_t order,
                                           const double param) const {
  //  CHECK(param <= duration_);
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

double StandingStillTrajectory1d::Evaluate_s(const double t) const {
  return fixed_position_;
}

double StandingStillTrajectory1d::Evaluate_v(const double t) const {
  return 0.0;
}

double StandingStillTrajectory1d::Evaluate_a(const double t) const {
  return 0.0;
}

double StandingStillTrajectory1d::Evaluate_j(const double t) const {
  return 0.0;
}

}  // namespace planning
}  // namespace apollo
