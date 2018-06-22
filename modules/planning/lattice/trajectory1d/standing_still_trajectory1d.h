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

#ifndef MODULES_PLANNING_LATTICE_STANDING_STILL_TRAJECTORY1D_H_
#define MODULES_PLANNING_LATTICE_STANDING_STILL_TRAJECTORY1D_H_

#include <stdint.h>
#include <string>
#include "modules/planning/math/curve1d/curve1d.h"

namespace apollo {
namespace planning {

class StandingStillTrajectory1d : public Curve1d {
 public:
  StandingStillTrajectory1d(const double p, const double duration);

  virtual ~StandingStillTrajectory1d() = default;

  double ParamLength() const override;

  std::string ToString() const override;

  double Evaluate(const std::uint32_t order, const double param) const override;

 private:
  double Evaluate_s(const double t) const;

  double Evaluate_v(const double t) const;

  double Evaluate_a(const double t) const;

  double Evaluate_j(const double t) const;

 private:
  double fixed_position_;

  double duration_;
};

}  // namespace planning
}  // namespace apollo

#endif  // MODULES_PLANNING_LATTICE_STANDING_STILL_TRAJECTORY1D_H_
