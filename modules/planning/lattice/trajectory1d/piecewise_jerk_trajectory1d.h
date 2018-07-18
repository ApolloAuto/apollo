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
 **/

#ifndef MODULES_PLANNING_LATTICE_TRAJECTORY1D_PIECEWISE_JERK_TRAJECTORY1D_H_
#define MODULES_PLANNING_LATTICE_TRAJECTORY1D_PIECEWISE_JERK_TRAJECTORY1D_H_

#include "modules/planning/math/curve1d/curve1d.h"

namespace apollo {
namespace planning {

class PiecewiseJerkTrajectory1d : public Curve1d {
 public:
  PiecewiseJerkTrajectory1d();

  virtual ~PiecewiseJerkTrajectory1d();

  double Evaluate(const std::uint32_t order,
                  const double param) const;

  double ParamLength() const;

  std::string ToString() const;
};

} // namespace planning
} // namespace apollo

#endif /* MODULES_PLANNING_LATTICE_TRAJECTORY1D_PIECEWISE_JERK_TRAJECTORY1D_H_ */
