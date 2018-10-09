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

#pragma once

#include <memory>
#include <vector>

#include "qpOASES.hpp"

#include "modules/common/math/qp_solver/qp_solver.h"
#include "modules/planning/math/smoothing_spline/spline_1d_solver.h"

namespace apollo {
namespace planning {

class ActiveSetSpline1dSolver : public Spline1dSolver {
 public:
  ActiveSetSpline1dSolver(const std::vector<double>& x_knots,
                          const uint32_t order)
      : Spline1dSolver(x_knots, order) {}

  bool Solve() override;

 private:
  std::unique_ptr<::qpOASES::SQProblem> sqp_solver_;
};

}  // namespace planning
}  // namespace apollo
