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
 * @file spline_2d_solver.h
 **/

#ifndef MODULES_PLANNING_SMOOTHING_SPLINE_SPLINE_2D_SOLVER_H_
#define MODULES_PLANNING_SMOOTHING_SPLINE_SPLINE_2D_SOLVER_H_

#include <qpOASES.hpp>

#include <memory>
#include <vector>

#include "modules/common/math/qp_solver/qp_solver.h"
#include "modules/planning/math/smoothing_spline/spline_2d.h"
#include "modules/planning/math/smoothing_spline/spline_2d_constraint.h"
#include "modules/planning/math/smoothing_spline/spline_2d_kernel.h"

namespace apollo {
namespace planning {

class Spline2dSolver {
 public:
  Spline2dSolver(const std::vector<double>& t_knots, const uint32_t order);

  void Reset(const std::vector<double>& t_knots, const uint32_t order);

  // customize setup
  Spline2dConstraint* mutable_constraint();
  Spline2dKernel* mutable_kernel();
  Spline2d* mutable_spline();

  // solve
  bool Solve();

  // extract
  const Spline2d& spline() const;

 private:
  Spline2d spline_;
  Spline2dKernel kernel_;
  Spline2dConstraint constraint_;
  std::unique_ptr<::qpOASES::SQProblem> sqp_solver_;

  int last_num_constraint_ = 0;
  int last_num_param_ = 0;
  bool last_problem_success_ = false;
};

}  // namespace planning
}  // namespace apollo

#endif  // MODULES_PLANNING_SMOOTHING_SPLINE_SPLINE_2D_SOLVER_H_
