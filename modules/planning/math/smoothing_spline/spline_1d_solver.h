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

#include <vector>

#include "Eigen/Core"

#include "modules/common/math/qp_solver/qp_solver.h"
#include "modules/planning/math/smoothing_spline/spline_1d.h"
#include "modules/planning/math/smoothing_spline/spline_1d_constraint.h"
#include "modules/planning/math/smoothing_spline/spline_1d_kernel.h"
#include "modules/planning/proto/qp_problem.pb.h"

namespace apollo {
namespace planning {

class Spline1dSolver {
 public:
  Spline1dSolver(const std::vector<double>& x_knots, const uint32_t order)
      : spline_(x_knots, order),
        constraint_(x_knots, order),
        kernel_(x_knots, order) {}

  virtual ~Spline1dSolver() = default;

  virtual void Reset(const std::vector<double>& x_knots, const uint32_t order) {
    spline_ = Spline1d(x_knots, order);
    constraint_ = Spline1dConstraint(x_knots, order);
    kernel_ = Spline1dKernel(x_knots, order);
  }

  virtual Spline1dConstraint* mutable_spline_constraint() {
    return &constraint_;
  }

  virtual Spline1dKernel* mutable_spline_kernel() { return &kernel_; }

  virtual bool Solve() = 0;

  // output
  virtual const Spline1d& spline() const { return spline_; }

  // convert qp problem to proto
  void GenerateProblemProto(QuadraticProgrammingProblem* const qp_proto) const;

 protected:
  void ConvertMatrixXdToProto(const Eigen::MatrixXd& matrix,
                              QPMatrix* const proto) const;

 protected:
  Spline1d spline_;
  Spline1dConstraint constraint_;
  Spline1dKernel kernel_;

  int last_num_constraint_ = 0;
  int last_num_param_ = 0;
  bool last_problem_success_ = false;
};

}  // namespace planning
}  // namespace apollo
