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
 * @file : spline_1d_generator.h
 * @brief: piecewise_smoothing_spline (pss) generator class
 *           solve pss by qp algorithm, include adding constraint, adding
 *kernel, and solver solve
 **/

// -- spline config
// 1. knots
// 2. order
//
// -- constraint config
// 1. init point constraint
// 2. inequality bound constraint
//      A. fx bound constraint
//      B. f' bound constraint
//      C. f'' bound constraint
//      D. f''' bound constraint
// 3. equality smooth constraint
//      A. Level of smoothness, linear, upto derivative,
//         upto second order derivative
//
// -- kernel configs
// 1. integrated kernel constraint;
// 2. regularized kernel avoid sigularity;
// 3. other customized kernel (e.g)
//      A. kernel -- ( -s(t_g)^2 ) given end point time, find the
//      B. kernel -- ( sum(l^2_i)) ...
//
// -- solve

#ifndef MODULES_PLANNING_MATH_SMOOTHING_SPLINE_SPLINE_1D_GENERATOR_H_
#define MODULES_PLANNING_MATH_SMOOTHING_SPLINE_SPLINE_1D_GENERATOR_H_

#include <memory>
#include <vector>

#include "qpOASES/include/qpOASES.hpp"

#include "modules/common/math/qp_solver/qp_solver.h"
#include "modules/planning/math/smoothing_spline/spline_1d.h"
#include "modules/planning/math/smoothing_spline/spline_1d_constraint.h"
#include "modules/planning/math/smoothing_spline/spline_1d_kernel.h"

namespace apollo {
namespace planning {

class Spline1dGenerator {
 public:
  Spline1dGenerator(const std::vector<double>& x_knots, const uint32_t order);

  void Reset(const std::vector<double>& x_knots, const uint32_t order);

  // add constraint through pss_constraint
  Spline1dConstraint* mutable_spline_constraint();

  // add kernel through pss_kernel
  Spline1dKernel* mutable_spline_kernel();

  // solve
  bool Solve();

  // output
  const Spline1d& spline() const;

 private:
  Spline1d spline_;
  Spline1dConstraint spline_constraint_;
  Spline1dKernel spline_kernel_;

  std::unique_ptr<::qpOASES::SQProblem> sqp_solver_;

  int last_num_constraint_ = 0;
  int last_num_param_ = 0;
};

}  // namespace planning
}  // namespace apollo

#endif  // MODULES_PLANNING_MATH_SMOOTHING_SPLINE_SPLINE_1D_GENERATOR_H_
