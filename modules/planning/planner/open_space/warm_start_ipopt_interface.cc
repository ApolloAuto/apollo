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

/*
 * warm_start_ipopt_interface.cc
 */

#include "modules/planning/planner/open_space/warm_start_ipopt_interface.h"

#include <math.h>
#include <utility>

#include "modules/common/log.h"
#include "modules/common/math/math_utils.h"
#include "modules/planning/common/planning_gflags.h"

namespace apollo {
namespace planning {

constexpr std::size_t N = 80;

WarmUpIPOPTInterface::WarmUpIPOPTInterface(int num_of_variables,
                                           int num_of_constraints, int horizon)
    : num_of_variables_(num_of_variables),
      num_of_constraints_(num_of_constraints),
      horizon_(horizon) {}

void WarmUpIPOPTInterface::get_optimization_results() const {}

bool WarmUpIPOPTInterface::get_nlp_info(int& n, int& m, int& nnz_jac_g,
                                        int& nnz_h_lag,
                                        IndexStyleEnum& index_style) {
  // number of variables
  n = num_of_variables_;

  // number of constraints

  m = num_of_constraints_;

  // number of nonzero hessian and lagrangian.

  // TODO(QiL) : Update nnz_jac_g;
  nnz_jac_g = 0;

  // TOdo(QiL) : Update nnz_h_lag;
  nnz_h_lag = 0;

  index_style = IndexStyleEnum::C_STYLE;
  return true;
}

bool WarmUpIPOPTInterface::get_bounds_info(int n, double* x_l, double* x_u,
                                           int m, double* g_l, double* g_u) {
  // here, the n and m we gave IPOPT in get_nlp_info are passed back to us.
  // If desired, we could assert to make sure they are what we think they are.
  CHECK(n == num_of_variables_) << "num_of_variables_ mismatch, n: " << n
                                << ", num_of_variables_: " << num_of_variables_;
  CHECK(m == num_of_constraints_)
      << "num_of_constraints_ mismatch, n: " << n
      << ", num_of_constraints_: " << num_of_constraints_;

  for (std::size_t i = 0; i < horizon_; ++i) {
    std::size_t index = i * n;

    //
    /*
        // theta
        x_l[index] = theta_lower;
        x_u[index] = theta_upper;

        // kappa
        x_l[index + 1] = kappa_lower;
        x_u[index + 1] = kappa_upper;

        // dkappa
        x_l[index + 2] = dkappa_lower;
        x_u[index + 2] = dkappa_upper;

        // x
        x_l[index + 3] = x_lower;
        x_u[index + 3] = x_upper;

        // y
        x_l[index + 4] = y_lower;
        x_u[index + 4] = y_upper;
        */
  }

  // we have one equality constraint, so we set the bounds on this constraint
  // to be equal (and zero).
  g_l[0] = g_u[0] = 0.0;

  return true;
}

bool WarmUpIPOPTInterface::eval_g(int n, const double* x, bool new_x, int m,
                                  double* g) {
  return true;
}

bool WarmUpIPOPTInterface::eval_jac_g(int n, const double* x, bool new_x, int m,
                                      int nele_jac, int* iRow, int* jCol,
                                      double* values) {
  return true;
}

bool WarmUpIPOPTInterface::eval_h(int n, const double* x, bool new_x,
                                  double obj_factor, int m,
                                  const double* lambda, bool new_lambda,
                                  int nele_hess, int* iRow, int* jCol,
                                  double* values) {
  return true;
}

void WarmUpIPOPTInterface::set_start_point() {}

void WarmUpIPOPTInterface::set_end_point() {}

}  // namespace planning
}  // namespace apollo
