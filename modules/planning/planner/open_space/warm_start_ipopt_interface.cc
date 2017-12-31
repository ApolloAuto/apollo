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

#include "modules/common/math/math_utils.h"
#include "modules/planning/common/planning_gflags.h"

namespace apollo {
namespace planning {

constexpr std::size_t N = 80;

WarmUpIPOPTInterface::WarmUpIPOPTInterface() {}

void WarmUpIPOPTInterface::get_optimization_results() const {}

bool WarmUpIPOPTInterface::get_nlp_info(int& n, int& m, int& nnz_jac_g,
                                        int& nnz_h_lag,
                                        IndexStyleEnum& index_style) {
  // number of variables
  n = 0;

  // number of constraints

  m = 0;

  // number of nonzero hessian and lagrangian.
  nnz_jac_g = 0;

  nnz_h_lag = 0;

  index_style = IndexStyleEnum::C_STYLE;
  return true;
}

bool WarmUpIPOPTInterface::get_bounds_info(int n, double* x_l, double* x_u,
                                           int m, double* g_l, double* g_u) {
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
                                  double* values) {}

void WarmUpIPOPTInterface::set_start_point() {}

void WarmUpIPOPTInterface::set_end_point() {}

}  // namespace planning
}  // namespace apollo
