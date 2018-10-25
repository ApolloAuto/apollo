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

/*
 * @file
 */

#include "modules/planning/open_space/dual_variable_warm_start_ipopt_interface.h"

#include <math.h>
#include <utility>

#include "cyber/common/log.h"
#include "modules/common/configs/vehicle_config_helper.h"
#include "modules/common/math/math_utils.h"
#include "modules/common/util/util.h"
#include "modules/planning/common/planning_gflags.h"

namespace apollo {
namespace planning {

constexpr std::size_t N = 80;

DualVariableWarmStartIPOPTInterface::DualVariableWarmStartIPOPTInterface(
    int num_of_variables, int num_of_constraints, std::size_t horizon, float ts,
    Eigen::MatrixXd x0, Eigen::MatrixXd xf, Eigen::MatrixXd XYbounds)
    : num_of_variables_(num_of_variables),
      num_of_constraints_(num_of_constraints),
      horizon_(horizon),
      ts_(ts),
      x0_(x0),
      xf_(xf),
      XYbounds_(XYbounds) {
  /*
const auto& wheelbase_ = common::VehicleConfigHelper::Instance()
->GetConfig().vehicle_param().wheel_base();
*/
  l_warm_up_(horizon_ + 1, 4);
  n_warm_up_(horizon_ + 1, 2);
}

bool DualVariableWarmStartIPOPTInterface::get_nlp_info(
    int& n, int& m, int& nnz_jac_g, int& nnz_h_lag,
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

bool DualVariableWarmStartIPOPTInterface::get_starting_point(
    int n, bool init_x, double* x, bool init_z, double* z_L, double* z_U, int m,
    bool init_lambda, double* lambda) {
  return true;
}

bool DualVariableWarmStartIPOPTInterface::get_bounds_info(int n, double* x_l,
                                                          double* x_u, int m,
                                                          double* g_l,
                                                          double* g_u) {
  // here, the n and m we gave IPOPT in get_nlp_info are passed back to us.
  // If desired, we could assert to make sure they are what we think they are.
  CHECK(n == num_of_variables_) << "num_of_variables_ mismatch, n: " << n
                                << ", num_of_variables_: " << num_of_variables_;
  CHECK(m == num_of_constraints_)
      << "num_of_constraints_ mismatch, n: " << n
      << ", num_of_constraints_: " << num_of_constraints_;

  return true;
}

bool DualVariableWarmStartIPOPTInterface::eval_f(int n, const double* x,
                                                 bool new_x,
                                                 double& obj_value) {
  return true;
}

bool DualVariableWarmStartIPOPTInterface::eval_grad_f(int n, const double* x,
                                                      bool new_x,
                                                      double* grad_f) {
  return true;
}

bool DualVariableWarmStartIPOPTInterface::eval_h(int n, const double* x,
                                                 bool new_x, double obj_factor,
                                                 int m, const double* lambda,
                                                 bool new_lambda, int nele_hess,
                                                 int* iRow, int* jCol,
                                                 double* values) {
  return true;
}

bool DualVariableWarmStartIPOPTInterface::eval_g(int n, const double* x,
                                                 bool new_x, int m, double* g) {
  return true;
}

bool DualVariableWarmStartIPOPTInterface::eval_jac_g(int n, const double* x,
                                                     bool new_x, int m,
                                                     int nele_jac, int* iRow,
                                                     int* jCol,
                                                     double* values) {
  return true;
}

void DualVariableWarmStartIPOPTInterface::finalize_solution(
    Ipopt::SolverReturn status, int n, const double* x, const double* z_L,
    const double* z_U, int m, const double* g, const double* lambda,
    double obj_value, const Ipopt::IpoptData* ip_data,
    Ipopt::IpoptCalculatedQuantities* ip_cq) {}

void DualVariableWarmStartIPOPTInterface::get_optimization_results(
    Eigen::MatrixXd* l_warm_up, Eigen::MatrixXd* n_warm_up) const {
  *l_warm_up = l_warm_up_;
  *n_warm_up = n_warm_up_;
}
}  // namespace planning
}  // namespace apollo
