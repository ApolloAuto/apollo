/******************************************************************************
 * Copyright 2019 The Apollo Authors. All Rights Reserved.
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

#pragma once

#include <adolc/adolc.h>
#include <adolc/adolc_openmp.h>
#include <adolc/adolc_sparse.h>
#include <adolc/adouble.h>
#include <omp.h>
#include <coin/IpTNLP.hpp>
#include <coin/IpTypes.hpp>

#include "Eigen/Dense"

#include "cyber/common/log.h"
#include "cyber/common/macros.h"
#include "modules/common/configs/proto/vehicle_config.pb.h"
#include "modules/common/configs/vehicle_config_helper.h"
#include "modules/common/math/math_utils.h"
#include "modules/common/util/util.h"
#include "modules/planning/common/planning_gflags.h"
#include "modules/planning/proto/planner_open_space_config.pb.h"

#define tag_f 1
#define tag_g 2
#define tag_L 3
#define HPOFF 30

namespace apollo {
namespace planning {

class DistanceApproachInterface : public Ipopt::TNLP {
 public:
  virtual ~DistanceApproachInterface() = default;

  /** Method to return some info about the nlp */
  virtual bool get_nlp_info(int& n, int& m, int& nnz_jac_g,    // NOLINT
                            int& nnz_h_lag,                    // NOLINT
                            IndexStyleEnum& index_style) = 0;  // NOLINT

  /** Method to return the bounds for my problem */
  virtual bool get_bounds_info(int n, double* x_l, double* x_u, int m,
                               double* g_l, double* g_u) = 0;

  /** Method to return the starting point for the algorithm */
  virtual bool get_starting_point(int n, bool init_x, double* x, bool init_z,
                                  double* z_L, double* z_U, int m,
                                  bool init_lambda, double* lambda) = 0;

  /** Method to return the objective value */
  virtual bool eval_f(int n, const double* x, bool new_x,  // NOLINT
                      double& obj_value) = 0;              // NOLINT

  /** Method to return the gradient of the objective */
  virtual bool eval_grad_f(int n, const double* x, bool new_x,
                           double* grad_f) = 0;

  /** Method to return the constraint residuals */
  virtual bool eval_g(int n, const double* x, bool new_x, int m, double* g) = 0;

  /** Check unfeasible constraints for further study**/
  virtual bool check_g(int n, const double* x, int m, const double* g) = 0;

  /** Method to return:
   *   1) The structure of the jacobian (if "values" is nullptr)
   *   2) The values of the jacobian (if "values" is not nullptr)
   */
  virtual bool eval_jac_g(int n, const double* x, bool new_x, int m,
                          int nele_jac, int* iRow, int* jCol,
                          double* values) = 0;
  // sequential implementation to jac_g
  virtual bool eval_jac_g_ser(int n, const double* x, bool new_x, int m,
                              int nele_jac, int* iRow, int* jCol,
                              double* values) = 0;

  /** Method to return:
   *   1) The structure of the hessian of the lagrangian (if "values" is
   * nullptr) 2) The values of the hessian of the lagrangian (if "values" is not
   * nullptr)
   */
  virtual bool eval_h(int n, const double* x, bool new_x, double obj_factor,
                      int m, const double* lambda, bool new_lambda,
                      int nele_hess, int* iRow, int* jCol, double* values) = 0;

  /** @name Solution Methods */
  /** This method is called when the algorithm is complete so the TNLP can
   * store/write the solution */
  virtual void finalize_solution(Ipopt::SolverReturn status, int n,
                                 const double* x, const double* z_L,
                                 const double* z_U, int m, const double* g,
                                 const double* lambda, double obj_value,
                                 const Ipopt::IpoptData* ip_data,
                                 Ipopt::IpoptCalculatedQuantities* ip_cq) = 0;

  virtual void get_optimization_results(
      Eigen::MatrixXd* state_result, Eigen::MatrixXd* control_result,
      Eigen::MatrixXd* time_result, Eigen::MatrixXd* dual_l_result,
      Eigen::MatrixXd* dual_n_result) const = 0;
};

}  // namespace planning
}  // namespace apollo
