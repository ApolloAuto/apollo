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

#ifndef MODULES_PLANNING_LATTICE_LATERAL_TRAJECTORY_OPTIMIZER_INTERFACE_H_
#define MODULES_PLANNING_LATTICE_LATERAL_TRAJECTORY_OPTIMIZER_INTERFACE_H_

#include <utility>
#include <vector>

#include "IpTNLP.hpp"
#include "IpTypes.hpp"

namespace apollo {
namespace planning {

class LateralTrajectoryOptimizerInterface : public Ipopt::TNLP {
 public:
  LateralTrajectoryOptimizerInterface(const double d_init,
      const double d_prime_init, const double d_pprime_init,
      const double delta_s, const double d_ppprime_max,
      std::vector<std::pair<double, double>> d_bounds);

  virtual ~LateralTrajectoryOptimizerInterface() = default;

  void set_objective_weights(const double w_d, const double w_d_prime,
      const double w_d_pprime, const double w_d_obs);

  /** Method to return some info about the nlp */
  bool get_nlp_info(int& n, int& m, int& nnz_jac_g, int& nnz_h_lag,
                    IndexStyleEnum& index_style) override;

  /** Method to return the bounds for my problem */
  bool get_bounds_info(int n, double* x_l, double* x_u, int m, double* g_l,
                       double* g_u) override;

  /** Method to return the starting point for the algorithm */
  bool get_starting_point(int n, bool init_x, double* x, bool init_z,
                          double* z_L, double* z_U, int m, bool init_lambda,
                          double* lambda) override;

  /** Method to return the objective value */
  bool eval_f(int n, const double* x, bool new_x, double& obj_value) override;

  /** Method to return the gradient of the objective */
  bool eval_grad_f(int n, const double* x, bool new_x, double* grad_f) override;

  /** Method to return the constraint residuals */
  bool eval_g(int n, const double* x, bool new_x, int m, double* g) override;

  /** Method to return:
   *   1) The structure of the jacobian (if "values" is NULL)
   *   2) The values of the jacobian (if "values" is not NULL)
   */
  bool eval_jac_g(int n, const double* x, bool new_x, int m, int nele_jac,
                  int* iRow, int* jCol, double* values) override;

  /** Method to return:
   *   1) The structure of the hessian of the lagrangian (if "values" is NULL)
   *   2) The values of the hessian of the lagrangian (if "values" is not NULL)
   */
  bool eval_h(int n, const double* x, bool new_x, double obj_factor, int m,
              const double* lambda, bool new_lambda, int nele_hess, int* iRow,
              int* jCol, double* values) override;

  /** @name Solution Methods */
  /** This method is called when the algorithm is complete so the TNLP can
   * store/write the solution */
  void finalize_solution(Ipopt::SolverReturn status, int n, const double* x,
                         const double* z_L, const double* z_U, int m,
                         const double* g, const double* lambda,
                         double obj_value, const Ipopt::IpoptData* ip_data,
                         Ipopt::IpoptCalculatedQuantities* ip_cq) override;

  void GetOptimizationResult(
      std::vector<double>* ptr_opt_d,
      std::vector<double>* ptr_opt_d_prime,
      std::vector<double>* ptr_opt_d_pprime) const;

 private:
  std::size_t num_of_points_;

  std::size_t num_of_variables_;

  std::size_t num_of_constraints_;

  double d_init_ = 0.0;

  double d_prime_init_ = 0.0;

  double d_pprime_init_ = 0.0;

  double delta_s_ = 0.0;

  double d_ppprime_max_ = 0.0;

  std::vector<std::pair<double, double>> d_bounds_;

  double w_d_ = 1.0;

  double w_d_prime_ = 1.0;

  double w_d_pprime_ = 1.0;

  double w_d_obs_ = 1.0;

  std::vector<double> opt_d_;

  std::vector<double> opt_d_prime_;

  std::vector<double> opt_d_pprime_;
};

}  // namespace planning
}  // namespace apollo

#endif  // MODULES_PLANNING_LATTICE_LATERAL_TRAJECTORY_OPTIMIZER_INTERFACE_H_
