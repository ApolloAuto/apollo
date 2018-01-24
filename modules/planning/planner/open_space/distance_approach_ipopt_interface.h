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
 * warm_start_ipopt_interface.h
 */

#ifndef MODULES_PLANNING_PLANNER_OPEN_SPACE_DISTANCE_APPROACH_IPOPT_INTERFACE_H_
#define MODULES_PLANNING_PLANNER_OPEN_SPACE_DISTANCE_APPROACH_IPOPT_INTERFACE_H_

#include <vector>

#include "Eigen/Dense"
#include "IpTNLP.hpp"
#include "IpTypes.hpp"

#include "modules/common/configs/vehicle_config_helper.h"

namespace apollo {
namespace planning {

class DistanceApproachIPOPTInterface : public Ipopt::TNLP {
 public:
  explicit DistanceApproachIPOPTInterface(
      const int num_of_variables, const int num_of_constraints,
      std::size_t horizon, float ts, Eigen::MatrixXd ego, Eigen::MatrixXd x0,
      Eigen::MatrixXd xf, Eigen::MatrixXd XYbounds, Eigen::MatrixXd vOb,
      std::size_t nOb);

  virtual ~DistanceApproachIPOPTInterface() = default;

  void get_optimization_results(std::vector<double>* x1_result,
                                std::vector<double>* x2_result,
                                std::vector<double>* x3_result,
                                std::vector<double>* x4_result,
                                std::vector<double>* u1_result,
                                std::vector<double>* u2_result,
                                std::vector<double>* t_result) const;

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

 private:
  int num_of_variables_;
  int num_of_constraints_;
  std::size_t horizon_;
  float ts_;
  Eigen::MatrixXd ego_;
  Eigen::MatrixXd x0_;
  Eigen::MatrixXd xf_;
  Eigen::MatrixXd XYbounds_;

  std::vector<double> x1_result_;
  std::vector<double> x2_result_;
  std::vector<double> x3_result_;
  std::vector<double> x4_result_;
  std::vector<double> u1_result_;
  std::vector<double> u2_result_;
  std::vector<double> t_result_;

  double w_ev_;
  double l_ev_;
  std::vector<double> g_;
  double offset_;
  Eigen::MatrixXd vOb_;
  std::size_t nOb_;
  std::size_t vObsum_;
  double wheelbase_;
};

}  // namespace planning
}  // namespace apollo

#endif
// MODULES_PLANNING_PLANNER_OPEN_SPACE_DISTANCE_APPROACH_IPOPT_INTERFACE_H_
