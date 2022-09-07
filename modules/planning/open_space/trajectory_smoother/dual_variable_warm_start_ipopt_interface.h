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

#pragma once

#include <limits>
#include <vector>

#include <adolc/adolc.h>
#include <adolc/adolc_sparse.h>

#include <coin/IpTNLP.hpp>
#include <coin/IpTypes.hpp>

#include "Eigen/Dense"

#include "modules/common_msgs/config_msgs/vehicle_config.pb.h"
#include "modules/common/configs/vehicle_config_helper.h"
#include "modules/planning/proto/planner_open_space_config.pb.h"

// #define tag_f 1
// #define tag_g 2
#define tag_L 3
// #define HPOFF 30

namespace apollo {
namespace planning {

class DualVariableWarmStartIPOPTInterface : public Ipopt::TNLP {
 public:
  DualVariableWarmStartIPOPTInterface(
      size_t horizon, double ts, const Eigen::MatrixXd& ego,
      const Eigen::MatrixXi& obstacles_edges_num, const size_t obstacles_num,
      const Eigen::MatrixXd& obstacles_A, const Eigen::MatrixXd& obstacles_b,
      const Eigen::MatrixXd& xWS,
      const PlannerOpenSpaceConfig& planner_open_space_config);

  virtual ~DualVariableWarmStartIPOPTInterface() = default;

  void get_optimization_results(Eigen::MatrixXd* l_warm_up,
                                Eigen::MatrixXd* n_warm_up) const;

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
   *   1) The structure of the jacobian (if "values" is nullptr)
   *   2) The values of the jacobian (if "values" is not nullptr)
   */
  bool eval_jac_g(int n, const double* x, bool new_x, int m, int nele_jac,
                  int* iRow, int* jCol, double* values) override;

  /** Method to return:
   *   1) The structure of the hessian of the lagrangian (if "values" is
   * nullptr) 2) The values of the hessian of the lagrangian (if "values" is not
   * nullptr)
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

  //***************    start ADOL-C part ***********************************
  /** Template to return the objective value */
  template <class T>
  bool eval_obj(int n, const T* x, T* obj_value);

  /** Template to compute constraints */
  template <class T>
  bool eval_constraints(int n, const T* x, int m, T* g);

  /** Method to generate the required tapes */
  void generate_tapes(int n, int m, int* nnz_h_lag);
  //***************    end   ADOL-C part ***********************************

 private:
  int num_of_variables_;
  int num_of_constraints_;
  int horizon_;
  double ts_;
  Eigen::MatrixXd ego_;
  int lambda_horizon_ = 0;
  int miu_horizon_ = 0;
  int dual_formulation_horizon_ = 0;

  Eigen::MatrixXd l_warm_up_;
  Eigen::MatrixXd n_warm_up_;
  double wheelbase_;

  double w_ev_;
  double l_ev_;
  std::vector<double> g_;
  double offset_;
  Eigen::MatrixXi obstacles_edges_num_;
  int obstacles_num_;
  int obstacles_edges_sum_;

  // lagrangian l start index
  int l_start_index_ = 0;

  // lagrangian n start index
  int n_start_index_ = 0;

  // lagrangian d start index
  int d_start_index_ = 0;

  // obstacles_A
  Eigen::MatrixXd obstacles_A_;

  // obstacles_b
  Eigen::MatrixXd obstacles_b_;

  // states of warm up stage
  Eigen::MatrixXd xWS_;

  double weight_d_;

  //***************    start ADOL-C part ***********************************
  double* obj_lam;
  unsigned int* rind_L; /* row indices    */
  unsigned int* cind_L; /* column indices */
  double* hessval;      /* values */
  int nnz_L;
  int options_L[4];
  //***************    end   ADOL-C part ***********************************
};

}  // namespace planning
}  // namespace apollo
