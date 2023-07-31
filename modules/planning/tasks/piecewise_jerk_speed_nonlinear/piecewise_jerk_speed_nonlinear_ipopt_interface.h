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

/**
 * @file piecewise_jerk_speed_nonlinear_ipopt_interface.h
 **/

#pragma once

#include <unordered_map>
#include <utility>
#include <vector>

#include <coin/IpTNLP.hpp>
#include <coin/IpTypes.hpp>

#include "modules/planning/planning_base/common/path/path_data.h"
#include "modules/planning/planning_base/common/trajectory1d/piecewise_jerk_trajectory1d.h"

namespace apollo {
namespace planning {

class PiecewiseJerkSpeedNonlinearIpoptInterface : public Ipopt::TNLP {
 public:
  PiecewiseJerkSpeedNonlinearIpoptInterface(
      const double s_init, const double s_dot_init, const double s_ddot_init,
      const double delta_t, const int num_of_points, const double s_max_,
      const double s_dot_max, const double s_ddot_min, const double s_ddot_max,
      const double s_dddot_min, const double s_dddot_max);

  virtual ~PiecewiseJerkSpeedNonlinearIpoptInterface() = default;

  void set_warm_start(const std::vector<std::vector<double>> &speed_profile);

  void set_curvature_curve(const PiecewiseJerkTrajectory1d &curvature_curve);

  void get_optimization_results(std::vector<double> *ptr_opt_s,
                                std::vector<double> *ptr_opt_v,
                                std::vector<double> *ptr_opt_a);

  void set_end_state_target(const double s_target, const double v_target,
                            const double a_target);

  void set_reference_speed(const double s_dot_ref);

  void set_reference_spatial_distance(const std::vector<double> &s_ref);

  void set_speed_limit_curve(const PiecewiseJerkTrajectory1d &v_bound_f);

  void set_safety_bounds(
      const std::vector<std::pair<double, double>> &safety_bounds);

  void set_soft_safety_bounds(
      const std::vector<std::pair<double, double>> &soft_safety_bounds);

  void set_w_target_state(const double w_target_s, const double w_target_v,
                          const double w_target_a);

  void set_w_overall_a(const double w_overall_a);

  void set_w_overall_j(const double w_overall_j);

  void set_w_overall_centripetal_acc(const double w_overall_centripetal_acc);

  void set_w_reference_speed(const double w_reference_speed);

  void set_w_reference_spatial_distance(const double w_ref_s);

  void set_w_soft_s_bound(const double w_soft_s_bound);

  /** Method to return some info about the nlp */
  bool get_nlp_info(int &n, int &m, int &nnz_jac_g, int &nnz_h_lag,
                    IndexStyleEnum &index_style) override;

  /** Method to return the bounds for my problem */
  bool get_bounds_info(int n, double *x_l, double *x_u, int m, double *g_l,
                       double *g_u) override;

  /** Method to return the starting point for the algorithm */
  bool get_starting_point(int n, bool init_x, double *x, bool init_z,
                          double *z_L, double *z_U, int m, bool init_lambda,
                          double *lambda) override;

  /** Method to return the objective value */
  bool eval_f(int n, const double *x, bool new_x, double &obj_value) override;

  /** Method to return the gradient of the objective */
  bool eval_grad_f(int n, const double *x, bool new_x, double *grad_f) override;

  /** Method to return the constraint residuals */
  bool eval_g(int n, const double *x, bool new_x, int m, double *g) override;

  /** Method to return:
   *   1) The structure of the jacobian (if "values" is nullptr)
   *   2) The values of the jacobian (if "values" is not nullptr)
   */
  bool eval_jac_g(int n, const double *x, bool new_x, int m, int nele_jac,
                  int *iRow, int *jCol, double *values) override;

  /** Method to return:
   *   1) The structure of the hessian of the lagrangian (if "values" is
   * nullptr) 2) The values of the hessian of the lagrangian (if "values" is not
   * nullptr)
   */
  bool eval_h(int n, const double *x, bool new_x, double obj_factor, int m,
              const double *lambda, bool new_lambda, int nele_hess, int *iRow,
              int *jCol, double *values) override;

  /** @name Solution Methods */
  /** This method is called when the algorithm is complete so the TNLP can
   * store/write the solution */
  void finalize_solution(Ipopt::SolverReturn status, int n, const double *x,
                         const double *z_L, const double *z_U, int m,
                         const double *g, const double *lambda,
                         double obj_value, const Ipopt::IpoptData *ip_data,
                         Ipopt::IpoptCalculatedQuantities *ip_cq) override;

 private:
  int to_hash_key(const int i, const int j) const;

  std::unordered_map<int, int> hessian_mapper_;

  PiecewiseJerkTrajectory1d curvature_curve_;

  bool use_v_bound_ = false;

  bool use_soft_safety_bound_ = false;

  PiecewiseJerkTrajectory1d v_bound_func_;

  const double s_init_;

  const double s_dot_init_;

  const double s_ddot_init_;

  const double delta_t_;

  const int num_of_points_;

  const double s_max_;

  const double s_dot_max_;

  const double s_ddot_min_;

  const double s_ddot_max_;

  const double s_dddot_min_;

  const double s_dddot_max_;

  const int v_offset_;

  const int a_offset_;

  int lower_s_slack_offset_ = 0;

  int upper_s_slack_offset_ = 0;

  int num_of_variables_ = 0;

  int num_of_constraints_ = 0;

  double w_target_s_ = 10000.0;

  double w_target_v_ = 10000.0;

  double w_target_a_ = 10000.0;

  double w_ref_v_ = 1.0;

  double w_ref_s_ = 1.0;

  double w_overall_a_ = 100.0;

  double w_overall_j_ = 10.0;

  double w_overall_centripetal_acc_ = 500.0;

  double w_soft_s_bound_ = 0.0;

  double v_max_ = 0.0;

  double s_target_ = 0.0;

  double v_target_ = 0.0;

  double a_target_ = 0.0;

  double v_ref_ = 0.0;

  std::vector<std::pair<double, double>> safety_bounds_;

  std::vector<std::pair<double, double>> soft_safety_bounds_;

  bool has_end_state_target_ = false;

  std::vector<double> opt_s_;

  std::vector<double> opt_v_;

  std::vector<double> opt_a_;

  std::vector<std::vector<double>> x_warm_start_;

  std::vector<double> s_ref_;
};
}  // namespace planning
}  // namespace apollo
