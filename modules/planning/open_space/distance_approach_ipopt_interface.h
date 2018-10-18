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
 * @file
 */

#pragma once

#include <math.h>
#include <utility>
#include <vector>

#include "Eigen/Dense"
#include "IpTNLP.hpp"
#include "IpTypes.hpp"

#include "cybertron/common/log.h"
#include "cybertron/common/macros.h"
#include "modules/common/configs/proto/vehicle_config.pb.h"
#include "modules/common/configs/vehicle_config_helper.h"
#include "modules/common/math/math_utils.h"
#include "modules/common/util/file.h"
#include "modules/common/util/util.h"
#include "modules/planning/common/planning_gflags.h"
#include "modules/planning/proto/planner_open_space_config.pb.h"

namespace apollo {
namespace planning {

class DistanceApproachIPOPTInterface : public Ipopt::TNLP {
 public:
  explicit DistanceApproachIPOPTInterface(
      const int num_of_variables, const int num_of_constraints,
      std::size_t horizon, float ts, Eigen::MatrixXd ego,
      const Eigen::MatrixXd& xWS, const Eigen::MatrixXd& uWS,
      const Eigen::MatrixXd& timeWS, Eigen::MatrixXd x0, Eigen::MatrixXd xf,
      Eigen::MatrixXd last_time_u, Eigen::MatrixXd XYbounds,
      Eigen::MatrixXd obstacles_edges_num, std::size_t obstacles_num,
      const Eigen::MatrixXd& obstacles_A, const Eigen::MatrixXd& obstacles_b,
      bool use_fix_time);

  virtual ~DistanceApproachIPOPTInterface() = default;

  void set_objective_weights(
      const apollo::planning::DistanceApproachConfig& distance_approach_config);

  void get_optimization_results(Eigen::MatrixXd* state_result,
                                Eigen::MatrixXd* control_result,
                                Eigen::MatrixXd* time_result) const;

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

 private:
  int num_of_variables_;
  int num_of_constraints_;
  std::size_t horizon_;
  double ts_;
  Eigen::MatrixXd ego_;
  Eigen::MatrixXd xWS_;
  Eigen::MatrixXd uWS_;
  Eigen::MatrixXd timeWS_;
  Eigen::MatrixXd x0_;
  Eigen::MatrixXd xf_;
  Eigen::MatrixXd last_time_u_;
  Eigen::MatrixXd XYbounds_;

  // penalty
  double weight_state_x_;
  double weight_state_y_;
  double weight_state_phi_;
  double weight_input_steer_;
  double weight_input_a_;
  double weight_rate_steer_;
  double weight_rate_a_;
  double weight_stitching_steer_;
  double weight_stitching_a_;
  double weight_first_order_time_;
  double weight_second_order_time_;

  double w_ev_;
  double l_ev_;
  std::vector<double> g_;
  double offset_;
  Eigen::MatrixXd obstacles_edges_num_;
  std::size_t obstacles_num_;
  std::size_t obstacles_edges_sum_;
  double wheelbase_;

  Eigen::MatrixXd state_result_;
  Eigen::MatrixXd control_result_;
  Eigen::MatrixXd time_result_;

  // obstacles_A
  Eigen::MatrixXd obstacles_A_;

  // obstacles_b
  Eigen::MatrixXd obstacles_b_;

  // whether to use fix time
  bool use_fix_time_ = false;

  // state start index
  std::size_t state_start_index_ = 0;

  // control start index.
  std::size_t control_start_index_ = 0;

  // time start index
  std::size_t time_start_index_ = 0;

  // lagrangian l start index
  std::size_t l_start_index_ = 0;

  // lagrangian n start index
  std::size_t n_start_index_ = 0;

  double min_safety_distance_ = 0.0;

  double max_steer_angle_ = 0.0;

  double max_speed_forward_ = 0.0;

  double max_speed_reverse_ = 0.0;

  double max_acceleration_forward_ = 0.0;

  double max_acceleration_reverse_ = 0.0;

  double min_time_sample_scaling_ = 0.0;

  double max_time_sample_scaling_ = 0.0;

  double max_steer_rate_ = 0.0;

 private:
  DistanceApproachConfig distance_approach_config_;
  PlannerOpenSpaceConfig planner_open_space_config_;
  const common::VehicleParam vehicle_param_ =
      common::VehicleConfigHelper::GetConfig().vehicle_param();
};

}  // namespace planning
}  // namespace apollo
