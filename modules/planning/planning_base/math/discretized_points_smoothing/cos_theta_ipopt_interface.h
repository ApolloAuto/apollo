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

#pragma once

#include <cstddef>
#include <map>
#include <utility>
#include <vector>

#include <adolc/adolc.h>
#include <adolc/adolc_sparse.h>
#include <adolc/adouble.h>
#include <coin/IpIpoptApplication.hpp>
#include <coin/IpIpoptCalculatedQuantities.hpp>
#include <coin/IpIpoptData.hpp>
#include <coin/IpOrigIpoptNLP.hpp>
#include <coin/IpSolveStatistics.hpp>
#include <coin/IpTNLP.hpp>
#include <coin/IpTNLPAdapter.hpp>
#include <coin/IpTypes.hpp>

#define tag_f 1
#define tag_g 2
#define tag_L 3
#define HPOFF 30

namespace apollo {
namespace planning {

class CosThetaIpoptInterface : public Ipopt::TNLP {
 public:
  CosThetaIpoptInterface(std::vector<std::pair<double, double>> points,
                         std::vector<double> bounds);

  virtual ~CosThetaIpoptInterface() = default;

  void set_weight_cos_included_angle(const double weight_cos_included_angle);

  void set_weight_anchor_points(const double weight_anchor_points);

  void set_weight_length(const double weight_length);

  void set_automatic_differentiation_flag(const bool use_ad);

  void get_optimization_results(std::vector<double>* ptr_x,
                                std::vector<double>* ptr_y) const;

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

  /** Template to compute contraints */
  template <class T>
  bool eval_constraints(int n, const T* x, int m, T* g);

  /** Method to generate the required tapes */
  virtual void generate_tapes(int n, int m, int* nnz_jac_g, int* nnz_h_lag);

  //***************    end   ADOL-C part ***********************************

 private:
  std::vector<std::pair<double, double>> ref_points_;

  std::vector<double> bounds_;

  std::vector<double> opt_x_;

  std::vector<double> opt_y_;

  size_t num_of_variables_ = 0;

  size_t num_of_constraints_ = 0;

  size_t nnz_jac_g_ = 0;

  size_t nnz_h_lag_ = 0;

  size_t num_of_points_ = 0;

  std::map<std::pair<size_t, size_t>, size_t> idx_map_;

  void hessian_strcuture();

  double weight_cos_included_angle_ = 0.0;

  double weight_anchor_points_ = 0.0;

  double weight_length_ = 0.0;

  //***************    start ADOL-C part ***********************************

  bool use_automatic_differentiation_ = false;
  /**@name Methods to block default compiler methods.
   */
  CosThetaIpoptInterface(const CosThetaIpoptInterface&);
  CosThetaIpoptInterface& operator=(const CosThetaIpoptInterface&);

  std::vector<double> obj_lam_;

  // TODO(Jinyun): Not changed to std::vector yet, need further debug
  //** variables for sparsity exploitation
  // std::vector<unsigned int> rind_g_; /* row indices    */
  // std::vector<unsigned int> cind_g_; /* column indices */
  // std::vector<double> jacval_;       /* values         */
  // std::vector<unsigned int> rind_L_; /* row indices    */
  // std::vector<unsigned int> cind_L_; /* column indices */
  // std::vector<double> hessval_;      /* values */

  //** variables for sparsity exploitation
  unsigned int* rind_g_; /* row indices    */
  unsigned int* cind_g_; /* column indices */
  double* jacval_;       /* values         */
  unsigned int* rind_L_; /* row indices    */
  unsigned int* cind_L_; /* column indices */
  double* hessval_;      /* values */

  int nnz_jac_ = 0;
  int nnz_L_ = 0;
  int options_g_[4];
  int options_L_[4];

  //***************    end   ADOL-C part ***********************************
};
}  // namespace planning
}  // namespace apollo
