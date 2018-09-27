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
#include "modules/planning/planner/open_space/distance_approach_ipopt_interface.h"
#include "IpTNLP.hpp"
#include "IpTypes.hpp"

#include "gtest/gtest.h"

namespace apollo {
namespace planning {

class DistanceApproachIPOPTInterfaceTest : public ::testing::Test {
 public:
  virtual void SetUp() { ProblemSetup(); }

 protected:
  void ProblemSetup();

 protected:
  std::size_t horizon_ = 5;
  std::size_t obstacles_num_ = 10;
  float ts_ = 0.01;
  Eigen::MatrixXd ego_ = Eigen::MatrixXd::Ones(4, 1);
  Eigen::MatrixXd x0_ = Eigen::MatrixXd::Ones(4, 1);
  Eigen::MatrixXd xf_ = 10 * Eigen::MatrixXd::Ones(4, 1);
  Eigen::MatrixXd XYbounds_ = Eigen::MatrixXd::Ones(4, 1);
  Eigen::MatrixXd obstacles_vertices_num_;

  int num_of_variables_;

  int num_of_constraints_;

  std::unique_ptr<DistanceApproachIPOPTInterface> ptop_ = nullptr;
};

void DistanceApproachIPOPTInterfaceTest::ProblemSetup() {
  obstacles_vertices_num_ = 4 * Eigen::MatrixXd::Ones(obstacles_num_, 1);

  num_of_variables_ = 4 * (horizon_ + 1) + 2 * horizon_ + (horizon_ + 1) +
                      (horizon_ + 1) * 4 * obstacles_num_ +
                      4 * obstacles_num_ * (horizon_ + 1);

  num_of_constraints_ = 4 * (horizon_ + 1) + 2 * horizon_ + (horizon_ + 1) +
                        (horizon_ + 1) * 4 * obstacles_num_ +
                        4 * obstacles_num_ * (horizon_ + 1);

  ptop_.reset(new DistanceApproachIPOPTInterface(
      num_of_variables_, num_of_constraints_, horizon_, ts_, ego_, x0_, xf_,
      XYbounds_, obstacles_vertices_num_, obstacles_num_));
}

TEST_F(DistanceApproachIPOPTInterfaceTest, initilization) {
  EXPECT_NE(ptop_, nullptr);
}

TEST_F(DistanceApproachIPOPTInterfaceTest, get_nlp_info) {
  int n, m, nnz_jac_g, nnz_h_lag;
  Ipopt::TNLP::IndexStyleEnum index_style;
  ptop_->get_nlp_info(n, m, nnz_jac_g, nnz_h_lag, index_style);
  EXPECT_EQ(n, num_of_variables_);
  EXPECT_EQ(m, num_of_constraints_);
  EXPECT_EQ(nnz_jac_g, 0);
  EXPECT_EQ(nnz_h_lag, 0);
  EXPECT_EQ(index_style, Ipopt::TNLP::C_STYLE);
}

TEST_F(DistanceApproachIPOPTInterfaceTest, get_bounds_info) {
  int kNumOfVariables = 520;
  int kNumOfConstraints = 520;
  double x_l[kNumOfVariables];
  double x_u[kNumOfVariables];
  double g_l[kNumOfConstraints];
  double g_u[kNumOfConstraints];
  bool res = ptop_->get_bounds_info(kNumOfVariables, x_l, x_u,
                                    kNumOfConstraints, g_l, g_u);
  EXPECT_TRUE(res);
}

TEST_F(DistanceApproachIPOPTInterfaceTest, get_starting_point) {
  int kNumOfVariables = 520;
  int kNumOfConstraints = 520;
  bool init_x = true;
  bool init_z = false;
  bool init_lambda = false;
  double x[kNumOfVariables];
  double z_L[kNumOfVariables];
  double z_U[kNumOfVariables];
  double lambda[kNumOfVariables];
  bool res =
      ptop_->get_starting_point(kNumOfVariables, init_x, x, init_z, z_L, z_U,
                                kNumOfConstraints, init_lambda, lambda);
  EXPECT_TRUE(res);
}

}  // namespace planning
}  // namespace apollo
