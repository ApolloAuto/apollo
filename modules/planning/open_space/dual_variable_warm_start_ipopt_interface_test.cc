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
#include "modules/planning/open_space/dual_variable_warm_start_ipopt_interface.h"

#include "gtest/gtest.h"

namespace apollo {
namespace planning {

class DualVariableWarmStartIPOPTInterfaceTest : public ::testing::Test {
 public:
  virtual void SetUp() {
    CHECK(apollo::common::util::GetProtoFromFile(
        FLAGS_planner_open_space_config_filename, &planner_open_space_config_))
        << "Failed to load open space config file "
        << FLAGS_planner_open_space_config_filename;

    distance_approach_config_ =
        planner_open_space_config_.distance_approach_config();

    ProblemSetup();
  }

 protected:
  void ProblemSetup();

 protected:
  std::size_t horizon_ = 5;
  std::size_t obstacles_num_ = 10;
  float ts_ = 0.01;
  Eigen::MatrixXd ego_ = Eigen::MatrixXd::Ones(4, 1);
  Eigen::MatrixXd x0_ = Eigen::MatrixXd::Ones(4, 1);
  Eigen::MatrixXd xf_ = 10 * Eigen::MatrixXd::Ones(4, 1);
  Eigen::MatrixXd last_time_u_ = Eigen::MatrixXd::Zero(2, 1);
  std::vector<double> XYbounds_ = {1.0, 1.0, 1.0, 1.0};
  Eigen::MatrixXd xWS_ = Eigen::MatrixXd::Ones(4, 6);
  Eigen::MatrixXd uWS_ = Eigen::MatrixXd::Ones(2, 5);
  Eigen::MatrixXd obstacles_edges_num_;
  Eigen::MatrixXd obstacles_A_ = Eigen::MatrixXd::Ones(10, 2);
  Eigen::MatrixXd obstacles_b_ = Eigen::MatrixXd::Ones(10, 1);
  int num_of_variables_;
  bool use_fix_time_ = false;

  int num_of_constraints_;

  std::unique_ptr<DistanceApproachIPOPTInterface> ptop_ = nullptr;
  apollo::planning::PlannerOpenSpaceConfig planner_open_space_config_;
  apollo::planning::DistanceApproachConfig distance_approach_config_;
};

void DistanceApproachIPOPTInterfaceTest::ProblemSetup() {
  obstacles_edges_num_ = 4 * Eigen::MatrixXd::Ones(obstacles_num_, 1);

  num_of_variables_ = 4 * (horizon_ + 1) + 2 * horizon_ + (horizon_ + 1) +
                      (horizon_ + 1) * 4 * obstacles_num_ +
                      4 * obstacles_num_ * (horizon_ + 1);

  num_of_constraints_ =
      4 * horizon_ + horizon_ + horizon_ + 4 * obstacles_num_ * (horizon_ + 1);

  ptop_.reset(new DistanceApproachIPOPTInterface(
      num_of_variables_, num_of_constraints_, horizon_, ts_, ego_, xWS_, uWS_,
      x0_, xf_, last_time_u_, XYbounds_, obstacles_edges_num_, obstacles_num_,
      obstacles_A_, obstacles_b_, planner_open_space_config_));
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
  EXPECT_EQ(nnz_jac_g, 1884);
  EXPECT_EQ(nnz_h_lag, 0);
  EXPECT_EQ(index_style, Ipopt::TNLP::C_STYLE);
}

TEST_F(DistanceApproachIPOPTInterfaceTest, get_bounds_info) {
  int kNumOfVariables = 520;
  int kNumOfConstraints = 270;
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
  int kNumOfConstraints = 270;
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

TEST_F(DistanceApproachIPOPTInterfaceTest, eval_f) {
  int kNumOfVariables = 520;
  double obj_value;
  double x[kNumOfVariables];
  std::fill_n(x, kNumOfVariables, 1.2);
  bool res = ptop_->eval_f(kNumOfVariables, x, true, obj_value);
  EXPECT_TRUE(res);
}

}  // namespace planning
}  // namespace apollo
