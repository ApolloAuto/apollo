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
#include "modules/planning/open_space/trajectory_smoother/dual_variable_warm_start_ipopt_interface.h"

#include "cyber/common/file.h"
#include "gtest/gtest.h"

#include "modules/planning/common/planning_gflags.h"

namespace apollo {
namespace planning {

class DualVariableWarmStartIPOPTInterfaceTest : public ::testing::Test {
 public:
  virtual void SetUp() {
    FLAGS_planner_open_space_config_filename =
        "/apollo/modules/planning/testdata/conf/"
        "open_space_standard_parking_lot.pb.txt";

    CHECK(apollo::cyber::common::GetProtoFromFile(
        FLAGS_planner_open_space_config_filename, &planner_open_space_config_))
        << "Failed to load open space config file "
        << FLAGS_planner_open_space_config_filename;

    ProblemSetup();
  }

 protected:
  void ProblemSetup();

 protected:
  size_t horizon_ = 5;
  size_t obstacles_num_ = 10;
  double ts_ = 0.01;
  Eigen::MatrixXd ego_ = Eigen::MatrixXd::Ones(4, 1);
  Eigen::MatrixXd last_time_u_ = Eigen::MatrixXd::Zero(2, 1);
  Eigen::MatrixXi obstacles_edges_num_;
  Eigen::MatrixXd obstacles_A_ = Eigen::MatrixXd::Ones(10, 2);
  Eigen::MatrixXd obstacles_b_ = Eigen::MatrixXd::Ones(10, 1);
  int num_of_variables_ = 0;
  double rx_ = 0.0;
  double ry_ = 0.0;
  double r_yaw_ = 0.0;

  int num_of_constraints_ = 0;

  std::unique_ptr<DualVariableWarmStartIPOPTInterface> ptop_ = nullptr;
  PlannerOpenSpaceConfig planner_open_space_config_;
};

void DualVariableWarmStartIPOPTInterfaceTest::ProblemSetup() {
  obstacles_edges_num_ = 4 * Eigen::MatrixXi::Ones(obstacles_num_, 1);
  Eigen::MatrixXd xWS = Eigen::MatrixXd::Ones(4, horizon_ + 1);
  ptop_.reset(new DualVariableWarmStartIPOPTInterface(
      horizon_, ts_, ego_, obstacles_edges_num_, obstacles_num_, obstacles_A_,
      obstacles_b_, xWS, planner_open_space_config_));
}

TEST_F(DualVariableWarmStartIPOPTInterfaceTest, initilization) {
  EXPECT_NE(ptop_, nullptr);
}

TEST_F(DualVariableWarmStartIPOPTInterfaceTest, get_bounds_info) {
  int kNumOfVariables = 540;
  int kNumOfConstraints = 240;
  double x_l[kNumOfVariables];
  double x_u[kNumOfVariables];
  double g_l[kNumOfConstraints];
  double g_u[kNumOfConstraints];
  bool res = ptop_->get_bounds_info(kNumOfVariables, x_l, x_u,
                                    kNumOfConstraints, g_l, g_u);
  EXPECT_TRUE(res);
}

TEST_F(DualVariableWarmStartIPOPTInterfaceTest, get_starting_point) {
  int kNumOfVariables = 540;
  int kNumOfConstraints = 240;
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

TEST_F(DualVariableWarmStartIPOPTInterfaceTest, eval_f) {
  int kNumOfVariables = 540;
  double obj_value;
  double x[kNumOfVariables];
  std::fill_n(x, kNumOfVariables, 1.2);
  bool res = ptop_->eval_f(kNumOfVariables, x, true, obj_value);
  EXPECT_DOUBLE_EQ(obj_value, 72.000000000000085);
  EXPECT_TRUE(res);
}

}  // namespace planning
}  // namespace apollo
