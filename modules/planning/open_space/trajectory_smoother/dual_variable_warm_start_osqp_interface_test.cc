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
#include "modules/planning/open_space/trajectory_smoother/dual_variable_warm_start_osqp_interface.h"

#include "cyber/common/file.h"
#include "gtest/gtest.h"

#include "modules/planning/common/planning_gflags.h"

namespace apollo {
namespace planning {

class DualVariableWarmStartOSQPInterfaceTest : public ::testing::Test {
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
  size_t obstacles_num_ = 4;
  double ts_ = 0.1;
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

  std::unique_ptr<DualVariableWarmStartOSQPInterface> ptop_ = nullptr;
  PlannerOpenSpaceConfig planner_open_space_config_;
};

void DualVariableWarmStartOSQPInterfaceTest::ProblemSetup() {
  obstacles_edges_num_ = Eigen::MatrixXi(obstacles_num_, 1);
  obstacles_edges_num_ << 2, 1, 2, 1;
  Eigen::MatrixXd xWS = Eigen::MatrixXd::Ones(4, horizon_ + 1);
  ptop_.reset(new DualVariableWarmStartOSQPInterface(
      horizon_, ts_, ego_, obstacles_edges_num_, obstacles_num_, obstacles_A_,
      obstacles_b_, xWS, planner_open_space_config_));
}

TEST_F(DualVariableWarmStartOSQPInterfaceTest, initilization) {
  EXPECT_NE(ptop_, nullptr);
}

TEST_F(DualVariableWarmStartOSQPInterfaceTest, optimize) {
  int obstacles_edges_sum = obstacles_edges_num_.sum();
  Eigen::MatrixXd l_warm_up(obstacles_edges_sum, horizon_ + 1);
  Eigen::MatrixXd n_warm_up(4 * obstacles_num_, horizon_ + 1);

  bool res = ptop_->optimize();
  EXPECT_TRUE(res);

  ptop_->get_optimization_results(&l_warm_up, &n_warm_up);
  for (int r = 0; r < obstacles_edges_sum; ++r) {
    for (int c = 0; c < static_cast<int>(horizon_) + 1; ++c) {
      EXPECT_EQ(l_warm_up(r, c), 0.0);
    }
  }
  for (int r = 0; r < 4 * static_cast<int>(obstacles_num_); ++r) {
    for (int c = 0; c < static_cast<int>(horizon_) + 1; ++c) {
      EXPECT_EQ(n_warm_up(r, c), 0.0);
    }
  }
}
}  // namespace planning
}  // namespace apollo
