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
#include "modules/planning/open_space/trajectory_smoother/distance_approach_ipopt_interface.h"

#include "IpTNLP.hpp"
#include "IpTypes.hpp"

#include "modules/common/util/file.h"
#include "modules/planning/common/planning_gflags.h"

#include "gtest/gtest.h"

namespace apollo {
namespace planning {

class DistanceApproachIPOPTInterfaceTest : public ::testing::Test {
 public:
  virtual void SetUp() {
    FLAGS_planner_open_space_config_filename =
        "/apollo/modules/planning/testdata/conf/"
        "open_space_standard_parking_lot.pb.txt";
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
  size_t horizon_ = 5;
  size_t obstacles_num_ = 10;
  double ts_ = 0.01;
  Eigen::MatrixXd ego_ = Eigen::MatrixXd::Ones(4, 1);
  Eigen::MatrixXd x0_ = Eigen::MatrixXd::Ones(4, 1);
  Eigen::MatrixXd xf_ = 10 * Eigen::MatrixXd::Ones(4, 1);
  Eigen::MatrixXd last_time_u_ = Eigen::MatrixXd::Zero(2, 1);
  std::vector<double> XYbounds_ = {1.0, 1.0, 1.0, 1.0};
  Eigen::MatrixXd xWS_ = Eigen::MatrixXd::Ones(4, 6);
  Eigen::MatrixXd uWS_ = Eigen::MatrixXd::Ones(2, 5);
  Eigen::MatrixXi obstacles_edges_num_;
  size_t obstacles_edges_sum_;
  Eigen::MatrixXd obstacles_A_ = Eigen::MatrixXd::Ones(10, 2);
  Eigen::MatrixXd obstacles_b_ = Eigen::MatrixXd::Ones(10, 1);
  bool use_fix_time_ = false;
  std::unique_ptr<DistanceApproachIPOPTInterface> ptop_ = nullptr;
  apollo::planning::PlannerOpenSpaceConfig planner_open_space_config_;
  apollo::planning::DistanceApproachConfig distance_approach_config_;
};

void DistanceApproachIPOPTInterfaceTest::ProblemSetup() {
  obstacles_edges_num_ = 4 * Eigen::MatrixXi::Ones(obstacles_num_, 1);
  obstacles_edges_sum_ = obstacles_edges_num_.sum();
  Eigen::MatrixXd l_warm_up_ =
      Eigen::MatrixXd::Ones(obstacles_edges_sum_, horizon_ + 1);
  Eigen::MatrixXd n_warm_up_ =
      Eigen::MatrixXd::Ones(4 * obstacles_num_, horizon_ + 1);
  ptop_.reset(new DistanceApproachIPOPTInterface(
      horizon_, ts_, ego_, xWS_, uWS_, l_warm_up_, n_warm_up_, x0_, xf_,
      last_time_u_, XYbounds_, obstacles_edges_num_, obstacles_num_,
      obstacles_A_, obstacles_b_, planner_open_space_config_));
}

TEST_F(DistanceApproachIPOPTInterfaceTest, initilization) {
  EXPECT_NE(ptop_, nullptr);
}

TEST_F(DistanceApproachIPOPTInterfaceTest, get_bounds_info) {
  int n = 520;
  int m = 270;
  double x_l[520];
  double x_u[520];
  double g_l[270];
  double g_u[270];
  bool res = ptop_->get_bounds_info(n, x_l, x_u, m, g_l, g_u);
  EXPECT_TRUE(res);
}

TEST_F(DistanceApproachIPOPTInterfaceTest, get_starting_point) {
  int n = 520;
  int m = 270;
  bool init_x = true;
  bool init_z = false;
  bool init_lambda = false;
  double x[520];
  double z_L[520];
  double z_U[520];
  double lambda[520];
  bool res = ptop_->get_starting_point(n, init_x, x, init_z, z_L, z_U, m,
                                       init_lambda, lambda);
  EXPECT_TRUE(res);
}

TEST_F(DistanceApproachIPOPTInterfaceTest, eval_f) {
  int n = 520;
  double obj_value = 0.0;
  double x[520];
  std::fill_n(x, n, 1.2);
  bool res = ptop_->eval_f(n, x, true, obj_value);
  EXPECT_DOUBLE_EQ(obj_value, 596.79999999999995);
  EXPECT_TRUE(res);
}

}  // namespace planning
}  // namespace apollo
