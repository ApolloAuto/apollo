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

#include "cyber/common/file.h"
#include "gtest/gtest.h"

namespace apollo {
namespace planning {

class DistanceApproachIPOPTInterfaceTest : public ::testing::Test {
 public:
  virtual void SetUp() {
    FLAGS_planner_open_space_config_filename =
        "/apollo/modules/planning/testdata/conf/"
        "open_space_standard_parking_lot.pb.txt";
    CHECK(apollo::cyber::common::GetProtoFromFile(
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
  size_t horizon_ = 43;
  size_t obstacles_num_ = 4;
  double ts_ = 0.5;
  Eigen::MatrixXd ego_ = Eigen::MatrixXd::Ones(4, 1);
  Eigen::MatrixXd x0_ = Eigen::MatrixXd::Ones(4, 1);
  Eigen::MatrixXd xf_ = 10 * Eigen::MatrixXd::Ones(4, 1);
  Eigen::MatrixXd last_time_u_ = Eigen::MatrixXd::Zero(2, 1);
  std::vector<double> XYbounds_ = {1.0, 1.0, 1.0, 1.0};
  Eigen::MatrixXd xWS_ = Eigen::MatrixXd::Ones(4, 44);
  Eigen::MatrixXd uWS_ = Eigen::MatrixXd::Ones(2, 43);
  Eigen::MatrixXi obstacles_edges_num_;  // {2, 1, 2, 1}
  size_t obstacles_edges_sum_;
  Eigen::MatrixXd obstacles_A_ = Eigen::MatrixXd::Ones(6, 2);
  Eigen::MatrixXd obstacles_b_ = Eigen::MatrixXd::Ones(6, 1);
  bool use_fix_time_ = false;
  std::unique_ptr<DistanceApproachIPOPTInterface> ptop_ = nullptr;
  PlannerOpenSpaceConfig planner_open_space_config_;
  DistanceApproachConfig distance_approach_config_;
};

void DistanceApproachIPOPTInterfaceTest::ProblemSetup() {
  // obstacles_edges_num_ = 4 * Eigen::MatrixXi::Ones(obstacles_num_, 1);
  obstacles_edges_num_ = Eigen::MatrixXi(obstacles_num_, 1);
  obstacles_edges_num_ << 2, 1, 2, 1;
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
  int n = 1274;
  int m = 2194;
  double x_l[1274];
  double x_u[1274];
  double g_l[2194];
  double g_u[2194];
  bool res = ptop_->get_bounds_info(n, x_l, x_u, m, g_l, g_u);
  EXPECT_TRUE(res);
}

TEST_F(DistanceApproachIPOPTInterfaceTest, get_starting_point) {
  int n = 1274;
  int m = 2194;
  bool init_x = true;
  bool init_z = false;
  bool init_lambda = false;
  double x[1274];
  double z_L[2194];
  double z_U[2194];
  double lambda[2194];
  bool res = ptop_->get_starting_point(n, init_x, x, init_z, z_L, z_U, m,
                                       init_lambda, lambda);
  EXPECT_TRUE(res);
}

TEST_F(DistanceApproachIPOPTInterfaceTest, eval_f) {
  int n = 1274;
  double obj_value = 0.0;
  double x[1274];
  std::fill_n(x, n, 1.2);
  bool res = ptop_->eval_f(n, x, true, obj_value);
  EXPECT_DOUBLE_EQ(obj_value, 1443.3600000000008) << "eval_f: " << obj_value;
  EXPECT_TRUE(res);
}

}  // namespace planning
}  // namespace apollo
