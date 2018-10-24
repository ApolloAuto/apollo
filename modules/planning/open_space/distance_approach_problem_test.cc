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
#include "modules/planning/open_space/distance_approach_problem.h"

#include "gtest/gtest.h"

namespace apollo {
namespace planning {

class DistanceApproachProblemTest : public ::testing::Test {
 public:
  virtual void SetUp() {
    CHECK(apollo::common::util::GetProtoFromFile(
        FLAGS_planner_open_space_config_filename, &planner_open_space_config_))
        << "Failed to load open space config file "
        << FLAGS_planner_open_space_config_filename;
  }

 protected:
  std::unique_ptr<DistanceApproachProblem> distance_approach_ = nullptr;
  int num_of_variables_ = 160;
  int num_of_constraints_ = 200;
  std::size_t horizon_ = 20;
  float ts_ = 0.01;
  Eigen::MatrixXd ego_ = Eigen::MatrixXd::Ones(4, 1);
  Eigen::MatrixXd x0_ = Eigen::MatrixXd::Ones(4, 1);
  Eigen::MatrixXd xf_ = Eigen::MatrixXd::Ones(4, 1);
  Eigen::MatrixXd XYbounds_ = Eigen::MatrixXd::Ones(4, 1);
  Eigen::MatrixXd last_time_u_ = Eigen::MatrixXd::Zero(2, 1);
  Eigen::MatrixXd obstacles_edges_num_ = Eigen::MatrixXd::Ones(12, 4);
  Eigen::MatrixXd xWS_ = Eigen::MatrixXd::Zero(4, horizon_ + 1);
  Eigen::MatrixXd uWS_ = Eigen::MatrixXd::Zero(2, horizon_);
  std::size_t obstacles_num = 10;
  Eigen::MatrixXd obstacles_edges_num = Eigen::MatrixXd::Zero(1, horizon_ + 1);
  Eigen::MatrixXd obstacles_A = Eigen::MatrixXd::Ones(4, 1);
  Eigen::MatrixXd obstacles_b = Eigen::MatrixXd::Ones(4, 1);
  apollo::planning::PlannerOpenSpaceConfig planner_open_space_config_;
};

TEST_F(DistanceApproachProblemTest, initilization) {
  distance_approach_.reset(new DistanceApproachProblem(
      x0_, xf_, last_time_u_, horizon_, ts_, ego_, xWS_, uWS_, XYbounds_,
      obstacles_num, obstacles_edges_num, obstacles_A, obstacles_b,
      planner_open_space_config_));
  EXPECT_NE(distance_approach_, nullptr);
}

}  // namespace planning
}  // namespace apollo
