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
#include "modules/planning/open_space/trajectory_smoother/distance_approach_problem.h"

#include "cyber/common/file.h"
#include "gtest/gtest.h"
#include "modules/planning/common/planning_gflags.h"

namespace apollo {
namespace planning {

class DistanceApproachProblemTest : public ::testing::Test {
 public:
  virtual void SetUp() {
    FLAGS_planner_open_space_config_filename =
        "/apollo/modules/planning/testdata/conf/"
        "open_space_standard_parking_lot.pb.txt";

    CHECK(apollo::cyber::common::GetProtoFromFile(
        FLAGS_planner_open_space_config_filename, &planner_open_space_config_))
        << "Failed to load open space config file "
        << FLAGS_planner_open_space_config_filename;
  }

 protected:
  std::unique_ptr<DistanceApproachProblem> distance_approach_ = nullptr;
  int num_of_variables_ = 160;
  int num_of_constraints_ = 200;
  size_t horizon_ = 20;
  double ts_ = 0.01;
  Eigen::MatrixXd ego_ = Eigen::MatrixXd::Ones(4, 1);
  Eigen::MatrixXd x0_ = Eigen::MatrixXd::Ones(4, 1);
  Eigen::MatrixXd xf_ = Eigen::MatrixXd::Ones(4, 1);
  std::vector<double> XYbounds_ = {1.0, 1.0, 1.0, 1.0};
  Eigen::MatrixXd last_time_u_ = Eigen::MatrixXd::Zero(2, 1);
  Eigen::MatrixXi obstacles_edges_num_ = Eigen::MatrixXi::Ones(12, 4);
  Eigen::MatrixXd xWS_ = Eigen::MatrixXd::Zero(4, horizon_ + 1);
  Eigen::MatrixXd uWS_ = Eigen::MatrixXd::Zero(2, horizon_);
  Eigen::MatrixXi obstacles_edges_num = Eigen::MatrixXi::Zero(1, horizon_ + 1);
  Eigen::MatrixXd obstacles_A = Eigen::MatrixXd::Ones(4, 1);
  Eigen::MatrixXd obstacles_b = Eigen::MatrixXd::Ones(4, 1);
  PlannerOpenSpaceConfig planner_open_space_config_;
};

TEST_F(DistanceApproachProblemTest, initilization) {
  distance_approach_.reset(
      new DistanceApproachProblem(planner_open_space_config_));
  EXPECT_NE(distance_approach_, nullptr);
}

}  // namespace planning
}  // namespace apollo
