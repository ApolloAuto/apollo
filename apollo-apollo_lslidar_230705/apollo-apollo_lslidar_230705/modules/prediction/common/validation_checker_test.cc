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

#include "modules/prediction/common/validation_checker.h"

#include "gtest/gtest.h"

#include "modules/prediction/common/prediction_gflags.h"

namespace apollo {
namespace prediction {

using ::apollo::common::TrajectoryPoint;

class ValidationCheckerTest : public ::testing::Test {
 public:
  void SetUp() {}
};

TEST_F(ValidationCheckerTest, valid_centripedal_acc) {
  FLAGS_centripedal_acc_threshold = 2.0;
  std::vector<TrajectoryPoint> trajectory_points(0);
  double theta = 0.0;
  double relative_time = 0.1;
  for (int i = 0; i < 3; ++i) {
    TrajectoryPoint trajectory_point;
    trajectory_point.mutable_path_point()->set_theta(theta);
    trajectory_point.set_v(1.0);
    trajectory_point.set_relative_time(i * relative_time);
    theta += 0.1;
    trajectory_points.emplace_back(std::move(trajectory_point));
  }
  EXPECT_EQ(trajectory_points.size(), 3);
  EXPECT_TRUE(
      ValidationChecker::ValidCentripetalAcceleration(trajectory_points));
}

TEST_F(ValidationCheckerTest, invalid_centripedal_acc) {
  FLAGS_centripedal_acc_threshold = 2.0;
  std::vector<TrajectoryPoint> trajectory_points(0);
  double theta = 0.0;
  double relative_time = 0.1;
  for (int i = 0; i < 3; ++i) {
    TrajectoryPoint trajectory_point;
    trajectory_point.mutable_path_point()->set_theta(theta);
    trajectory_point.set_v(1.0);
    trajectory_point.set_relative_time(i * relative_time);
    theta += 3.14;
    trajectory_points.emplace_back(std::move(trajectory_point));
  }
  EXPECT_EQ(trajectory_points.size(), 3);
  EXPECT_TRUE(
      !ValidationChecker::ValidCentripetalAcceleration(trajectory_points));
}

TEST_F(ValidationCheckerTest, valid_trajectory_point) {
  TrajectoryPoint trajectory_point;
  trajectory_point.mutable_path_point()->set_x(0.0);
  trajectory_point.mutable_path_point()->set_y(0.0);
  trajectory_point.mutable_path_point()->set_theta(0.0);
  trajectory_point.set_v(0.0);
  trajectory_point.set_a(0.0);
  trajectory_point.set_relative_time(0.0);
  EXPECT_TRUE(ValidationChecker::ValidTrajectoryPoint(trajectory_point));
}

TEST_F(ValidationCheckerTest, invalid_trajectory_point) {
  TrajectoryPoint trajectory_point;
  EXPECT_FALSE(ValidationChecker::ValidTrajectoryPoint(trajectory_point));

  trajectory_point.mutable_path_point()->set_x(
      std::numeric_limits<double>::quiet_NaN());
  trajectory_point.mutable_path_point()->set_y(0.0);
  trajectory_point.mutable_path_point()->set_theta(0.0);
  trajectory_point.set_v(0.0);
  trajectory_point.set_a(0.0);
  trajectory_point.set_relative_time(0.0);
  EXPECT_FALSE(ValidationChecker::ValidTrajectoryPoint(trajectory_point));
}

}  // namespace prediction
}  // namespace apollo
