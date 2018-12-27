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

#include "modules/localization/lmd/predictor/output/predictor_output.h"

#include "gtest/gtest.h"

namespace apollo {
namespace localization {

using apollo::common::Status;

class PredictorOutputTest : public ::testing::Test {
 public:
  PredictorOutputTest()
      : predictor_(10.0, [&](double, const Pose&) { return Status::OK(); }) {}

 protected:
  PredictorOutput predictor_;
};

TEST_F(PredictorOutputTest, PredictByImu1) {
  auto& imu = predictor_.dep_predicteds_[kPredictorImuName];
  imu.clear();
  Pose imu_pose;
  imu_pose.mutable_linear_acceleration()->set_x(1.0);
  imu_pose.mutable_linear_acceleration()->set_y(1.0);
  imu_pose.mutable_linear_acceleration()->set_z(0.0);
  imu_pose.mutable_angular_velocity()->set_x(0.0);
  imu_pose.mutable_angular_velocity()->set_y(0.0);
  imu_pose.mutable_angular_velocity()->set_z(0.0);
  imu.Push(0.0, imu_pose);
  imu_pose.mutable_linear_acceleration()->set_x(0.5);
  imu_pose.mutable_linear_acceleration()->set_y(0.5);
  imu_pose.mutable_linear_acceleration()->set_z(0.0);
  imu.Push(0.5, imu_pose);
  imu.Push(3.0, imu_pose);

  Pose old_pose;
  old_pose.mutable_position()->set_x(0.0);
  old_pose.mutable_position()->set_y(0.0);
  old_pose.mutable_position()->set_z(0.0);
  old_pose.mutable_orientation()->set_qw(1.0);
  old_pose.mutable_orientation()->set_qx(0.0);
  old_pose.mutable_orientation()->set_qy(0.0);
  old_pose.mutable_orientation()->set_qz(0.0);
  old_pose.mutable_linear_velocity()->set_x(0.0);
  old_pose.mutable_linear_velocity()->set_y(0.0);
  old_pose.mutable_linear_velocity()->set_z(0.0);

  Pose new_pose;
  EXPECT_TRUE(predictor_.PredictByImu(0.0, old_pose, 1.0, &new_pose));
  EXPECT_NEAR(0.372, new_pose.position().x(), 1e-3);
  EXPECT_NEAR(0.083, new_pose.position().y(), 1e-3);
  EXPECT_NEAR(0.0, new_pose.position().z(), 1e-3);
  EXPECT_NEAR(0.682, new_pose.linear_velocity().x(), 1e-3);
  EXPECT_NEAR(-0.052, new_pose.linear_velocity().y(), 1e-3);
  EXPECT_NEAR(0.0, new_pose.linear_velocity().z(), 1e-3);
}

TEST_F(PredictorOutputTest, PredictByImu2) {
  auto& imu = predictor_.dep_predicteds_[kPredictorImuName];
  imu.clear();
  Pose imu_pose;
  imu_pose.mutable_linear_acceleration()->set_x(1.0);
  imu_pose.mutable_linear_acceleration()->set_y(1.0);
  imu_pose.mutable_linear_acceleration()->set_z(0.0);
  imu_pose.mutable_angular_velocity()->set_x(0.0);
  imu_pose.mutable_angular_velocity()->set_y(0.0);
  imu_pose.mutable_angular_velocity()->set_z(0.0);
  imu.Push(0.0, imu_pose);
  imu_pose.mutable_linear_acceleration()->set_x(1.0);
  imu_pose.mutable_linear_acceleration()->set_y(1.0);
  imu_pose.mutable_linear_acceleration()->set_z(0.0);
  imu.Push(1.0, imu_pose);

  Pose old_pose;
  old_pose.mutable_position()->set_x(0.0);
  old_pose.mutable_position()->set_y(0.0);
  old_pose.mutable_position()->set_z(0.0);
  old_pose.mutable_orientation()->set_qw(1.0);
  old_pose.mutable_orientation()->set_qx(0.0);
  old_pose.mutable_orientation()->set_qy(0.0);
  old_pose.mutable_orientation()->set_qz(0.0);
  old_pose.mutable_linear_velocity()->set_x(0.0);
  old_pose.mutable_linear_velocity()->set_y(0.0);
  old_pose.mutable_linear_velocity()->set_z(0.0);

  Pose new_pose;
  EXPECT_TRUE(predictor_.PredictByImu(0.0, old_pose, 1.0, &new_pose));
  EXPECT_NEAR(0.515, new_pose.position().x(), 1e-3);
  EXPECT_NEAR(0.184, new_pose.position().y(), 1e-3);
  EXPECT_NEAR(0.0, new_pose.position().z(), 1e-3);
  EXPECT_NEAR(1.047, new_pose.linear_velocity().x(), 1e-3);
  EXPECT_NEAR(0.052, new_pose.linear_velocity().y(), 1e-3);
  EXPECT_NEAR(0.0, new_pose.linear_velocity().z(), 1e-3);
}

}  // namespace localization
}  // namespace apollo
