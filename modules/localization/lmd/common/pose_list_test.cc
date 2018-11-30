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

#include "modules/localization/lmd/common/pose_list.h"

#include "gtest/gtest.h"

namespace apollo {
namespace localization {

TEST(PoseListTest, FindMatchingPose) {
  Pose pose1;
  pose1.mutable_position()->set_x(0.0);
  pose1.mutable_position()->set_y(0.0);
  pose1.mutable_position()->set_z(0.0);
  pose1.mutable_orientation()->set_qw(1.0);
  pose1.mutable_orientation()->set_qx(0.0);
  pose1.mutable_orientation()->set_qy(0.0);
  pose1.mutable_orientation()->set_qz(0.0);
  pose1.mutable_linear_velocity()->set_x(0.0);
  pose1.mutable_linear_velocity()->set_y(0.0);
  pose1.mutable_linear_velocity()->set_z(0.0);
  pose1.mutable_angular_velocity()->set_x(0.0);
  pose1.mutable_angular_velocity()->set_y(0.0);
  pose1.mutable_angular_velocity()->set_z(0.0);
  pose1.mutable_linear_acceleration()->set_x(0.0);
  pose1.mutable_linear_acceleration()->set_y(0.0);
  pose1.mutable_linear_acceleration()->set_z(0.0);
  pose1.mutable_euler_angles()->set_x(0.0);
  pose1.mutable_euler_angles()->set_y(0.0);
  pose1.mutable_euler_angles()->set_z(0.0);

  Pose pose2;
  pose2.mutable_position()->set_x(1.0);
  pose2.mutable_position()->set_y(1.0);
  pose2.mutable_position()->set_z(1.0);
  pose2.mutable_orientation()->set_qw(0.707);
  pose2.mutable_orientation()->set_qx(0.0);
  pose2.mutable_orientation()->set_qy(0.0);
  pose2.mutable_orientation()->set_qz(0.707);
  pose2.mutable_linear_velocity()->set_x(1.0);
  pose2.mutable_linear_velocity()->set_y(1.0);
  pose2.mutable_linear_velocity()->set_z(1.0);
  pose2.mutable_angular_velocity()->set_x(1.0);
  pose2.mutable_angular_velocity()->set_y(1.0);
  pose2.mutable_angular_velocity()->set_z(1.0);
  pose2.mutable_linear_acceleration()->set_x(1.0);
  pose2.mutable_linear_acceleration()->set_y(1.0);
  pose2.mutable_linear_acceleration()->set_z(1.0);
  pose2.mutable_euler_angles()->set_x(1.0);
  pose2.mutable_euler_angles()->set_y(1.0);
  pose2.mutable_euler_angles()->set_z(1.0);

  PoseList pose_list(100.0);
  pose_list.Push(0.0, pose1);
  pose_list.Push(1.0, pose2);

  Pose pose;
  EXPECT_TRUE(pose_list.FindMatchingPose(0.333, &pose));
  EXPECT_NEAR(0.333, pose.position().x(), 1e-10);
  EXPECT_NEAR(0.333, pose.position().y(), 1e-10);
  EXPECT_NEAR(0.333, pose.position().z(), 1e-10);
  EXPECT_NEAR(0.965981, pose.orientation().qw(), 1e-6);
  EXPECT_NEAR(0.0, pose.orientation().qx(), 1e-6);
  EXPECT_NEAR(0.0, pose.orientation().qy(), 1e-6);
  EXPECT_NEAR(0.258537, pose.orientation().qz(), 1e-6);
  EXPECT_NEAR(0.333, pose.linear_velocity().x(), 1e-10);
  EXPECT_NEAR(0.333, pose.linear_velocity().y(), 1e-10);
  EXPECT_NEAR(0.333, pose.linear_velocity().z(), 1e-10);
  EXPECT_NEAR(0.333, pose.angular_velocity().x(), 1e-10);
  EXPECT_NEAR(0.333, pose.angular_velocity().y(), 1e-10);
  EXPECT_NEAR(0.333, pose.angular_velocity().z(), 1e-10);
  EXPECT_NEAR(0.333, pose.linear_acceleration().x(), 1e-10);
  EXPECT_NEAR(0.333, pose.linear_acceleration().y(), 1e-10);
  EXPECT_NEAR(0.333, pose.linear_acceleration().z(), 1e-10);
  EXPECT_NEAR(0.333, pose.euler_angles().x(), 1e-10);
  EXPECT_NEAR(0.333, pose.euler_angles().y(), 1e-10);
  EXPECT_NEAR(0.333, pose.euler_angles().z(), 1e-10);

  EXPECT_TRUE(pose_list.FindMatchingPose(1e-11, &pose));
  EXPECT_NEAR(0.0, pose.position().x(), 1e-10);
  EXPECT_NEAR(0.0, pose.position().y(), 1e-10);
  EXPECT_NEAR(0.0, pose.position().z(), 1e-10);

  EXPECT_TRUE(pose_list.FindMatchingPose(1.0, &pose));
  EXPECT_NEAR(1.0, pose.position().x(), 1e-10);
  EXPECT_NEAR(1.0, pose.position().y(), 1e-10);
  EXPECT_NEAR(1.0, pose.position().z(), 1e-10);

  EXPECT_FALSE(pose_list.FindMatchingPose(-1.0, &pose));
  EXPECT_FALSE(pose_list.FindMatchingPose(2.0, &pose));
}

TEST(PoseListTest, FindNearestPose) {
  PoseList pose_list(100.0);

  Pose pose;
  EXPECT_FALSE(pose_list.FindNearestPose(0.0, &pose));

  Pose pose1;
  pose1.mutable_position()->set_x(0.0);
  pose1.mutable_position()->set_y(0.0);
  pose1.mutable_position()->set_z(0.0);
  pose_list.Push(10.0, pose1);
  Pose pose2;
  pose2.mutable_position()->set_x(1.0);
  pose2.mutable_position()->set_y(1.0);
  pose2.mutable_position()->set_z(1.0);
  pose_list.Push(11.0, pose2);

  EXPECT_TRUE(pose_list.FindNearestPose(9.0, &pose));
  EXPECT_NEAR(0.0, pose.position().x(), 1e-10);
  EXPECT_NEAR(0.0, pose.position().y(), 1e-10);
  EXPECT_NEAR(0.0, pose.position().z(), 1e-10);

  EXPECT_TRUE(pose_list.FindNearestPose(14.0, &pose));
  EXPECT_NEAR(1.0, pose.position().x(), 1e-10);
  EXPECT_NEAR(1.0, pose.position().y(), 1e-10);
  EXPECT_NEAR(1.0, pose.position().z(), 1e-10);

  EXPECT_TRUE(pose_list.FindNearestPose(10.5, &pose));
  EXPECT_NEAR(0.5, pose.position().x(), 1e-10);
  EXPECT_NEAR(0.5, pose.position().y(), 1e-10);
  EXPECT_NEAR(0.5, pose.position().z(), 1e-10);
}

}  // namespace localization
}  // namespace apollo
