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

#include "modules/localization/ndt/localization_pose_buffer.h"

#include <memory>

#include "Eigen/Core"
#include "Eigen/Geometry"
#include "gtest/gtest.h"

#include "modules/common/time/time.h"

namespace apollo {
namespace localization {
namespace ndt {

class NDTLocalizationTest : public ::testing::Test {
 public:
  virtual void SetUp() { pose_buffer_ptr_.reset(new LocalizationPoseBuffer()); }
  virtual void TearDown() {}

 protected:
  std::unique_ptr<LocalizationPoseBuffer> pose_buffer_ptr_;
};

TEST_F(NDTLocalizationTest, UpdateLidarPose) {
  Eigen::Affine3d odometry_pose = Eigen::Affine3d::Identity();
  Eigen::Affine3d lidar_pose = Eigen::Affine3d::Identity();
  double time_now = apollo::common::time::Clock::NowInSeconds();
  pose_buffer_ptr_->UpdateLidarPose(time_now, lidar_pose, odometry_pose);
  time_now = apollo::common::time::Clock::NowInSeconds();
  pose_buffer_ptr_->UpdateLidarPose(time_now, lidar_pose, odometry_pose);
  EXPECT_EQ(pose_buffer_ptr_->GetUsedBufferSize(), 2);
  EXPECT_EQ(pose_buffer_ptr_->GetHeadIndex(), 0);
}

TEST_F(NDTLocalizationTest, UpdateOdometryPose) {
  Eigen::Affine3d odometry_pose = Eigen::Affine3d::Identity();
  Eigen::Affine3d lidar_pose = Eigen::Affine3d::Identity();
  double time_now = apollo::common::time::Clock::NowInSeconds();
  pose_buffer_ptr_->UpdateLidarPose(time_now, lidar_pose, odometry_pose);
  time_now = apollo::common::time::Clock::NowInSeconds();
  pose_buffer_ptr_->UpdateLidarPose(time_now, lidar_pose, odometry_pose);
  odometry_pose.translation()[0] = 1.0;
  odometry_pose.translation()[1] = 0.0;
  odometry_pose.translation()[2] = 0.0;

  time_now = apollo::common::time::Clock::NowInSeconds();
  Eigen::Affine3d pose =
      pose_buffer_ptr_->UpdateOdometryPose(time_now, odometry_pose);
  EXPECT_LE(std::abs(pose.translation()[0] - 1.0), 1e-5);
}

}  // namespace ndt
}  // namespace localization
}  // namespace apollo
