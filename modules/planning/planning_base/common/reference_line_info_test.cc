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

#include "modules/planning/planning_base/common/reference_line_info.h"

#include "gtest/gtest.h"

#include "modules/common_msgs/planning_msgs/planning.pb.h"

namespace apollo {
namespace planning {

class ReferenceLineInfoTest : public ::testing::Test {
 public:
  virtual void SetUp() {}

 protected:
  ReferenceLineInfo reference_line_info_;
};

TEST_F(ReferenceLineInfoTest, BasicTest) {
  EXPECT_EQ(reference_line_info_.trajectory_type(), ADCTrajectory::UNKNOWN);

  reference_line_info_.set_trajectory_type(ADCTrajectory::NORMAL);
  EXPECT_EQ(reference_line_info_.trajectory_type(), ADCTrajectory::NORMAL);

  reference_line_info_.set_trajectory_type(ADCTrajectory::PATH_FALLBACK);
  EXPECT_EQ(reference_line_info_.trajectory_type(),
            ADCTrajectory::PATH_FALLBACK);

  reference_line_info_.set_trajectory_type(ADCTrajectory::SPEED_FALLBACK);
  EXPECT_EQ(reference_line_info_.trajectory_type(),
            ADCTrajectory::SPEED_FALLBACK);
}

}  // namespace planning
}  // namespace apollo
