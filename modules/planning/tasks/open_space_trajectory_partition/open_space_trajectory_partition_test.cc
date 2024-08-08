
/******************************************************************************
 * Copyright 2019 The Apollo Authors. All Rights Reserved.
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

#include "modules/planning/tasks/open_space_trajectory_partition/open_space_trajectory_partition.h"

#include "gtest/gtest.h"

namespace apollo {
namespace planning {

class OpenSpaceTrajectoryPartitionTest : public ::testing::Test {
 public:
  virtual void SetUp() { injector_ = std::make_shared<DependencyInjector>(); }

 protected:
  OpenSpaceTrajectoryPartitionConfig config_;
  std::shared_ptr<DependencyInjector> injector_;
};

TEST_F(OpenSpaceTrajectoryPartitionTest, Init) {
  OpenSpaceTrajectoryPartition open_space_trajectory_partition;
  open_space_trajectory_partition.Init(
      "/scenarios/valet_parking/conf/valet_parking_parking",
      "OPEN_SPACE_TRAJECTORY_PARTITION", injector_);
  EXPECT_EQ(open_space_trajectory_partition.Name(),
            "OPEN_SPACE_TRAJECTORY_PARTITION");
}

}  // namespace planning
}  // namespace apollo
