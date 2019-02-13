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

#include "modules/planning/common/trajectory/publishable_trajectory.h"

#include "cyber/common/file.h"
#include "gtest/gtest.h"

#include "modules/common/util/util.h"

namespace apollo {
namespace planning {

TEST(basic_test, DiscretizedTrajectory) {
  const std::string path_of_standard_trajectory =
      "modules/planning/testdata/trajectory_data/standard_trajectory.pb.txt";
  ADCTrajectory trajectory;
  EXPECT_TRUE(cyber::common::GetProtoFromFile(path_of_standard_trajectory,
                                              &trajectory));
  DiscretizedTrajectory discretized_trajectory(trajectory);

  PublishableTrajectory publishable_trajectory(12349834.26,
                                               discretized_trajectory);
  EXPECT_EQ(publishable_trajectory.header_time(), 12349834.26);

  ADCTrajectory output_trajectory;
  publishable_trajectory.PopulateTrajectoryProtobuf(&output_trajectory);

  for (int i = 0; i < output_trajectory.trajectory_point_size(); ++i) {
    EXPECT_TRUE(apollo::common::util::IsProtoEqual(
        output_trajectory.trajectory_point(i), trajectory.trajectory_point(i)));
  }
}

}  // namespace planning
}  // namespace apollo
