/******************************************************************************
 * Copyright 2017 The Apollo Authors. All Rights Reserved.
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

#include "modules/planning/planning_base/common/frame.h"

#include "gtest/gtest.h"

#include "modules/common_msgs/perception_msgs/perception_obstacle.pb.h"
#include "modules/common_msgs/prediction_msgs/prediction_obstacle.pb.h"

#include "cyber/common/file.h"
#include "modules/common/util/util.h"
#include "modules/planning/planning_base/gflags/planning_gflags.h"

namespace apollo {
namespace planning {

class FrameTest : public ::testing::Test {
 public:
  virtual void SetUp() {
    ASSERT_TRUE(cyber::common::GetProtoFromFile(
        "/apollo/modules/planning/planning_base/testdata/common/"
        "sample_prediction.pb.txt",
        &prediction_obstacles_));
  }

 protected:
  prediction::PredictionObstacles prediction_obstacles_;
};

TEST_F(FrameTest, AlignPredictionTime) {
  int first_traj_size = prediction_obstacles_.prediction_obstacle(0)
                            .trajectory(0)
                            .trajectory_point_size();
  double origin_pred_time = prediction_obstacles_.header().timestamp_sec();
  Frame::AlignPredictionTime(origin_pred_time + 0.1, &prediction_obstacles_);
  ASSERT_EQ(first_traj_size - 1, prediction_obstacles_.prediction_obstacle(0)
                                     .trajectory(0)
                                     .trajectory_point_size());

  Frame::AlignPredictionTime(origin_pred_time + 0.5, &prediction_obstacles_);
  ASSERT_EQ(first_traj_size - 3, prediction_obstacles_.prediction_obstacle(0)
                                     .trajectory(0)
                                     .trajectory_point_size());

  Frame::AlignPredictionTime(origin_pred_time + 12.0, &prediction_obstacles_);
  ASSERT_EQ(0, prediction_obstacles_.prediction_obstacle(0)
                   .trajectory(0)
                   .trajectory_point_size());
}

}  // namespace planning
}  // namespace apollo
