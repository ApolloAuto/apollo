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

#include "modules/planning/tasks/deciders/path_reuse_decider/path_reuse_decider.h"

#include "gtest/gtest.h"
#include "modules/planning/proto/planning_config.pb.h"

namespace apollo {
namespace planning {

class PathReuseDeciderTest : public ::testing::Test {
 public:
  virtual void SetUp() {
    config_.set_task_type(TaskConfig::PATH_REUSE_DECIDER);
    config_.mutable_path_reuse_decider_config();
  }

  virtual void TearDown() {}

 protected:
  TaskConfig config_;
};

TEST_F(PathReuseDeciderTest, Init) {
  PathReuseDecider path_reuse_decider(config_);
  EXPECT_EQ(path_reuse_decider.Name(),
            TaskConfig::TaskType_Name(config_.task_type()));
}

// TEST_F(PathReuseDeciderTest, GetHistoryStopPositions) {
//   PathReuseDecider path_reuse_decider(config_);
//   History* history = History::Instance();
//   ADCTrajectory adc_trajectory;
//   EXPECT_TRUE(apollo::cyber::common::GetProtoFromFile(
//       "/apollo/modules/planning/testdata/common/history_01.pb.txt",
//       &adc_trajectory));
//   history->Add(adc_trajectory);
//   const std::vector<const HistoryObjectDecision*>
//       history_stop_objects_decisions =
//           history->GetLastFrame()->GetStopObjectDecisions();
//   std::vector<const common::PointENU*> history_stop_positions;
//   path_reuse_decider.GetHistoryStopPositions(history_stop_objects_decisions,
//                                              &history_stop_positions);
//   common::PointENU stop_position;
//   double stop_x = 586261.33054620528;
//   double stop_y = 4141304.5678338786;
//   stop_position.set_x(stop_x);
//   stop_position.set_y(stop_y);
//   const common::PointENU* point = history_stop_positions[1];
//   EXPECT_DOUBLE_EQ(point->x(), stop_position.x());
//   EXPECT_DOUBLE_EQ(point->y(), stop_position.y());
// }

}  // namespace planning
}  // namespace apollo
