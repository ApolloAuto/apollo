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
#include "modules/planning/tasks/open_space_roi_decider/open_space_roi_decider.h"

#include "gtest/gtest.h"

namespace apollo {
namespace planning {

class OpenSpaceRoiDeciderTest : public ::testing::Test {
 public:
  virtual void SetUp() { injector_ = std::make_shared<DependencyInjector>(); }

 protected:
  OpenSpaceRoiDeciderConfig config_;
  std::shared_ptr<DependencyInjector> injector_;
};

TEST_F(OpenSpaceRoiDeciderTest, Init) {
  OpenSpaceRoiDecider open_space_roi_decider;
  open_space_roi_decider.Init(
      "scenarios/lane_follow_scenario/conf/lane_follow_stage",
      "OPEN_SPACE_ROI_DECIDER", injector_);
  EXPECT_EQ(open_space_roi_decider.Name(), "OPEN_SPACE_ROI_DECIDER");
}

}  // namespace planning
}  // namespace apollo
