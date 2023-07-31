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
#include "modules/planning/scenarios/valet_parking/stage_approaching_parking_spot.h"
#include "gtest/gtest.h"
#include "modules/planning/planning_base/proto/planning_config.pb.h"

namespace apollo {
namespace planning {

class StageApproachingParkingSpotTest : public ::testing::Test {
 public:
  virtual void SetUp() {
    config_.set_name("VALET_PARKING_APPROACHING_PARKING_SPOT");
    injector_ = std::make_shared<DependencyInjector>();
  }

 protected:
  StagePipeline config_;
  std::shared_ptr<DependencyInjector> injector_;
  ValetParkingContext context_;
};

TEST_F(StageApproachingParkingSpotTest, Init) {
  StageApproachingParkingSpot stage_approaching_parking_spot;
  stage_approaching_parking_spot.Init(
      config_, injector_, "scenarios/valet_parking/conf", &context_);
  EXPECT_EQ(stage_approaching_parking_spot.Name(),
            "VALET_PARKING_APPROACHING_PARKING_SPOT");
}

}  // namespace planning
}  // namespace apollo
