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
#include "modules/planning/scenarios/park_and_go/stage_check.h"

#include "gtest/gtest.h"
#include "modules/planning/scenarios/park_and_go/context.h"

namespace apollo {
namespace planning {

class ParkAndGoStageCheckTest : public ::testing::Test {
 public:
  virtual void SetUp() {
    config_.set_name("PARK_AND_GO_CHECK");
    injector_ = std::make_shared<DependencyInjector>();
  }

 protected:
  StagePipeline config_;
  std::shared_ptr<DependencyInjector> injector_;
  ParkAndGoContext context_;
};

TEST_F(ParkAndGoStageCheckTest, Init) {
  ParkAndGoStageCheck park_and_go_stage_check;
  park_and_go_stage_check.Init(config_, injector_, "scenarios/park_and_go/conf",
                               &context_);
  EXPECT_EQ(park_and_go_stage_check.Name(), ("PARK_AND_GO_CHECK"));
}

}  // namespace planning
}  // namespace apollo
