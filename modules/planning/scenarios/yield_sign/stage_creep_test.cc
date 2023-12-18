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

#include "modules/planning/scenarios/yield_sign/stage_creep.h"
#include "gtest/gtest.h"
#include "cyber/common/file.h"
#include "cyber/common/log.h"
#include "modules/planning/planning_base/gflags/planning_gflags.h"

namespace apollo {
namespace planning {

class YieldSignStageCreepTest : public ::testing::Test {
 public:
  virtual void SetUp() {
    config_.set_name("YIELD_SIGN_CREEP");
    injector_ = std::make_shared<DependencyInjector>();
  }

 protected:
  StagePipeline config_;
  std::shared_ptr<DependencyInjector> injector_;
  YieldSignContext context_;
};

TEST_F(YieldSignStageCreepTest, Init) {
  YieldSignStageCreep yield_sign_stage_creep;
  yield_sign_stage_creep.Init(config_, injector_, "scenarios/yield_sign/conf",
                              &context_);
  EXPECT_EQ(yield_sign_stage_creep.Name(), ("YIELD_SIGN_CREEP"));
}

}  // namespace planning
}  // namespace apollo
