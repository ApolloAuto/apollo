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
#include "modules/planning/tasks/open_space_fallback_decider/open_space_fallback_decider.h"

#include "gtest/gtest.h"
#include "modules/planning/tasks/open_space_fallback_decider/proto/open_space_fallback_decider.pb.h"

namespace apollo {
namespace planning {

class OpenSpaceFallbackDeciderTest : public ::testing::Test {
 public:
  virtual void SetUp() {}

 protected:
  OpenSpaceFallBackDeciderConfig config_;
};

TEST_F(OpenSpaceFallbackDeciderTest, Init) {
  auto injector = std::make_shared<DependencyInjector>();
  OpenSpaceFallbackDecider open_space_fallback_decider;
  open_space_fallback_decider.Init(
      "scenarios/valet_parking/conf/valet_parking_stage",
      "OPEN_SPACE_FALLBACK_DECIDER", injector);
  EXPECT_EQ(open_space_fallback_decider.Name(), "OPEN_SPACE_FALLBACK_DECIDER");
}

}  // namespace planning
}  // namespace apollo
