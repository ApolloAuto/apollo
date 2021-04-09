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

#include "modules/planning/scenarios/scenario_manager.h"

#include <memory>

#include "gtest/gtest.h"

namespace apollo {
namespace planning {

class ScenarioManagerTest : public ::testing::Test {
 public:
  virtual void SetUp() {}

 protected:
  ScenarioManager scenario_manager_;
};

TEST_F(ScenarioManagerTest, Simple) {
  EXPECT_TRUE(scenario_manager_.Init());
  common::TrajectoryPoint tp;

  uint32_t sequence_num = 10;
  const double start_time = 123.45;
  common::VehicleState vehicle_state;
  ReferenceLineProvider reference_line_provider;
  Frame frame(sequence_num, tp, start_time, vehicle_state,
              &reference_line_provider);

  scenario_manager_.Update(tp, frame);
}

}  // namespace planning
}  // namespace apollo
