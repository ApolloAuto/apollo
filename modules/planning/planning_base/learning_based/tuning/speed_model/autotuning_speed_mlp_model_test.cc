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
 * @file: testing casese for mlp net model
 **/

#include "modules/planning/planning_base/learning_based/tuning/speed_model/autotuning_speed_mlp_model.h"

#include "gtest/gtest.h"

namespace apollo {
namespace planning {

TEST(AutotuningSpeedMLPModel, test_case_one) {
  AutotuningSpeedMLPModel speed_model;
  EXPECT_TRUE(speed_model.SetParams().ok());
}

}  // namespace planning
}  // namespace apollo
