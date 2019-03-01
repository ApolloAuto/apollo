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

#include "modules/planning/common/speed_profile_generator.h"

#include "gtest/gtest.h"

#include "modules/planning/common/ego_info.h"
#include "modules/planning/common/planning_gflags.h"

namespace apollo {
namespace planning {

TEST(SpeedProfileGeneratorTest, GenerateFallbackSpeedProfile) {
  auto speed_data = SpeedProfileGenerator::GenerateFallbackSpeedProfile();
  EXPECT_FALSE(speed_data.empty());

  common::TrajectoryPoint adc_planning_point;
  adc_planning_point.set_v(FLAGS_polynomial_speed_fallback_velocity + 0.1);

  common::VehicleState vs;

  EgoInfo::Instance()->Update(adc_planning_point, vs);
  auto speed_data2 = SpeedProfileGenerator::GenerateFallbackSpeedProfile();
  EXPECT_FALSE(speed_data2.empty());
}

}  // namespace planning
}  // namespace apollo
