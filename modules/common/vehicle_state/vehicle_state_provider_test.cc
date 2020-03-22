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

#include "modules/common/vehicle_state/vehicle_state_provider.h"

#include <string>

#include "Eigen/Core"
#include "gtest/gtest.h"

#include "cyber/common/file.h"
#include "modules/canbus/proto/chassis.pb.h"
#include "modules/localization/common/localization_gflags.h"
#include "modules/localization/proto/localization.pb.h"

namespace apollo {
namespace common {
namespace vehicle_state_provider {

using apollo::canbus::Chassis;
using apollo::localization::LocalizationEstimate;

class VehicleStateProviderTest : public ::testing::Test {
 public:
  virtual void SetUp() {
    std::string localization_file =
        "modules/localization/testdata/3_localization_result_1.pb.txt";
    ACHECK(cyber::common::GetProtoFromFile(localization_file, &localization_));
    chassis_.set_speed_mps(3.0);
    chassis_.set_gear_location(canbus::Chassis::GEAR_DRIVE);
    FLAGS_enable_map_reference_unify = false;
  }

 protected:
  LocalizationEstimate localization_;
  Chassis chassis_;
};

TEST_F(VehicleStateProviderTest, Accessors) {
  auto vehicle_state_provider = VehicleStateProvider::Instance();
  vehicle_state_provider->Update(localization_, chassis_);
  EXPECT_DOUBLE_EQ(vehicle_state_provider->x(), 357.51331791372041);
  EXPECT_DOUBLE_EQ(vehicle_state_provider->y(), 96.165912376788725);
  EXPECT_DOUBLE_EQ(vehicle_state_provider->heading(), -1.8388082455104939);
  EXPECT_DOUBLE_EQ(vehicle_state_provider->roll(), 0.047026695713820919);
  EXPECT_DOUBLE_EQ(vehicle_state_provider->pitch(), -0.010712737572581465);
  EXPECT_DOUBLE_EQ(vehicle_state_provider->yaw(), 2.8735807348741953);
  EXPECT_DOUBLE_EQ(vehicle_state_provider->linear_velocity(), 3.0);
  EXPECT_DOUBLE_EQ(vehicle_state_provider->angular_velocity(),
                   -0.0079623083093763921);
  EXPECT_DOUBLE_EQ(vehicle_state_provider->linear_acceleration(),
                   -0.079383290718229638);
  EXPECT_DOUBLE_EQ(vehicle_state_provider->gear(), canbus::Chassis::GEAR_DRIVE);
}

TEST_F(VehicleStateProviderTest, EstimateFuturePosition) {
  auto vehicle_state_provider = VehicleStateProvider::Instance();
  vehicle_state_provider->Update(localization_, chassis_);
  common::math::Vec2d future_position =
      vehicle_state_provider->EstimateFuturePosition(1.0);
  EXPECT_NEAR(future_position.x(), 356.707, 1e-3);
  EXPECT_NEAR(future_position.y(), 93.276, 1e-3);
  future_position = vehicle_state_provider->EstimateFuturePosition(2.0);
  EXPECT_NEAR(future_position.x(), 355.879, 1e-3);
  EXPECT_NEAR(future_position.y(), 90.393, 1e-3);
}

}  // namespace vehicle_state_provider
}  // namespace common
}  // namespace apollo
