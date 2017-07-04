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

#include "modules/common/vehicle_state/vehicle_state.h"

#include <string>

#include "Eigen/Core"
#include "gtest/gtest.h"

#include "modules/canbus/proto/chassis.pb.h"
#include "modules/common/util/file.h"
#include "modules/localization/common/localization_gflags.h"
#include "modules/localization/proto/localization.pb.h"

namespace apollo {
namespace common {
namespace vehicle_state {

using apollo::localization::LocalizationEstimate;
using apollo::canbus::Chassis;

class VehicleStateTest : public ::testing::Test {
 public:
  virtual void SetUp() {
    std::string localization_file =
        "modules/localization/testdata/3_localization_result_1.pb.txt";
    CHECK(::apollo::common::util::GetProtoFromFile(localization_file,
                                                         &localization_));
    chassis_.set_speed_mps(3.0);
    FLAGS_enable_map_reference_unify = false;
  }

 protected:
  LocalizationEstimate localization_;
  Chassis chassis_;
};

TEST_F(VehicleStateTest, Accessors) {
  VehicleState vehicle_state(&localization_, &chassis_);
  EXPECT_DOUBLE_EQ(vehicle_state.x(), 357.51331791372041);
  EXPECT_DOUBLE_EQ(vehicle_state.y(), 96.165912376788725);
  EXPECT_DOUBLE_EQ(vehicle_state.heading(), -1.8388082455104939);
  EXPECT_DOUBLE_EQ(vehicle_state.linear_velocity(), 3.0);
  EXPECT_DOUBLE_EQ(vehicle_state.angular_velocity(), -0.0079623083093763921);
  EXPECT_DOUBLE_EQ(vehicle_state.linear_acceleration(), -0.079383290718229638);
}

TEST_F(VehicleStateTest, EstimateFuturePosition) {
  VehicleState vehicle_state(&localization_, &chassis_);
  Eigen::Vector2d future_position = vehicle_state.EstimateFuturePosition(1.0);
  EXPECT_NEAR(future_position[0], 356.707, 1e-3);
  EXPECT_NEAR(future_position[1], 93.276, 1e-3);
  future_position = vehicle_state.EstimateFuturePosition(2.0);
  EXPECT_NEAR(future_position[0], 355.879, 1e-3);
  EXPECT_NEAR(future_position[1], 90.393, 1e-3);
}

}  // namespace vehicle_state
}  // namespace common
}  // namespace apollo
