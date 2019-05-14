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

#include "modules/common/vehicle_model/vehicle_model.h"

#include "cyber/common/file.h"
#include "gtest/gtest.h"
#include "modules/common/vehicle_state/proto/vehicle_state.pb.h"
#include "modules/common/vehicle_state/vehicle_state_provider.h"

namespace apollo {
namespace common {

using apollo::canbus::Chassis;
using apollo::localization::LocalizationEstimate;

class VehicleModelTest : public ::testing::Test {
 public:
  virtual void SetUp() {
    std::string localization_pre_file =
        "modules/common/testdata/localization_pre.pb.txt";
    CHECK(cyber::common::GetProtoFromFile(localization_pre_file,
                                          &localization_pre_));
    std::string localization_post_file =
        "modules/common/testdata/localization_post.pb.txt";
    CHECK(cyber::common::GetProtoFromFile(localization_post_file,
                                          &localization_post_));
    const std::string chassis_pre_file =
        "modules/common/testdata/chassis_pre.pb.txt";
    CHECK(cyber::common::GetProtoFromFile(chassis_pre_file, &chassis_pre_));
    const std::string chassis_post_file =
        "modules/common/testdata/chassis_post.pb.txt";
    CHECK(cyber::common::GetProtoFromFile(chassis_post_file, &chassis_post_));
  }

 protected:
  LocalizationEstimate localization_pre_;
  LocalizationEstimate localization_post_;
  Chassis chassis_pre_;
  Chassis chassis_post_;
  VehicleState cur_vehicle_state_;
  VehicleState predicted_vehicle_state_;
  VehicleState expected_vehicle_state_;
};

TEST_F(VehicleModelTest, RearCenteredKinematicBicycleModel) {
  double predicted_time_horizon = localization_post_.measurement_time() -
                                  localization_pre_.measurement_time();

  auto vehicle_state_provider = VehicleStateProvider::Instance();

  vehicle_state_provider->Update(localization_pre_, chassis_pre_);
  cur_vehicle_state_ = vehicle_state_provider->vehicle_state();

  vehicle_state_provider->Update(localization_post_, chassis_post_);
  expected_vehicle_state_ = vehicle_state_provider->vehicle_state();

  VehicleState predicted_vehicle_state_ =
      VehicleModel::Predict(predicted_time_horizon, cur_vehicle_state_);
  EXPECT_NEAR(expected_vehicle_state_.x(), predicted_vehicle_state_.x(), 2e-2);
  EXPECT_NEAR(expected_vehicle_state_.y(), predicted_vehicle_state_.y(), 2e-2);
  EXPECT_NEAR(expected_vehicle_state_.heading(),
              predicted_vehicle_state_.heading(), 1e-3);
  EXPECT_NEAR(expected_vehicle_state_.linear_velocity(),
              predicted_vehicle_state_.linear_velocity(), 2e-2);
}
}  // namespace common
}  // namespace apollo
