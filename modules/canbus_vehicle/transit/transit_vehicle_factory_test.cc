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

#include "modules/canbus_vehicle/transit/transit_vehicle_factory.h"

#include "gtest/gtest.h"

#include "modules/canbus/proto/canbus_conf.pb.h"
#include "modules/canbus/proto/vehicle_parameter.pb.h"

#include "cyber/common/file.h"

namespace apollo {
namespace canbus {

class TransitVehicleFactoryTest : public ::testing::Test {
 public:
  virtual void SetUp() {
    std::string canbus_conf_file =
        "/apollo/modules/canbus_vehicle/transit/testdata/"
        "transit_canbus_conf_test.pb.txt";
    cyber::common::GetProtoFromFile(canbus_conf_file, &canbus_conf_);
    params_ = canbus_conf_.vehicle_parameter();
    params_.set_brand(apollo::common::TRANSIT);
    transit_factory_.SetVehicleParameter(params_);
  }
  virtual void TearDown() {}

 protected:
  TransitVehicleFactory transit_factory_;
  CanbusConf canbus_conf_;
  VehicleParameter params_;
};

TEST_F(TransitVehicleFactoryTest, Init) {
  apollo::cyber::Init("vehicle_factory_test");
  EXPECT_EQ(transit_factory_.Init(&canbus_conf_), true);
}

}  // namespace canbus
}  // namespace apollo
