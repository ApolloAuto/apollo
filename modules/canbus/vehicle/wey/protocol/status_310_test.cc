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

#include "modules/canbus/vehicle/wey/protocol/status_310.h"
#include "gtest/gtest.h"

namespace apollo {
namespace canbus {
namespace wey {

class Status310Test : public ::testing::Test {
 public:
  virtual void SetUp() {}
};

TEST_F(Status310Test, reset) {
  Status310 status;
  int32_t length = 8;
  ChassisDetail chassis_detail;
  uint8_t bytes[8] = {0x88, 0x44, 0x22, 0x11, 0x11, 0x12, 0x13, 0x14};

  status.Parse(bytes, length, &chassis_detail);
  EXPECT_DOUBLE_EQ(chassis_detail.wey().status_310().longitudeaccvalid(), 0);
  EXPECT_DOUBLE_EQ(chassis_detail.wey().status_310().lateralaccevalid(), 1);
  EXPECT_DOUBLE_EQ(chassis_detail.wey().status_310().vehdynyawratevalid(), 0);
  EXPECT_DOUBLE_EQ(chassis_detail.wey().status_310().flwheelspdvalid(), 0);
  EXPECT_DOUBLE_EQ(chassis_detail.wey().status_310().frwheelspdvalid(), 0);
  EXPECT_DOUBLE_EQ(chassis_detail.wey().status_310().rlwheelspdvalid(), 1);
  EXPECT_DOUBLE_EQ(chassis_detail.wey().status_310().rrwheelspdvalid(), 0);
  EXPECT_DOUBLE_EQ(chassis_detail.wey().status_310().vehiclespdvalid(), 0);
  EXPECT_DOUBLE_EQ(chassis_detail.wey().status_310().longitudedrivingmode(), 2);
  EXPECT_DOUBLE_EQ(chassis_detail.wey().status_310().engspdvalid(), 0);
  EXPECT_DOUBLE_EQ(chassis_detail.wey().status_310().accepedaloverride(), 0);
  EXPECT_DOUBLE_EQ(chassis_detail.wey().status_310().brakepedalstatus(), 0);
  EXPECT_DOUBLE_EQ(chassis_detail.wey().status_310().espbrakelightsts(), 0);
  EXPECT_DOUBLE_EQ(chassis_detail.wey().status_310().epbswtpositionvalid(), 0);
  EXPECT_DOUBLE_EQ(chassis_detail.wey().status_310().epbsts(), 1);
  EXPECT_DOUBLE_EQ(chassis_detail.wey().status_310().currentgearvalid(), 0);
  EXPECT_DOUBLE_EQ(chassis_detail.wey().status_310().epstrqsnsrsts(), 0);
  EXPECT_DOUBLE_EQ(chassis_detail.wey().status_310().eps_interferdetdvalid(),
                   0);
  EXPECT_DOUBLE_EQ(chassis_detail.wey().status_310().epshandsdetnsts(), 0);
  EXPECT_DOUBLE_EQ(chassis_detail.wey().status_310().eps_handsdetnstsvalid(),
                   0);
  EXPECT_DOUBLE_EQ(chassis_detail.wey().status_310().steerwheelanglesign(), 1);
  EXPECT_DOUBLE_EQ(chassis_detail.wey().status_310().steerwheelspdsign(), 0);
  EXPECT_DOUBLE_EQ(chassis_detail.wey().status_310().driverdoorsts(), 0);
  EXPECT_DOUBLE_EQ(chassis_detail.wey().status_310().rldoorsts(), 0);
  EXPECT_DOUBLE_EQ(chassis_detail.wey().status_310().rrdoorsts(), 1);
  EXPECT_DOUBLE_EQ(chassis_detail.wey().status_310().frontfoglmpsts(), 0);
  EXPECT_DOUBLE_EQ(chassis_detail.wey().status_310().rearfoglmpsts(), 0);
  EXPECT_DOUBLE_EQ(chassis_detail.wey().status_310().lowbeamsts(), 1);
  EXPECT_DOUBLE_EQ(chassis_detail.wey().status_310().highbeamsts(), 0);
  EXPECT_DOUBLE_EQ(chassis_detail.wey().status_310().leftturnlampsts(), 0);
  EXPECT_DOUBLE_EQ(chassis_detail.wey().status_310().rightturnlampsts(), 1);
  EXPECT_DOUBLE_EQ(chassis_detail.wey().status_310().bcm_availsts(), 2);
  EXPECT_DOUBLE_EQ(chassis_detail.wey().status_310().brakelmpsts(), 0);
}

}  // namespace wey
}  // namespace canbus
}  // namespace apollo
