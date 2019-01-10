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

#include "modules/canbus/vehicle/transit/protocol/llc_vehiclestatus_25.h"
#include "gtest/gtest.h"

#include "modules/drivers/canbus/common/byte.h"
#include "modules/drivers/canbus/common/canbus_consts.h"

namespace apollo {
namespace canbus {
namespace transit {
using ::apollo::drivers::canbus::Byte;

class llc_vehiclestatus_25Test : public ::testing ::Test {
 public:
  virtual void SetUp() {}

 protected:
  Llcvehiclestatus25 Llcauxiliary_status25_;
};

TEST_F(llc_vehiclestatus_25Test, 12voltage) {
  const std::uint8_t kBytes = 0xFF;
  std::int32_t length = 8;
  EXPECT_DOUBLE_EQ(Llcauxiliary_status25_.llc_fbk_12voltage(&kBytes, length),
                   25.5);
}

}  // namespace transit
}  // namespace canbus
}  // namespace apollo
