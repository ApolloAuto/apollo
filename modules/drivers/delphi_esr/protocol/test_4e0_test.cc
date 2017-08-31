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

#include "modules/drivers/delphi_esr/protocol/test_4e0.h"

#include "gtest/gtest.h"

namespace apollo {
namespace drivers {
namespace delphi_esr {

class Test4e0Test : public ::testing::Test {
 public:
  virtual void SetUp() {}
};

TEST_F(Test4e0Test, Parse) {
  Test4e0 test_4e0;
  int32_t length = 8;
  DelphiESR delphi_esr;
  uint8_t bytes[8];

  bytes[0] = 0b00000010;
  bytes[1] = 0b00000101;
  bytes[2] = 0b00000101;
  bytes[3] = 0b00000001;
  bytes[4] = 0b00000101;
  bytes[5] = 0b00000011;

  test_4e0.Parse(bytes, length, &delphi_esr);
  // TODO(lizh): add more checks
  EXPECT_EQ(261, delphi_esr.test_4e0().scan_index());
}

}  // namespace delphi_esr 
}  // namespace drivers
}  // namespace apollo
