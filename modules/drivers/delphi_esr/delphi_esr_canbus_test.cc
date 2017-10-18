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

#include "modules/drivers/delphi_esr/delphi_esr_canbus.h"
#include "modules/drivers/delphi_esr/delphi_esr_message_manager.h"
#include "modules/drivers/proto/delphi_esr.pb.h"

#include "gtest/gtest.h"

namespace apollo {
namespace drivers {

using apollo::common::ErrorCode;

TEST(DelphiESRCanbusTest, Simple) {
  SensorCanbus<DelphiESR> cb;
  EXPECT_EQ(cb.Name(), "canbus");
}

}  // namespace drivers
}  // namespace apollo
