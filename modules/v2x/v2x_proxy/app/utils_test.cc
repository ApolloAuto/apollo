/******************************************************************************
 * Copyright 2020 The Apollo Authors. All Rights Reserved.
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
 * @file utils_test.cc
 */

#include "modules/v2x/v2x_proxy/app/utils.h"

#include "gtest/gtest.h"

namespace apollo {
namespace v2x {

TEST(v2x_proxy_app, v2x_proxy_UtilsGetNextColor) {
  EXPECT_EQ(OSLightColor::SingleTrafficLight_Color_YELLOW,
            utils::GetNextColor(OSLightColor::SingleTrafficLight_Color_GREEN));
  EXPECT_EQ(OSLightColor::SingleTrafficLight_Color_RED,
            utils::GetNextColor(OSLightColor::SingleTrafficLight_Color_YELLOW));
  EXPECT_EQ(OSLightColor::SingleTrafficLight_Color_GREEN,
            utils::GetNextColor(OSLightColor::SingleTrafficLight_Color_RED));
  EXPECT_EQ(OSLightColor::SingleTrafficLight_Color_BLACK,
            utils::GetNextColor(OSLightColor::SingleTrafficLight_Color_BLACK));
}

}  // namespace v2x
}  // namespace apollo
