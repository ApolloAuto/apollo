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
#include "modules/perception/traffic_light/base/light.h"

#include <sstream>

#include "gtest/gtest.h"

namespace apollo {
namespace perception {
namespace traffic_light {

TEST(LightTest, test_light) {
  LightStatus light_status;
  {
    std::ostringstream oss;
    oss << "Status: [color:"
        << "unknown color"
        << " confidence:" << 0.0 << "]";
    EXPECT_EQ(oss.str(), light_status.to_string());
  }

  light_status.color = RED;
  {
    std::ostringstream oss;
    oss << "Status: [color:"
        << "red"
        << " confidence:" << 0.0 << "]";
    EXPECT_EQ(oss.str(), light_status.to_string());
  }

  light_status.color = GREEN;
  {
    std::ostringstream oss;
    oss << "Status: [color:"
        << "green"
        << " confidence:" << 0.0 << "]";
    EXPECT_EQ(oss.str(), light_status.to_string());
  }

  light_status.color = YELLOW;
  {
    std::ostringstream oss;
    oss << "Status: [color:"
        << "yellow"
        << " confidence:" << 0.0 << "]";
    EXPECT_EQ(oss.str(), light_status.to_string());
  }

  light_status.color = BLACK;
  {
    std::ostringstream oss;
    oss << "Status: [color:"
        << "black"
        << " confidence:" << 0.0 << "]";
    EXPECT_EQ(oss.str(), light_status.to_string());
  }
}

}  // namespace traffic_light
}  // namespace perception
}  // namespace apollo
