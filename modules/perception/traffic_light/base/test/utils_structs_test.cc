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
#include <gtest/gtest.h>

#include "modules/perception/traffic_light/base/utils.h"

namespace apollo {
namespace perception {
namespace traffic_light {

TEST(UtilsTest, test_refined_box) {
  cv::Size size(200, 200);
  cv::Rect rect1(-1, -1, 200, 200);
  auto box1 = RefinedBox(rect1, size);
  EXPECT_EQ(0, box1.x);
  EXPECT_EQ(0, box1.y);
  EXPECT_EQ(199, box1.width);
  EXPECT_EQ(199, box1.height);

  cv::Rect rect2(-300, -300, -10, -10);
  auto box2 = RefinedBox(rect2, size);
  EXPECT_EQ(0, box2.x);
  EXPECT_EQ(0, box2.y);
  EXPECT_EQ(0, box2.width);
  EXPECT_EQ(0, box2.height);

  cv::Rect rect3(10, 10, 300, 300);
  auto box3 = RefinedBox(rect3, size);
  EXPECT_EQ(10, box3.x);
  EXPECT_EQ(10, box3.y);
  EXPECT_EQ(189, box3.width);
  EXPECT_EQ(189, box3.height);
}

}
}
}
