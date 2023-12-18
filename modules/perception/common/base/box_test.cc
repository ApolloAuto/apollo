/******************************************************************************
 * Copyright 2018 The Apollo Authors. All Rights Reserved.
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

#include "modules/perception/common/base/box.h"

#include "gtest/gtest.h"

namespace apollo {
namespace perception {
namespace base {

TEST(ImageCoreTest, operator_test) {
  {
    BBox2D<int> bbox(1, 2, 3, 4);
    Rect<int> rect = static_cast<Rect<int>>(bbox);
    EXPECT_EQ(rect.x, 1);
    EXPECT_EQ(rect.y, 2);
    EXPECT_EQ(rect.width, 2);
    EXPECT_EQ(rect.height, 2);
  }
  {
    Rect<int> rect(1, 2, 3, 4);
    BBox2D<int> bbox = static_cast<BBox2D<int>>(rect);
    EXPECT_EQ(bbox.xmin, 1);
    EXPECT_EQ(bbox.ymin, 2);
    EXPECT_EQ(bbox.xmax, 4);
    EXPECT_EQ(bbox.ymax, 6);
  }
  {
    Rect<int> rect1(1, 2, 3, 4);
    Rect<int> rect2(5, 6, 7, 8);
    Rect<int> rect3 = rect1 | rect2;
    EXPECT_EQ(rect3.x, 1);
    EXPECT_EQ(rect3.y, 2);
    EXPECT_EQ(rect3.width, 11);
    EXPECT_EQ(rect3.height, 12);
  }
  {
    Rect<int> rect1(2, 2, 4, 4);
    Rect<int> rect2(4, 4, 4, 4);
    Rect<int> rect3 = rect1 & rect2;
    EXPECT_EQ(rect3.x, 4);
    EXPECT_EQ(rect3.y, 4);
    EXPECT_EQ(rect3.width, 2);
    EXPECT_EQ(rect3.height, 2);
  }
  {
    Rect<int> rect1(1, 2, 3, 4);
    Rect<int> rect2(1, 2, 3, 4);
    EXPECT_EQ(rect1, rect2);
  }
  {
    Rect<float> rect1(100.0001f, 200.11111f, 1000.0f, 500.0f);
    Rect<float> rect2(100.0001f, 200.11111f, 1000.0f, 500.0f);
    EXPECT_EQ(rect1, rect2);
  }
  {
    Rect<float> rect1(100.002f, 200.11111f, 1000.0f, 500.0f);
    Rect<float> rect2(100.0001f, 200.11111f, 1000.0f, 500.0f);
    EXPECT_NE(rect1, rect2);
  }
  {
    Rect<double> rect1(478291.0000001, 48273912.111112323, 10000000.0,
                       20000000.0);
    Rect<double> rect2(478291.0000001, 48273912.111112323, 10000000.0,
                       20000000.0);
    EXPECT_EQ(rect1, rect2);
  }
  {
    Rect<double> rect1(478291.0000001, 48273912.111112323, 10000000.0,
                       20000000.0);
    Rect<double> rect2(478291.0000002, 48273912.111112323, 10000000.0,
                       20000000.0);
    EXPECT_NE(rect1, rect2);
  }
  {
    Rect<int> rect1(1, 2, 3, 4);
    Rect<int> rect2(2, 2, 3, 4);
    EXPECT_NE(rect1, rect2);
    EXPECT_EQ(rect1.ToStr(), "[ 3 x 4 ] from ( 1 , 2 )");
  }
}

}  // namespace base
}  // namespace perception
}  // namespace apollo
