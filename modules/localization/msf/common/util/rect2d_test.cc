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

#include "modules/localization/msf/common/util/rect2d.h"

#include "gtest/gtest.h"

namespace apollo {
namespace localization {
namespace msf {

TEST(Rect2DTestSuite, Rect2DTest) {
  Rect2D<double> rect_a(0.5, 0.5, 1.0, 1.0);
  double min_x = rect_a.GetMinX();
  double min_y = rect_a.GetMinY();
  double max_x = rect_a.GetMaxX();
  double max_y = rect_a.GetMaxY();
  EXPECT_LT(std::abs(min_x - 0.5), 1e-5);
  EXPECT_LT(std::abs(min_y - 0.5), 1e-5);
  EXPECT_LT(std::abs(max_x - 1.0), 1e-5);
  EXPECT_LT(std::abs(max_y - 1.0), 1e-5);

  Eigen::Matrix<double, 2, 1> lt_corner = rect_a.GetLeftTopCorner();
  Eigen::Matrix<double, 2, 1> rb_corner = rect_a.GetRightBottomCorner();
  EXPECT_LT(std::abs(lt_corner(0) - 0.5), 1e-5);
  EXPECT_LT(std::abs(lt_corner(1) - 0.5), 1e-5);
  EXPECT_LT(std::abs(rb_corner(0) - 1.0), 1e-5);
  EXPECT_LT(std::abs(rb_corner(1) - 1.0), 1e-5);

  Rect2D<double> rect_b(rect_a);
  EXPECT_LT(std::abs(rect_b.GetMinX() - 0.5), 1e-5);

  Rect2D<double> rect_c = rect_b;
  EXPECT_LT(std::abs(rect_c.GetMinX() - 0.5), 1e-5);
}

}  // namespace msf
}  // namespace localization
}  // namespace apollo
