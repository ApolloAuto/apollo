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

#include "modules/common/math/quaternion.h"

#include <cmath>

#include "gtest/gtest.h"

namespace apollo {
namespace common {
namespace math {

TEST(QuaternionTest, QuaternionToHeading) {
  constexpr double v = sqrt(0.5);  // = cos(pi / 4) = sin(pi / 4)
  EXPECT_DOUBLE_EQ(0,
                   QuaternionToHeading(v, 0.0, 0.0, -v));  // Pointing to east.
  EXPECT_DOUBLE_EQ(
      M_PI_2, QuaternionToHeading(1.0, 0.0, 0.0, 0.0));  // Pointing to north.
  EXPECT_DOUBLE_EQ(-M_PI_2, QuaternionToHeading(0.0, 0.0, 0.0,
                                                1.0));  // Pointing to south.
  EXPECT_DOUBLE_EQ(-M_PI,
                   QuaternionToHeading(v, 0.0, 0.0, v));  // Pointing to west.

  Eigen::Quaternionf q(1.0, 0.0, 0.0, 0.0);
  EXPECT_DOUBLE_EQ(M_PI_2, QuaternionToHeading(q));  // Pointing to north.

  const double headings[] = {-3.3, -2.2, -1.1, 1.2, 2.3, 3.4, 4.5, 5.6, 6.7};
  for (double heading : headings) {
    EXPECT_NEAR(NormalizeAngle(heading),
                QuaternionToHeading(HeadingToQuaternion<double>(heading)),
                1e-15);
  }
}

}  // namespace math
}  // namespace common
}  // namespace apollo
