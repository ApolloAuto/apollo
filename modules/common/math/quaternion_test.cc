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
  const double v = sqrt(0.5);  // = cos(pi / 4) = sin(pi / 4)
  EXPECT_DOUBLE_EQ(0,
                   QuaternionToHeading(v, 0.0, 0.0, -v));  // Pointing to east.
  EXPECT_DOUBLE_EQ(
      M_PI_2, QuaternionToHeading(1.0, 0.0, 0.0, 0.0));  // Pointing to north.
  EXPECT_DOUBLE_EQ(-M_PI_2, QuaternionToHeading(0.0, 0.0, 0.0,
                                                1.0));  // Pointing to south.
  EXPECT_DOUBLE_EQ(-M_PI,
                   QuaternionToHeading(v, 0.0, 0.0, v));  // Pointing to west.

  Eigen::Quaternionf q(1.0, 0.0, 0.0, 0.0);
  EXPECT_FLOAT_EQ(M_PI_2, QuaternionToHeading(q));  // Pointing to north.

  const double headings[] = {-3.3, -2.2, -1.1, 1.2, 2.3, 3.4, 4.5, 5.6, 6.7};
  for (double heading : headings) {
    EXPECT_NEAR(NormalizeAngle(heading),
                QuaternionToHeading(HeadingToQuaternion<double>(heading)),
                1e-15);
  }
}

TEST(QuaternionTest, QuaternionRotate) {
  apollo::common::Quaternion q;
  q.set_qx(0.016590540978116377);
  q.set_qy(0.012968083311103572);
  q.set_qz(-0.99256254167039326);
  q.set_qw(-0.1199007240933047);

  Eigen::Vector3d original(0.18112868882363914, 0.38614886414425986,
                           -0.15861744649897938);

  auto rotated = QuaternionRotate(q, original);
  EXPECT_NEAR(rotated[0], -0.26184808017295008, 1e-9);
  EXPECT_NEAR(rotated[1], -0.32827419468368224, 1e-9);
  EXPECT_NEAR(rotated[2], -0.17535585973456849, 1e-9);
}

TEST(QuaternionTest, InverseQuaternionRotate) {
  apollo::common::Quaternion q;
  q.set_qx(0.016590540978116377);
  q.set_qy(0.012968083311103572);
  q.set_qz(-0.99256254167039326);
  q.set_qw(-0.1199007240933047);
  Eigen::Vector3d rotated(-0.26184808017295008, -0.32827419468368224,
                          -0.17535585973456849);
  auto original = InverseQuaternionRotate(q, rotated);
  EXPECT_NEAR(original[0], 0.18112868882363914, 1e-9);
  EXPECT_NEAR(original[1], 0.38614886414425986, 1e-9);
  EXPECT_NEAR(original[2], -0.15861744649897938, 1e-9);
}

}  // namespace math
}  // namespace common
}  // namespace apollo
