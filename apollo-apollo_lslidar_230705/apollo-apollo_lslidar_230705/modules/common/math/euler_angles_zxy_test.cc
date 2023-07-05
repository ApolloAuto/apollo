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

#include "modules/common/math/euler_angles_zxy.h"

#include <cmath>

#include "Eigen/Geometry"
#include "gtest/gtest.h"

namespace apollo {
namespace common {
namespace math {

namespace {

Eigen::Quaterniond GoldenEulerZXYToQuaternion(const double roll,
                                              const double pitch,
                                              const double yaw) {
  return Eigen::AngleAxisd(yaw, Eigen::Vector3d::UnitZ()) *
         Eigen::AngleAxisd(pitch, Eigen::Vector3d::UnitX()) *
         Eigen::AngleAxisd(roll, Eigen::Vector3d::UnitY());
}

}  // namespace

TEST(EulerAnglesZXYTest, SingleConstruct) {
  EulerAnglesZXYd a(1.0);
  EXPECT_DOUBLE_EQ(0.0, a.roll());
  EXPECT_DOUBLE_EQ(0.0, a.pitch());
  EXPECT_DOUBLE_EQ(1.0, a.yaw());

  auto q = a.ToQuaternion();
  EXPECT_DOUBLE_EQ(cos(0.5), q.w());
  EXPECT_DOUBLE_EQ(0.0, q.x());
  EXPECT_DOUBLE_EQ(0.0, q.y());
  EXPECT_DOUBLE_EQ(sin(0.5), q.z());
}

TEST(EulerAnglesZXYTest, FullConstructDouble) {
  EulerAnglesZXYd a(0.35, 0.24, -1.0);
  auto q_golden = GoldenEulerZXYToQuaternion(0.35, 0.24, -1.0);
  auto q = a.ToQuaternion();
  EXPECT_DOUBLE_EQ(q_golden.w(), q.w());
  EXPECT_DOUBLE_EQ(q_golden.x(), q.x());
  EXPECT_DOUBLE_EQ(q_golden.y(), q.y());
  EXPECT_DOUBLE_EQ(q_golden.z(), q.z());

  EulerAnglesZXYd b(q);
  EXPECT_DOUBLE_EQ(0.35, b.roll());
  EXPECT_DOUBLE_EQ(0.24, b.pitch());
  EXPECT_DOUBLE_EQ(-1.0, b.yaw());
}

TEST(EulerAnglesZXYTest, FullConstructFloat) {
  Eigen::Quaternionf q(1.0f, 2.0f, -3.0f, 4.0f);
  q.normalize();
  EulerAnglesZXYf a(q);
  auto q2 = a.ToQuaternion();
  EXPECT_NEAR(q.w(), q2.w(), 5e-7);
  EXPECT_NEAR(q.x(), q2.x(), 5e-7);
  EXPECT_NEAR(q.y(), q2.y(), 5e-7);
  EXPECT_NEAR(q.z(), q2.z(), 5e-7);
}

TEST(EulerAnglesZXYTest, IsValid) {
  EulerAnglesZXYd a(0.35, 5.0, -1.0);
  EXPECT_TRUE(a.IsValid());

  EulerAnglesZXYd b(0.35, 2.0, -1.0);
  EXPECT_FALSE(b.IsValid());
}

}  // namespace math
}  // namespace common
}  // namespace apollo
