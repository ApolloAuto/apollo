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

#include "modules/perception/common/graph/secure_matrix.h"

#include "Eigen/Core"
#include "gtest/gtest.h"

namespace apollo {
namespace perception {
namespace common {

TEST(SecureMatTest, test_reserve_mat) {
  SecureMat<float> test_mat;
  test_mat.Reserve(100, 100);
  test_mat.Resize(10, 20);
  EXPECT_EQ(10, test_mat.height());
  EXPECT_EQ(20, test_mat.width());

  test_mat.Reserve(1200, 1200);
  EXPECT_EQ(10, test_mat.height());
  EXPECT_EQ(20, test_mat.width());
}

TEST(SecureMatTest, test_resize_mat) {
  SecureMat<float> test_mat;
  test_mat.Reserve(100, 100);
  test_mat.Resize(3, 2);
  EXPECT_EQ(3, test_mat.height());
  EXPECT_EQ(2, test_mat.width());

  test_mat.Resize(500, 1500);
  EXPECT_EQ(500, test_mat.height());
  EXPECT_EQ(1500, test_mat.width());

  test_mat.Resize(2000, 1500);
  EXPECT_EQ(2000, test_mat.height());
  EXPECT_EQ(1500, test_mat.width());
}

TEST(SecureMatTest, test_fill_mat) {
  SecureMat<float> test_mat;
  test_mat.Reserve(10, 10);
  test_mat.Resize(2, 2);
  test_mat(0, 0) = 1;
  test_mat(0, 1) = 2;
  test_mat(1, 0) = 3;
  test_mat(1, 1) = 4;
  test_mat.ToString(&std::cout);
  EXPECT_EQ(1, test_mat(0, 0));
  EXPECT_EQ(2, test_mat(0, 1));
  EXPECT_EQ(3, test_mat(1, 0));
  EXPECT_EQ(4, test_mat(1, 1));
}

}  // namespace common
}  // namespace perception
}  // namespace apollo
