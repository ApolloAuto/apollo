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

#include "modules/common/math/matrix_operations.h"

#include "gtest/gtest.h"

namespace apollo {
namespace common {
namespace math {

TEST(PseudoInverseTest, PseudoInverseI) {
  const Eigen::Matrix<float, 2, 2> A = Eigen::MatrixXf::Identity(2, 2);

  const double epsilon = 1.0e-6;

  Eigen::Matrix<float, 2, 2> B = PseudoInverse<float, 2>(A, epsilon);

  EXPECT_EQ(B(0, 0), 1);
  EXPECT_EQ(B(0, 1), 0);
  EXPECT_EQ(B(0, 0), 1);
  EXPECT_EQ(B(0, 0), 1);

  const Eigen::Matrix<float, 2, 2> C = Eigen::MatrixXf::Zero(2, 2);

  Eigen::Matrix<float, 2, 2> D = PseudoInverse<float, 2>(C, epsilon);

  EXPECT_EQ(D(0, 0), 0);
  EXPECT_EQ(D(0, 1), 0);
  EXPECT_EQ(D(0, 0), 0);
  EXPECT_EQ(D(0, 0), 0);
}

TEST(PseudoInverseTest, PseudoInverseII) {
  const Eigen::Matrix<float, 5, 1> A = Eigen::MatrixXf::Ones(5, 1);

  const double epsilon = 1.0e-6;

  Eigen::Matrix<float, 1, 5> B = PseudoInverse<float, 5, 1>(A, epsilon);

  EXPECT_EQ(B.cols(), 5);
  EXPECT_EQ(B.rows(), 1);
  EXPECT_FLOAT_EQ(B(0, 0), 0.2);
  EXPECT_FLOAT_EQ(B(0, 1), 0.2);
  EXPECT_FLOAT_EQ(B(0, 2), 0.2);
  EXPECT_FLOAT_EQ(B(0, 3), 0.2);
  EXPECT_FLOAT_EQ(B(0, 4), 0.2);

  const Eigen::Matrix<float, 5, 1> C = Eigen::MatrixXf::Zero(5, 1);

  Eigen::Matrix<float, 1, 5> D = PseudoInverse<float, 5, 1>(C, epsilon);

  EXPECT_EQ(D.cols(), 5);
  EXPECT_EQ(D.rows(), 1);
  EXPECT_FLOAT_EQ(D(0, 0), 0);
  EXPECT_FLOAT_EQ(D(0, 1), 0);
  EXPECT_FLOAT_EQ(D(0, 2), 0);
  EXPECT_FLOAT_EQ(D(0, 3), 0);
  EXPECT_FLOAT_EQ(D(0, 4), 0);
}

}  // namespace math
}  // namespace common
}  // namespace apollo
