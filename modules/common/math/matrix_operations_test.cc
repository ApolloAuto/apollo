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
  EXPECT_EQ(B(1, 0), 0);
  EXPECT_EQ(B(1, 1), 1);

  const Eigen::Matrix<float, 2, 2> C = Eigen::MatrixXf::Zero(2, 2);

  Eigen::Matrix<float, 2, 2> D = PseudoInverse<float, 2>(C, epsilon);

  EXPECT_EQ(D(0, 0), 0);
  EXPECT_EQ(D(0, 1), 0);
  EXPECT_EQ(D(1, 0), 0);
  EXPECT_EQ(D(1, 1), 0);
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

TEST(ContinuousToDiscreteTest, c2d_fixed_size) {
  double ts = 0.0;

  Eigen::Matrix<float, 2, 2> m_a = Eigen::MatrixXf::Identity(2, 2);

  Eigen::Matrix<float, 2, 1> m_b = Eigen::MatrixXf::Ones(2, 1);

  Eigen::Matrix<float, 1, 2> m_c = Eigen::MatrixXf::Ones(1, 2);

  Eigen::Matrix<float, 1, 1> m_d = Eigen::MatrixXf::Identity(1, 1);

  Eigen::Matrix<float, 2, 2> prt_a_d;

  Eigen::Matrix<float, 2, 1> prt_b_d;

  Eigen::Matrix<float, 1, 2> prt_c_d;

  Eigen::Matrix<float, 1, 1> prt_d_d;

  bool res = ContinuousToDiscrete<float, 2, 1, 1>(
      m_a, m_b, m_c, m_d, ts, &prt_a_d, &prt_b_d, &prt_c_d, &prt_d_d);

  EXPECT_FALSE(res);

  ts = 1;

  res = ContinuousToDiscrete<float, 2, 1, 1>(m_a, m_b, m_c, m_d, ts, &prt_a_d,
                                             &prt_b_d, &prt_c_d, &prt_d_d);

  EXPECT_TRUE(res);

  EXPECT_FLOAT_EQ(prt_a_d(0, 0), 3);
  EXPECT_FLOAT_EQ(prt_a_d(0, 1), 0);
  EXPECT_FLOAT_EQ(prt_a_d(1, 0), 0);
  EXPECT_FLOAT_EQ(prt_a_d(1, 1), 3);

  EXPECT_FLOAT_EQ(prt_b_d(0, 0), 2);
  EXPECT_FLOAT_EQ(prt_b_d(1, 0), 2);

  EXPECT_FLOAT_EQ(prt_c_d(0, 0), 2);
  EXPECT_FLOAT_EQ(prt_c_d(0, 1), 2);

  EXPECT_FLOAT_EQ(prt_d_d(0, 0), 3);

  ts = 0.1;

  res = ContinuousToDiscrete<float, 2, 1, 1>(m_a, m_b, m_c, m_d, ts, &prt_a_d,
                                             &prt_b_d, &prt_c_d, &prt_d_d);

  EXPECT_TRUE(res);

  EXPECT_FLOAT_EQ(prt_a_d(0, 0), 1.1052631);
  EXPECT_FLOAT_EQ(prt_a_d(0, 1), 0);
  EXPECT_FLOAT_EQ(prt_a_d(1, 0), 0);
  EXPECT_FLOAT_EQ(prt_a_d(1, 1), 1.1052631);

  EXPECT_FLOAT_EQ(prt_b_d(0, 0), 0.33287135);
  EXPECT_FLOAT_EQ(prt_b_d(1, 0), 0.33287135);

  EXPECT_FLOAT_EQ(prt_c_d(0, 0), 0.33287135);
  EXPECT_FLOAT_EQ(prt_c_d(0, 1), 0.33287135);

  EXPECT_FLOAT_EQ(prt_d_d(0, 0), 2.0526316);

  ts = 0.01;

  res = ContinuousToDiscrete<float, 2, 1, 1>(m_a, m_b, m_c, m_d, ts, &prt_a_d,
                                             &prt_b_d, &prt_c_d, &prt_d_d);

  EXPECT_TRUE(res);

  EXPECT_FLOAT_EQ(prt_a_d(0, 0), 1.0100503);
  EXPECT_FLOAT_EQ(prt_a_d(0, 1), 0);
  EXPECT_FLOAT_EQ(prt_a_d(1, 0), 0);
  EXPECT_FLOAT_EQ(prt_a_d(1, 1), 1.0100503);

  EXPECT_FLOAT_EQ(prt_b_d(0, 0), 0.10050251);
  EXPECT_FLOAT_EQ(prt_b_d(1, 0), 0.10050251);

  EXPECT_FLOAT_EQ(prt_c_d(0, 0), 0.10050251);
  EXPECT_FLOAT_EQ(prt_c_d(0, 1), 0.10050251);

  EXPECT_FLOAT_EQ(prt_d_d(0, 0), 2.0050251);
}

TEST(ContinuousToDiscreteTest, c2d_dynamic_size) {
  double ts = 0.0;

  Eigen::MatrixXd m_a = Eigen::MatrixXd::Identity(2, 2);

  Eigen::MatrixXd m_b = Eigen::MatrixXd::Ones(2, 1);

  Eigen::MatrixXd m_c = Eigen::MatrixXd::Ones(1, 2);

  Eigen::MatrixXd m_d = Eigen::MatrixXd::Identity(1, 1);

  Eigen::MatrixXd prt_a_d;

  Eigen::MatrixXd prt_b_d;

  Eigen::MatrixXd prt_c_d;

  Eigen::MatrixXd prt_d_d;

  bool res = ContinuousToDiscrete(m_a, m_b, m_c, m_d, ts, &prt_a_d, &prt_b_d,
                                  &prt_c_d, &prt_d_d);

  EXPECT_FALSE(res);

  ts = 1;

  res = ContinuousToDiscrete(m_a, m_b, m_c, m_d, ts, &prt_a_d, &prt_b_d,
                             &prt_c_d, &prt_d_d);

  EXPECT_TRUE(res);

  EXPECT_FLOAT_EQ(prt_a_d(0, 0), 3);
  EXPECT_FLOAT_EQ(prt_a_d(0, 1), 0);
  EXPECT_FLOAT_EQ(prt_a_d(1, 0), 0);
  EXPECT_FLOAT_EQ(prt_a_d(1, 1), 3);

  EXPECT_FLOAT_EQ(prt_b_d(0, 0), 2);
  EXPECT_FLOAT_EQ(prt_b_d(1, 0), 2);

  EXPECT_FLOAT_EQ(prt_c_d(0, 0), 2);
  EXPECT_FLOAT_EQ(prt_c_d(0, 1), 2);

  EXPECT_FLOAT_EQ(prt_d_d(0, 0), 3);

  ts = 0.1;

  res = ContinuousToDiscrete(m_a, m_b, m_c, m_d, ts, &prt_a_d, &prt_b_d,
                             &prt_c_d, &prt_d_d);

  EXPECT_TRUE(res);

  EXPECT_FLOAT_EQ(prt_a_d(0, 0), 1.1052631);
  EXPECT_FLOAT_EQ(prt_a_d(0, 1), 0);
  EXPECT_FLOAT_EQ(prt_a_d(1, 0), 0);
  EXPECT_FLOAT_EQ(prt_a_d(1, 1), 1.1052631);

  EXPECT_FLOAT_EQ(prt_b_d(0, 0), 0.33287135);
  EXPECT_FLOAT_EQ(prt_b_d(1, 0), 0.33287135);

  EXPECT_FLOAT_EQ(prt_c_d(0, 0), 0.33287135);
  EXPECT_FLOAT_EQ(prt_c_d(0, 1), 0.33287135);

  EXPECT_FLOAT_EQ(prt_d_d(0, 0), 2.0526316);

  ts = 0.01;

  res = ContinuousToDiscrete(m_a, m_b, m_c, m_d, ts, &prt_a_d, &prt_b_d,
                             &prt_c_d, &prt_d_d);

  EXPECT_TRUE(res);

  EXPECT_FLOAT_EQ(prt_a_d(0, 0), 1.0100503);
  EXPECT_FLOAT_EQ(prt_a_d(0, 1), 0);
  EXPECT_FLOAT_EQ(prt_a_d(1, 0), 0);
  EXPECT_FLOAT_EQ(prt_a_d(1, 1), 1.0100503);

  EXPECT_FLOAT_EQ(prt_b_d(0, 0), 0.10050251);
  EXPECT_FLOAT_EQ(prt_b_d(1, 0), 0.10050251);

  EXPECT_FLOAT_EQ(prt_c_d(0, 0), 0.10050251);
  EXPECT_FLOAT_EQ(prt_c_d(0, 1), 0.10050251);

  EXPECT_FLOAT_EQ(prt_d_d(0, 0), 2.0050251);
}

}  // namespace math
}  // namespace common
}  // namespace apollo
