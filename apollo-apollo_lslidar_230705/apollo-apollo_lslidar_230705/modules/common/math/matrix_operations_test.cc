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

TEST(DENSE_TO_CSC_MATRIX, dense_to_csc_matrix_test) {
  {
    std::vector<double> data;
    std::vector<int> indices;
    std::vector<int> indptr;
    Eigen::MatrixXd dense_matrix(3, 3);
    dense_matrix << 1.2, 0, 2.2, 0, 0, 3.1, 4.8, 5.4, 6.01;
    DenseToCSCMatrix(dense_matrix, &data, &indices, &indptr);

    std::vector<double> data_golden = {1.2, 4.8, 5.4, 2.2, 3.1, 6.01};
    std::vector<int> indices_golden = {0, 2, 2, 0, 1, 2};
    std::vector<int> indptr_golden = {0, 2, 3, 6};

    EXPECT_EQ(data.size(), data_golden.size());
    EXPECT_EQ(indices.size(), indices_golden.size());
    EXPECT_EQ(indptr.size(), indptr_golden.size());

    for (size_t i = 0; i < data.size(); ++i) {
      EXPECT_DOUBLE_EQ(data[i], data_golden[i]);
    }
    for (size_t i = 0; i < indices.size(); ++i) {
      EXPECT_EQ(indices[i], indices_golden[i]);
    }
    for (size_t i = 0; i < indptr.size(); ++i) {
      EXPECT_EQ(indptr[i], indptr_golden[i]);
    }
  }

  {
    std::vector<double> data;
    std::vector<int> indices;
    std::vector<int> indptr;
    Eigen::MatrixXd dense_matrix(2, 2);
    dense_matrix << 4.0, 1.0, 1.0, 2.0;
    DenseToCSCMatrix(dense_matrix, &data, &indices, &indptr);

    std::vector<double> data_golden = {4.0, 1.0, 1.0, 2.0};
    std::vector<int> indices_golden = {0, 1, 0, 1};
    std::vector<int> indptr_golden = {0, 2, 4};

    EXPECT_EQ(data.size(), data_golden.size());
    EXPECT_EQ(indices.size(), indices_golden.size());
    EXPECT_EQ(indptr.size(), indptr_golden.size());

    for (size_t i = 0; i < data.size(); ++i) {
      EXPECT_DOUBLE_EQ(data[i], data_golden[i]);
    }
    for (size_t i = 0; i < indices.size(); ++i) {
      EXPECT_EQ(indices[i], indices_golden[i]);
    }
    for (size_t i = 0; i < indptr.size(); ++i) {
      EXPECT_EQ(indptr[i], indptr_golden[i]);
    }
  }
  {
    std::vector<double> data;
    std::vector<int> indices;
    std::vector<int> indptr;
    Eigen::MatrixXd dense_matrix(4, 6);
    dense_matrix << 11, 0, 0, 14, 0, 16, 0, 22, 0, 0, 25, 26, 0, 0, 33, 34, 0,
        36, 41, 0, 43, 44, 0, 46;

    DenseToCSCMatrix(dense_matrix, &data, &indices, &indptr);

    std::vector<double> data_golden = {11.0, 41.0, 22.0, 33.0, 43.0, 14.0, 34.0,
                                       44.0, 25.0, 16.0, 26.0, 36.0, 46.0};
    std::vector<int> indices_golden = {0, 3, 1, 2, 3, 0, 2, 3, 1, 0, 1, 2, 3};
    std::vector<int> indptr_golden = {0, 2, 3, 5, 8, 9, 13};

    EXPECT_EQ(data.size(), data_golden.size());
    EXPECT_EQ(indices.size(), indices_golden.size());
    EXPECT_EQ(indptr.size(), indptr_golden.size());

    for (size_t i = 0; i < data.size(); ++i) {
      EXPECT_DOUBLE_EQ(data[i], data_golden[i]);
    }
    for (size_t i = 0; i < indices.size(); ++i) {
      EXPECT_EQ(indices[i], indices_golden[i]);
    }
    for (size_t i = 0; i < indptr.size(); ++i) {
      EXPECT_EQ(indptr[i], indptr_golden[i]) << "i = " << i;
    }
  }
}

TEST(DENSE_TO_CSC_MATRIX, patterned_dense_to_csc_matrix_test) {
  int N = 5;
  Eigen::MatrixXd dense_matrix = Eigen::MatrixXd::Zero(N * 3, N * 3);
  for (int i = 0; i < N; ++i) {
    dense_matrix(i, i) = 1.0;
    if (i + 1 < N) {
      dense_matrix(i, i + 1) = 1.0;
    }
    if (i > 0) {
      dense_matrix(i, i - 1) = 1.0;
    }
  }

  for (int i = 0; i < N; ++i) {
    dense_matrix(i + N, i + N) = 1.0;
  }

  for (int i = 0; i < N; ++i) {
    dense_matrix(i + 2 * N, i + 2 * N) = 1.0;
    if (i + 1 < N) {
      dense_matrix(i + 2 * N, i + 2 * N + 1) = 1.0;
    }
    if (i > 0) {
      dense_matrix(i + 2 * N, i + 2 * N - 1) = 1.0;
    }
  }

  for (int i = 0; i < dense_matrix.rows(); ++i) {
    for (int j = 0; j < dense_matrix.cols(); ++j) {
      std::cout << dense_matrix(i, j) << ", ";
    }
    std::cout << std::endl;
  }
  std::cout << std::endl;

  std::vector<double> data;
  std::vector<int> indices;
  std::vector<int> indptr;

  DenseToCSCMatrix(dense_matrix, &data, &indices, &indptr);
  EXPECT_EQ(data.size(), indices.size());

  for (const auto ind : indices) {
    std::cout << ind << ", ";
  }
  std::cout << std::endl;

  for (const auto ipr : indptr) {
    std::cout << ipr << ", ";
  }
  std::cout << std::endl;
}

}  // namespace math
}  // namespace common
}  // namespace apollo
