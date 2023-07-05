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

/**
 * @file : spline_seg_kernel.cc
 * @brief: generating spline kernels
 **/

#include "modules/planning/math/smoothing_spline/spline_seg_kernel.h"

#include <vector>

namespace apollo {
namespace planning {

SplineSegKernel::SplineSegKernel() {
  const int reserved_num_params = reserved_order_ + 1;
  CalculateFx(reserved_num_params);
  CalculateDerivative(reserved_num_params);
  CalculateSecondOrderDerivative(reserved_num_params);
  CalculateThirdOrderDerivative(reserved_num_params);
}

Eigen::MatrixXd SplineSegKernel::Kernel(const uint32_t num_params,
                                        const double accumulated_x) {
  if (num_params > reserved_order_ + 1) {
    CalculateFx(num_params);
  }
  Eigen::MatrixXd term_matrix;
  IntegratedTermMatrix(num_params, accumulated_x, "fx", &term_matrix);
  return kernel_fx_.block(0, 0, num_params, num_params)
      .cwiseProduct(term_matrix);
}

Eigen::MatrixXd SplineSegKernel::NthDerivativeKernel(
    const uint32_t n, const uint32_t num_params, const double accumulated_x) {
  if (n == 1) {
    return DerivativeKernel(num_params, accumulated_x);
  } else if (n == 2) {
    return SecondOrderDerivativeKernel(num_params, accumulated_x);
  } else if (n == 3) {
    return ThirdOrderDerivativeKernel(num_params, accumulated_x);
  } else {
    return Eigen::MatrixXd::Zero(num_params, num_params);
  }
}

Eigen::MatrixXd SplineSegKernel::DerivativeKernel(const uint32_t num_params,
                                                  const double accumulated_x) {
  if (num_params > reserved_order_ + 1) {
    CalculateDerivative(num_params);
  }
  Eigen::MatrixXd term_matrix;
  IntegratedTermMatrix(num_params, accumulated_x, "derivative", &term_matrix);
  return kernel_derivative_.block(0, 0, num_params, num_params)
      .cwiseProduct(term_matrix);
}

Eigen::MatrixXd SplineSegKernel::SecondOrderDerivativeKernel(
    const uint32_t num_params, const double accumulated_x) {
  if (num_params > reserved_order_ + 1) {
    CalculateSecondOrderDerivative(num_params);
  }
  Eigen::MatrixXd term_matrix;
  IntegratedTermMatrix(num_params, accumulated_x, "second_order", &term_matrix);
  return kernel_second_order_derivative_.block(0, 0, num_params, num_params)
      .cwiseProduct(term_matrix);
}

Eigen::MatrixXd SplineSegKernel::ThirdOrderDerivativeKernel(
    const uint32_t num_params, const double accumulated_x) {
  if (num_params > reserved_order_ + 1) {
    CalculateThirdOrderDerivative(num_params);
  }
  Eigen::MatrixXd term_matrix;
  IntegratedTermMatrix(num_params, accumulated_x, "third_order", &term_matrix);
  return (kernel_third_order_derivative_.block(0, 0, num_params, num_params))
      .cwiseProduct(term_matrix);
}

void SplineSegKernel::IntegratedTermMatrix(const uint32_t num_params,
                                           const double x,
                                           const std::string& type,
                                           Eigen::MatrixXd* term_matrix) const {
  if (term_matrix->rows() != term_matrix->cols() ||
      term_matrix->rows() != static_cast<int>(num_params)) {
    term_matrix->resize(num_params, num_params);
  }

  std::vector<double> x_pow(2 * num_params + 1, 1.0);
  for (uint32_t i = 1; i < 2 * num_params + 1; ++i) {
    x_pow[i] = x_pow[i - 1] * x;
  }

  if (type == "fx") {
    for (uint32_t r = 0; r < num_params; ++r) {
      for (uint32_t c = 0; c < num_params; ++c) {
        (*term_matrix)(r, c) = x_pow[r + c + 1];
      }
    }

  } else if (type == "derivative") {
    for (uint32_t r = 1; r < num_params; ++r) {
      for (uint32_t c = 1; c < num_params; ++c) {
        (*term_matrix)(r, c) = x_pow[r + c - 1];
      }
    }
    (*term_matrix).block(0, 0, num_params, 1) =
        Eigen::MatrixXd::Zero(num_params, 1);
    (*term_matrix).block(0, 0, 1, num_params) =
        Eigen::MatrixXd::Zero(1, num_params);

  } else if (type == "second_order") {
    for (uint32_t r = 2; r < num_params; ++r) {
      for (uint32_t c = 2; c < num_params; ++c) {
        (*term_matrix)(r, c) = x_pow[r + c - 3];
      }
    }
    (*term_matrix).block(0, 0, num_params, 2) =
        Eigen::MatrixXd::Zero(num_params, 2);
    (*term_matrix).block(0, 0, 2, num_params) =
        Eigen::MatrixXd::Zero(2, num_params);

  } else {
    for (uint32_t r = 3; r < num_params; ++r) {
      for (uint32_t c = 3; c < num_params; ++c) {
        (*term_matrix)(r, c) = x_pow[r + c - 5];
      }
    }
    (*term_matrix).block(0, 0, num_params, 3) =
        Eigen::MatrixXd::Zero(num_params, 3);
    (*term_matrix).block(0, 0, 3, num_params) =
        Eigen::MatrixXd::Zero(3, num_params);
  }
}

void SplineSegKernel::CalculateFx(const uint32_t num_params) {
  kernel_fx_ = Eigen::MatrixXd::Zero(num_params, num_params);
  for (int r = 0; r < kernel_fx_.rows(); ++r) {
    for (int c = 0; c < kernel_fx_.cols(); ++c) {
      kernel_fx_(r, c) = 1.0 / (r + c + 1.0);
    }
  }
}

void SplineSegKernel::CalculateDerivative(const uint32_t num_params) {
  kernel_derivative_ = Eigen::MatrixXd::Zero(num_params, num_params);
  for (int r = 1; r < kernel_derivative_.rows(); ++r) {
    for (int c = 1; c < kernel_derivative_.cols(); ++c) {
      kernel_derivative_(r, c) = r * c / (r + c - 1.0);
    }
  }
}

void SplineSegKernel::CalculateSecondOrderDerivative(
    const uint32_t num_params) {
  kernel_second_order_derivative_ =
      Eigen::MatrixXd::Zero(num_params, num_params);
  for (int r = 2; r < kernel_second_order_derivative_.rows(); ++r) {
    for (int c = 2; c < kernel_second_order_derivative_.cols(); ++c) {
      kernel_second_order_derivative_(r, c) =
          (r * r - r) * (c * c - c) / (r + c - 3.0);
    }
  }
}

void SplineSegKernel::CalculateThirdOrderDerivative(const uint32_t num_params) {
  kernel_third_order_derivative_ =
      Eigen::MatrixXd::Zero(num_params, num_params);
  for (int r = 3; r < kernel_third_order_derivative_.rows(); ++r) {
    for (int c = 3; c < kernel_third_order_derivative_.cols(); ++c) {
      kernel_third_order_derivative_(r, c) =
          (r * r - r) * (r - 2) * (c * c - c) * (c - 2) / (r + c - 5.0);
    }
  }
}

}  // namespace planning
}  // namespace apollo
