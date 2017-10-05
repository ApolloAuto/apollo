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
  reserved_order_ = 6;
  calculate_fx(reserved_order_);
  CalculateDerivative(reserved_order_);
  CalculateSecondOrderDerivative(reserved_order_);
  CalculateThirdOrderDerivative(reserved_order_);
}

Eigen::MatrixXd SplineSegKernel::Kernel(const uint32_t order,
                                        const double accumulated_x) {
  if (order > reserved_order_) {
    calculate_fx(order);
  }
  Eigen::MatrixXd term_matrix;
  integrated_term_matrix(order, accumulated_x, "fx", &term_matrix);
  return kernel_fx_.block(0, 0, order, order).cwiseProduct(term_matrix);
}

Eigen::MatrixXd SplineSegKernel::NthDerivativeKernel(
    const uint32_t n, const uint32_t order, const double accumulated_x) {
  if (n == 1) {
    return DerivativeKernel(order, accumulated_x);
  } else if (n == 2) {
    return SecondOrderDerivativeKernel(order, accumulated_x);
  } else if (n == 3) {
    return ThirdOrderDerivativeKernel(order, accumulated_x);
  } else {
    return Eigen::MatrixXd::Zero(order, order);
  }
}

Eigen::MatrixXd SplineSegKernel::DerivativeKernel(const uint32_t order,
                                                  const double accumulated_x) {
  if (order > reserved_order_) {
    CalculateDerivative(order);
  }
  Eigen::MatrixXd term_matrix;
  integrated_term_matrix(order, accumulated_x, "derivative", &term_matrix);
  return kernel_derivative_.block(0, 0, order, order).cwiseProduct(term_matrix);
}

Eigen::MatrixXd SplineSegKernel::SecondOrderDerivativeKernel(
    const uint32_t order, const double accumulated_x) {
  if (order > reserved_order_) {
    CalculateSecondOrderDerivative(order);
  }
  Eigen::MatrixXd term_matrix;
  integrated_term_matrix(order, accumulated_x, "second_order", &term_matrix);
  return kernel_second_order_derivative_.block(0, 0, order, order)
      .cwiseProduct(term_matrix);
}

Eigen::MatrixXd SplineSegKernel::ThirdOrderDerivativeKernel(
    const uint32_t order, const double accumulated_x) {
  if (order > reserved_order_) {
    CalculateThirdOrderDerivative(order);
  }
  Eigen::MatrixXd term_matrix;
  integrated_term_matrix(order, accumulated_x, "third_order", &term_matrix);
  return (kernel_third_order_derivative_.block(0, 0, order, order))
      .cwiseProduct(term_matrix);
}

void SplineSegKernel::integrated_term_matrix(
    const uint32_t order, const double x, const std::string& type,
    Eigen::MatrixXd* term_matrix) const {
  if (term_matrix->rows() != term_matrix->cols() ||
      term_matrix->rows() != static_cast<int>(order)) {
    term_matrix->resize(order, order);
  }

  std::vector<double> x_pow(2 * order + 1, 1.0);
  for (uint32_t i = 1; i < 2 * order + 1; ++i) {
    x_pow[i] = x_pow[i - 1] * x;
  }

  if (type == "fx") {
    for (uint32_t r = 0; r < order; ++r) {
      for (uint32_t c = 0; c < order; ++c) {
        (*term_matrix)(r, c) = x_pow[r + c + 1];
      }
    }

  } else if (type == "derivative") {
    for (uint32_t r = 1; r < order; ++r) {
      for (uint32_t c = 1; c < order; ++c) {
        (*term_matrix)(r, c) = x_pow[r + c - 1];
      }
    }
    (*term_matrix).block(0, 0, order, 1) = Eigen::MatrixXd::Zero(order, 1);
    (*term_matrix).block(0, 0, 1, order) = Eigen::MatrixXd::Zero(1, order);

  } else if (type == "second_order") {
    for (uint32_t r = 2; r < order; ++r) {
      for (uint32_t c = 2; c < order; ++c) {
        (*term_matrix)(r, c) = x_pow[r + c - 3];
      }
    }
    (*term_matrix).block(0, 0, order, 2) = Eigen::MatrixXd::Zero(order, 2);
    (*term_matrix).block(0, 0, 2, order) = Eigen::MatrixXd::Zero(2, order);

  } else {
    for (uint32_t r = 3; r < order; ++r) {
      for (uint32_t c = 3; c < order; ++c) {
        (*term_matrix)(r, c) = x_pow[r + c - 5];
      }
    }
    (*term_matrix).block(0, 0, order, 3) = Eigen::MatrixXd::Zero(order, 3);
    (*term_matrix).block(0, 0, 3, order) = Eigen::MatrixXd::Zero(3, order);
  }
}

void SplineSegKernel::calculate_fx(const uint32_t order) {
  kernel_fx_ = Eigen::MatrixXd::Zero(order, order);
  for (int r = 0; r < kernel_fx_.rows(); ++r) {
    for (int c = 0; c < kernel_fx_.cols(); ++c) {
      kernel_fx_(r, c) = 1.0 / (r + c + 1.0);
    }
  }
}

void SplineSegKernel::CalculateDerivative(const uint32_t order) {
  kernel_derivative_ = Eigen::MatrixXd::Zero(order, order);
  for (int r = 1; r < kernel_derivative_.rows(); ++r) {
    for (int c = 1; c < kernel_derivative_.cols(); ++c) {
      kernel_derivative_(r, c) = r * c / (r + c - 1.0);
    }
  }
}

void SplineSegKernel::CalculateSecondOrderDerivative(
    const uint32_t order) {
  kernel_second_order_derivative_ = Eigen::MatrixXd::Zero(order, order);
  for (int r = 2; r < kernel_second_order_derivative_.rows(); ++r) {
    for (int c = 2; c < kernel_second_order_derivative_.cols(); ++c) {
      kernel_second_order_derivative_(r, c) =
          (r * r - r) * (c * c - c) / (r + c - 3.0);
    }
  }
}

void SplineSegKernel::CalculateThirdOrderDerivative(const uint32_t order) {
  kernel_third_order_derivative_ = Eigen::MatrixXd::Zero(order, order);
  for (int r = 3; r < kernel_third_order_derivative_.rows(); ++r) {
    for (int c = 3; c < kernel_third_order_derivative_.cols(); ++c) {
      kernel_third_order_derivative_(r, c) =
          (r * r - r) * (r - 2) * (c * c - c) * (c - 2) / (r + c - 5.0);
    }
  }
}

}  // namespace planning
}  // namespace apollo
