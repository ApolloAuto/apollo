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
 * @file
 **/

#include <chrono>

#include "cyber/common/log.h"
#include "gtest/gtest.h"

#include "modules/planning/common/planning_gflags.h"

#define private public
#define protected public
#include "modules/planning/math/finite_element_qp/fem_1d_linear_qp_problem.h"

#include "modules/planning/math/finite_element_qp/fem_1d_qp_problem.h"

namespace apollo {
namespace planning {

TEST(Fem1dLinearQpProblemTest, basic_test) {
  FLAGS_enable_osqp_debug = false;
  Fem1dQpProblem* fem_qp = new Fem1dLinearQpProblem();
  std::array<double, 3> x_init = {1.5, 0.0, 0.001};
  size_t n = 400;
  double delta_s = 0.5;
  std::vector<std::tuple<double, double, double>> x_bounds;
  for (size_t i = 0; i < n; ++i) {
    x_bounds.emplace_back(std::make_tuple(static_cast<double>(i), -1.81, 1.95));
  }
  std::array<double, 5> w = {1.0, 2.0, 3.0, 4.0, 1.45};
  double max_x_third_order_derivative = 0.25;
  EXPECT_TRUE(
      fem_qp->Init(n, x_init, delta_s, w, max_x_third_order_derivative));

  fem_qp->SetVariableBounds(x_bounds);

  for (int i = 0; i < 3; ++i) {
    auto start_time = std::chrono::system_clock::now();
    EXPECT_TRUE(fem_qp->Optimize());
    auto end_time = std::chrono::system_clock::now();
    std::chrono::duration<double> diff = end_time - start_time;
    AINFO << "qp_optimizer used time: " << diff.count() * 1000 << " ms.\n\n";
  }

  const std::vector<double> x = fem_qp->x();
  for (size_t i = 1; i < x.size(); ++i) {
    EXPECT_LE(x[i - 1] - 1e-5, fem_qp->x_bounds_[i - 1].second);
    EXPECT_GE(x[i - 1] + 1e-5, fem_qp->x_bounds_[i - 1].first);
  }
}

TEST(Fem1dLinearQpProblemTest, second_order_derivative_constraint_test) {
  FLAGS_enable_osqp_debug = true;
  Fem1dQpProblem* fem_qp = new Fem1dLinearQpProblem();
  std::array<double, 3> x_init = {4.5, 0.00, 0.0};
  double delta_s = 1.0;
  size_t n = 200;
  std::vector<std::tuple<double, double, double>> x_bounds;
  for (size_t i = 0; i < n; ++i) {
    x_bounds.emplace_back(std::make_tuple(static_cast<double>(i), -6.0, 6.0));
  }
  std::array<double, 5> w = {1.0, 100.0, 1000.0, 1000.0, 0.0};
  double max_x_third_order_derivative = 2.0;
  EXPECT_TRUE(
      fem_qp->Init(n, x_init, delta_s, w, max_x_third_order_derivative));

  fem_qp->SetVariableBounds(x_bounds);

  const double ddx_max = std::sqrt(0.5) / 15.0;
  std::vector<std::tuple<double, double, double>> ddx_bounds;
  for (size_t i = 0; i < 20; ++i) {
    ddx_bounds.emplace_back(
        std::make_tuple(static_cast<double>(i), -ddx_max, ddx_max));
  }
  fem_qp->SetVariableDerivativeBounds(ddx_bounds);

  auto start_time = std::chrono::system_clock::now();
  EXPECT_TRUE(fem_qp->Optimize());
  auto end_time = std::chrono::system_clock::now();
  std::chrono::duration<double> diff = end_time - start_time;
  AINFO << "qp_optimizer used time: " << diff.count() * 1000 << " ms.";

  const std::vector<double>& x = fem_qp->x();
  AINFO << "x.size() = " << x.size();
  AINFO << "x_bounds.size() = " << fem_qp->x_bounds_.size();
  for (size_t i = 1; i + 1 < x.size(); ++i) {
    EXPECT_LE(x[i - 1], fem_qp->x_bounds_[i].second);
    EXPECT_GE(x[i - 1], fem_qp->x_bounds_[i].first);
  }

  const std::vector<double>& ddx = fem_qp->x_second_order_derivative();
  AINFO << "ddx.size() = " << ddx.size();
  AINFO << "ddx_bounds.size() = " << fem_qp->ddx_bounds_.size();
  for (size_t i = 1; i + 1 < ddx.size(); ++i) {
    EXPECT_LE(ddx[i - 1], fem_qp->ddx_bounds_[i].second);
    EXPECT_GE(ddx[i - 1], fem_qp->ddx_bounds_[i].first);
  }
}

}  // namespace planning
}  // namespace apollo
