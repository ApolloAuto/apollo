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

#include "modules/planning/math/finite_element_qp/fem_1d_expanded_qp_problem.h"

#include <chrono>

#include "cyber/common/log.h"
#include "gtest/gtest.h"

namespace apollo {
namespace planning {

TEST(Fem1dLinearQpProblemTest, basic_test) {
  Fem1dQpProblem* fem_qp = new Fem1dExpandedQpProblem();
  std::array<double, 3> x_init = {1.5, 0.0, 0.001};
  double delta_s = 1.0;
  size_t n = 200;
  std::vector<std::tuple<double, double, double>> x_bounds;
  for (size_t i = 0; i < n; ++i) {
    x_bounds.emplace_back(std::make_tuple(static_cast<double>(i), -1.81, 1.95));
  }
  std::array<double, 5> w = {1.0, 2.0, 3.0, 4.0, 1.45};
  double max_x_third_order_derivative = 0.25;
  EXPECT_TRUE(
      fem_qp->Init(n, x_init, delta_s, w, max_x_third_order_derivative));

  fem_qp->SetVariableBounds(x_bounds);

  auto start_time = std::chrono::system_clock::now();
  EXPECT_TRUE(fem_qp->Optimize());
  auto end_time = std::chrono::system_clock::now();
  std::chrono::duration<double> diff = end_time - start_time;
  AINFO << "qp_optimizer used time: " << diff.count() * 1000 << " ms.";

  const std::vector<double> x = fem_qp->x();
  for (size_t i = 0; i < x.size(); ++i) {
    EXPECT_LE(x[i], std::get<2>(x_bounds[i]));
    EXPECT_GE(x[i], std::get<1>(x_bounds[i]));
  }
}

}  // namespace planning
}  // namespace apollo
