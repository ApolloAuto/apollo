/******************************************************************************
 * Copyright 2019 The Apollo Authors. All Rights Reserved.
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

#include <math.h>
#include <chrono>

#include "cyber/common/log.h"
#include "gtest/gtest.h"

#include "modules/planning/common/planning_gflags.h"

#define private public
#define protected public
#include "modules/planning/math/finite_element_optimizer/finite_element1d_optimizer.h"

namespace apollo {
namespace planning {

TEST(FiniteElement1dOptimizerTest, test) {
  FLAGS_enable_osqp_debug = true;
  std::array<double, 3> x_init = {1.5, 0.01, 0.001};

  size_t n = 400;
  std::vector<std::pair<double, double>> x_bounds;
  for (size_t i = 0; i < n; ++i) {
    if (i != 2) {
      x_bounds.emplace_back(-1.81, 1.95);
    } else {
      x_bounds.emplace_back(0.81, 1.95);
    }
  }

  double delta_s = 1.0;
  FiniteElement1dOptimizer* fem_optimizer =
      new FiniteElement1dOptimizer(n, x_init, delta_s);

  fem_optimizer->SetFirstOrderBounds(x_bounds);
  fem_optimizer->SetFirstOrderBounds(FLAGS_lateral_derivative_bound_default);
  fem_optimizer->SetSecondOrderBounds(FLAGS_lateral_derivative_bound_default);
  fem_optimizer->SetThirdOrderBound(1.25);

  auto start_time = std::chrono::system_clock::now();

  std::vector<double> x;
  std::vector<double> dx;
  std::vector<double> ddx;

  EXPECT_TRUE(fem_optimizer->Solve(&x, &dx, &ddx));
  auto end_time = std::chrono::system_clock::now();
  std::chrono::duration<double> diff = end_time - start_time;
  AINFO << "qp_optimizer used time: " << diff.count() * 1000 << " ms.";

  AINFO << "x.size() = " << x.size();
  for (size_t i = 0; i < x.size(); ++i) {
    EXPECT_LE(x[i], fem_optimizer->x_bounds_[i].second);
    EXPECT_GE(x[i], fem_optimizer->x_bounds_[i].first);
  }
  delete fem_optimizer;
}

}  // namespace planning
}  // namespace apollo
