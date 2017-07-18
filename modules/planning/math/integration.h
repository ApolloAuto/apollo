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
 * @file: integration.h
 **/

#ifndef MODULES_PLANNING_MATH_INTEGRATION_H_
#define MODULES_PLANNING_MATH_INTEGRATION_H_

#include <functional>
#include <vector>

namespace apollo {
namespace planning {

class Integration {
 public:
  Integration() = default;

  static double simpson(const std::vector<double>& funv_vec, const double dx,
                        const std::size_t nsteps);

  static double trapezoidal(const std::vector<double>& funv_vec,
                            const double dx, const std::size_t nsteps);

  // Given a target function and integral lower and upper bound,
  // compute the integral approximation using 5th order Gauss-Legendre
  // integration.
  // The target function must be a smooth function.
  // Example:
  // target function: auto func = [](const double& x) {return x * x;};
  //                  double integral = gauss_legendre(func, -2, 3);
  // This gives you the approximated integral of function x^2 in bound [-2, 3]
  static double gauss_legendre(const std::function<double(double)>& func,
                               const double lower_bound,
                               const double upper_bound);
};

}  // namespace planning
}  // namespace apollo

#endif  // MODULES_PLANNING_MATH_INTEGRATION_H_
