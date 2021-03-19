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
 * @brief Functions to compute integral.
 */

#pragma once

#include <array>
#include <functional>
#include <utility>
#include <vector>

/**
 * @namespace apollo::common::math
 * @brief apollo::common::math
 */
namespace apollo {
namespace common {
namespace math {

double IntegrateBySimpson(const std::vector<double>& funv_vec, const double dx,
                          const std::size_t nsteps);

double IntegrateByTrapezoidal(const std::vector<double>& funv_vec,
                              const double dx, const std::size_t nsteps);

/**
 * @brief Get the points and weights for different ordered Gauss-Legendre
 *        integration. Currently support order 2 - 10. Other input order will
 *        trigger compiling error.
 */
template <std::size_t N>
std::pair<std::array<double, N>, std::array<double, N>>
GetGaussLegendrePoints();

template <>
inline std::pair<std::array<double, 2>, std::array<double, 2>>
GetGaussLegendrePoints<2>() {
  std::array<double, 2> x;
  x[0] = -5.77350269189625764507e-01;
  x[1] = 5.77350269189625764507e-01;

  std::array<double, 2> w;
  w[0] = 1.0;
  w[1] = 1.0;

  return std::make_pair(x, w);
}

template <>
inline std::pair<std::array<double, 3>, std::array<double, 3>>
GetGaussLegendrePoints<3>() {
  std::array<double, 3> x;
  x[0] = 0.00000000000000000000e+00;
  x[1] = 7.74596669241483377010e-01;
  x[2] = -7.74596669241483377010e-01;

  std::array<double, 3> w;
  w[0] = 8.88888888888888888877e-01;
  w[1] = 5.55555555555555555562e-01;
  w[2] = 5.55555555555555555562e-01;

  return std::make_pair(x, w);
}

template <>
inline std::pair<std::array<double, 4>, std::array<double, 4>>
GetGaussLegendrePoints<4>() {
  std::array<double, 4> x;
  x[0] = 3.39981043584856264792e-01;
  x[1] = -3.39981043584856264792e-01;
  x[2] = 8.61136311594052575248e-01;
  x[3] = -8.61136311594052575248e-01;

  std::array<double, 4> w;
  w[0] = 6.52145154862546142644e-01;
  w[1] = 6.52145154862546142644e-01;
  w[2] = 3.47854845137453857383e-01;
  w[3] = 3.47854845137453857383e-01;

  return std::make_pair(x, w);
}

template <>
inline std::pair<std::array<double, 5>, std::array<double, 5>>
GetGaussLegendrePoints<5>() {
  std::array<double, 5> x;
  x[0] = 0.00000000000000000000e+00;
  x[1] = 5.38469310105683091018e-01;
  x[2] = -5.38469310105683091018e-01;
  x[3] = 9.06179845938663992811e-01;
  x[4] = -9.06179845938663992811e-01;

  std::array<double, 5> w;
  w[0] = 5.68888888888888888883e-01;
  w[1] = 4.78628670499366468030e-01;
  w[2] = 4.78628670499366468030e-01;
  w[3] = 2.36926885056189087515e-01;
  w[4] = 2.36926885056189087515e-01;

  return std::make_pair(x, w);
}

template <>
inline std::pair<std::array<double, 6>, std::array<double, 6>>
GetGaussLegendrePoints<6>() {
  std::array<double, 6> x;
  x[0] = 6.61209386466264513688e-01;
  x[1] = -6.61209386466264513688e-01;
  x[2] = 2.38619186083196908630e-01;
  x[3] = -2.38619186083196908630e-01;
  x[4] = 9.32469514203152027832e-01;
  x[5] = -9.32469514203152027832e-01;

  std::array<double, 6> w;
  w[0] = 3.60761573048138607569e-01;
  w[1] = 3.60761573048138607569e-01;
  w[2] = 4.67913934572691047389e-01;
  w[3] = 4.67913934572691047389e-01;
  w[4] = 1.71324492379170345043e-01;
  w[5] = 1.71324492379170345043e-01;

  return std::make_pair(x, w);
}

template <>
inline std::pair<std::array<double, 7>, std::array<double, 7>>
GetGaussLegendrePoints<7>() {
  std::array<double, 7> x;
  x[0] = 0.00000000000000000000e+00;
  x[1] = 4.05845151377397166917e-01;
  x[2] = -4.05845151377397166917e-01;
  x[3] = 7.41531185599394439864e-01;
  x[4] = -7.41531185599394439864e-01;
  x[5] = 9.49107912342758524541e-01;
  x[6] = -9.49107912342758524541e-01;

  std::array<double, 7> w;
  w[0] = 4.17959183673469387749e-01;
  w[1] = 3.81830050505118944961e-01;
  w[2] = 3.81830050505118944961e-01;
  w[3] = 2.79705391489276667890e-01;
  w[4] = 2.79705391489276667890e-01;
  w[5] = 1.29484966168869693274e-01;
  w[6] = 1.29484966168869693274e-01;

  return std::make_pair(x, w);
}

template <>
inline std::pair<std::array<double, 8>, std::array<double, 8>>
GetGaussLegendrePoints<8>() {
  std::array<double, 8> x;
  x[0] = 1.83434642495649804936e-01;
  x[1] = -1.83434642495649804936e-01;
  x[2] = 5.25532409916328985830e-01;
  x[3] = -5.25532409916328985830e-01;
  x[4] = 7.96666477413626739567e-01;
  x[5] = -7.96666477413626739567e-01;
  x[6] = 9.60289856497536231661e-01;
  x[7] = -9.60289856497536231661e-01;

  std::array<double, 8> w;
  w[0] = 3.62683783378361982976e-01;
  w[1] = 3.62683783378361982976e-01;
  w[2] = 3.13706645877887287338e-01;
  w[3] = 3.13706645877887287338e-01;
  w[4] = 2.22381034453374470546e-01;
  w[5] = 2.22381034453374470546e-01;
  w[6] = 1.01228536290376259154e-01;
  w[7] = 1.01228536290376259154e-01;

  return std::make_pair(x, w);
}

template <>
inline std::pair<std::array<double, 9>, std::array<double, 9>>
GetGaussLegendrePoints<9>() {
  std::array<double, 9> x;
  x[0] = 0.00000000000000000000e+00;
  x[1] = 8.36031107326635794313e-01;
  x[2] = -8.36031107326635794313e-01;
  x[3] = 9.68160239507626089810e-01;
  x[4] = -9.68160239507626089810e-01;
  x[5] = 3.24253423403808929042e-01;
  x[6] = -3.24253423403808929042e-01;
  x[7] = 6.13371432700590397285e-01;
  x[8] = -6.13371432700590397285e-01;

  std::array<double, 9> w;
  w[0] = 3.30239355001259763154e-01;
  w[1] = 1.80648160694857404059e-01;
  w[2] = 1.80648160694857404059e-01;
  w[3] = 8.12743883615744119737e-02;
  w[4] = 8.12743883615744119737e-02;
  w[5] = 3.12347077040002840057e-01;
  w[6] = 3.12347077040002840057e-01;
  w[7] = 2.60610696402935462313e-01;
  w[8] = 2.60610696402935462313e-01;

  return std::make_pair(x, w);
}

template <>
inline std::pair<std::array<double, 10>, std::array<double, 10>>
GetGaussLegendrePoints<10>() {
  std::array<double, 10> x;
  x[0] = 1.48874338981631210881e-01;
  x[1] = -1.48874338981631210881e-01;
  x[2] = 4.33395394129247190794e-01;
  x[3] = -4.33395394129247190794e-01;
  x[4] = 6.79409568299024406207e-01;
  x[5] = -6.79409568299024406207e-01;
  x[6] = 8.65063366688984510759e-01;
  x[7] = -8.65063366688984510759e-01;
  x[8] = 9.73906528517171720066e-01;
  x[9] = -9.73906528517171720066e-01;

  std::array<double, 10> w;
  w[0] = 2.95524224714752870187e-01;
  w[1] = 2.95524224714752870187e-01;
  w[2] = 2.69266719309996355105e-01;
  w[3] = 2.69266719309996355105e-01;
  w[4] = 2.19086362515982044000e-01;
  w[5] = 2.19086362515982044000e-01;
  w[6] = 1.49451349150580593150e-01;
  w[7] = 1.49451349150580593150e-01;
  w[8] = 6.66713443086881375920e-02;
  w[9] = 6.66713443086881375920e-02;

  return std::make_pair(x, w);
}

/**
 * @brief Compute the integral of a target single-variable function
 *        from a lower bound to an upper bound, by 5-th Gauss-Legendre method
 * Given a target function and integral lower and upper bound,
 * compute the integral approximation using 5th order Gauss-Legendre
 * integration.
 * The target function must be a smooth function.
 * Example:
 * target function: auto func = [](const double x) {return x * x;};
 *                  double integral = gauss_legendre(func, -2, 3);
 * This gives you the approximated integral of function x^2 in bound [-2, 3]
 *
 * reference: https://en.wikipedia.org/wiki/Gaussian_quadrature
 *            http://www.mymathlib.com/quadrature/gauss_legendre.html
 *
 * @param func The target single-variable function
 * @param lower_bound The lower bound of the integral
 * @param upper_bound The upper bound of the integral
 * @return The integral result
 */
template <std::size_t N>
double IntegrateByGaussLegendre(const std::function<double(double)>& func,
                                const double lower_bound,
                                const double upper_bound) {
  auto p = GetGaussLegendrePoints<N>();

  std::array<double, N> x = p.first;
  std::array<double, N> w = p.second;

  const double t = (upper_bound - lower_bound) * 0.5;
  const double m = (upper_bound + lower_bound) * 0.5;

  double integral = 0.0;
  for (size_t i = 0; i < N; ++i) {
    integral += w[i] * func(t * x[i] + m);
  }

  return integral * t;
}

}  // namespace math
}  // namespace common
}  // namespace apollo
