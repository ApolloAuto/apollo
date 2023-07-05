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
 * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or
 *implied. See the License for the specific language governing
 *permissions and limitations under the License.
 *****************************************************************************/

#pragma once

#include <limits>
#include <utility>
#include <vector>

#include "Eigen/Dense"
#include "modules/common_msgs/basic_msgs/pnc_point.pb.h"
#include "modules/prediction/common/prediction_gflags.h"

namespace apollo {
namespace prediction {
namespace math_util {
/**
 * @brief Normalize the value by specified mean and standard deviation.
 * @param value The value to be normalized.
 * @param mean The mean used for normalization.
 * @param std The standard deviation used for normalization.
 * @return The normalized value.
 */
double Normalize(const double value, const double mean, const double std);

/**
 * @brief RELU function used in neural networks as an activation function.
 * @param value The input.
 * @return The output of RELU function.
 */
double Relu(const double value);

/**
 * @brief Softmax function used in neural networks as an activation function.
 * @param vector The input.
 * @return The output of Softmax function.
 */
std::vector<double> Softmax(const std::vector<double>& value,
                            bool use_exp = true);

/**
 * @brief Solve quadratic equation.
 * @param coefficients The coefficients of quadratic equation.
 * @param roots Two roots of the equation if any.
 * @return An integer indicating the success of solving equation.
 */
int SolveQuadraticEquation(const std::vector<double>& coefficients,
                           std::pair<double, double>* roots);

/**
 * @brief Evaluate quintic polynomial.
 * @param coefficients of the quintic polynomial, lower to higher.
 * @param parameter of the quintic polynomial.
 * @return order of derivative to evaluate.
 */
double EvaluateQuinticPolynomial(const std::array<double, 6>& coeffs,
                                 const double t, const uint32_t order,
                                 const double end_t, const double end_v);

/**
 * @brief Evaluate quartic polynomial.
 * @param coefficients of the quartic polynomial, lower to higher.
 * @param parameter of the quartic polynomial.
 * @return order of derivative to evaluate.
 */
double EvaluateQuarticPolynomial(const std::array<double, 5>& coeffs,
                                 const double t, const uint32_t order,
                                 const double end_t, const double end_v);

/**
 * @brief Evaluate cubic polynomial.
 * @param coefficients of the cubic polynomial, lower to higher.
 * @param parameter of the cubic polynomial.
 * @param end_t ending time for extrapolation.
 * @param end_v ending velocity for extrapolation.
 * @return order of derivative to evaluate.
 */
double EvaluateCubicPolynomial(
    const std::array<double, 4>& coefs, const double t, const uint32_t order,
    const double end_t = std::numeric_limits<double>::infinity(),
    const double end_v = 0.0);

template <std::size_t N>
std::array<double, 2 * N - 2> ComputePolynomial(
    const std::array<double, N - 1>& start_state,
    const std::array<double, N - 1>& end_state, const double param);

template <>
inline std::array<double, 4> ComputePolynomial<3>(
    const std::array<double, 2>& start_state,
    const std::array<double, 2>& end_state, const double param) {
  std::array<double, 4> coefs;
  coefs[0] = start_state[0];
  coefs[1] = start_state[1];

  auto m0 = end_state[0] - start_state[0] - start_state[1] * param;
  auto m1 = end_state[1] - start_state[1];

  auto param_p3 = param * param * param;
  coefs[3] = (m1 * param - 2.0 * m0) / param_p3;

  coefs[2] = (m1 - 3.0 * coefs[3] * param * param) / param * 0.5;
  return coefs;
}

double GetSByConstantAcceleration(const double v0, const double acceleration,
                                  const double t);

}  // namespace math_util

namespace predictor_util {
/**
 * @brief Translate a point.
 * @param translate_x The translation along x-axis.
 * @param translate_y The translation along y-axis.
 * @param point The point to be translated.
 */
void TranslatePoint(const double translate_x, const double translate_y,
                    common::TrajectoryPoint* point);

/**
 * @brief Generate a set of free move trajectory points
 * @param state matrix
 * @param transition matrix
 * @param heading
 * @param start time
 * @param total number of generated trajectory points required
 * @param trajectory point interval period
 * @param generated trajectory points
 */
void GenerateFreeMoveTrajectoryPoints(
    Eigen::Matrix<double, 6, 1>* state,
    const Eigen::Matrix<double, 6, 6>& transition, double theta,
    const double start_time, const std::size_t num, const double period,
    std::vector<common::TrajectoryPoint>* points);

/**
 * @brief Adjust a speed value according to a curvature. If the input speed
 *        is okay on the input curvature, return the original speed, otherwise,
 *        adjust the speed.
 * @param speed The original speed value.
 * @param curvature The curvature value.
 * @return The adjusted speed according to the curvature.
 */
double AdjustSpeedByCurvature(const double speed, const double curvature);

}  // namespace predictor_util
}  // namespace prediction
}  // namespace apollo
