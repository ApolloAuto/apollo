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

#ifndef MODULES_PREDICTION_COMMON_PREDICTION_UTIL_H_
#define MODULES_PREDICTION_COMMON_PREDICTION_UTIL_H_

#include <utility>
#include <vector>

#include "Eigen/Dense"
#include "modules/common/proto/pnc_point.pb.h"
#include "modules/prediction/proto/lane_graph.pb.h"

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
 * @brief Sigmoid function used in neural networks as an activation function.
 * @param value The input.
 * @return The output of sigmoid function.
 */
double Sigmoid(const double value);

/**
 * @brief RELU function used in neural networks as an activation function.
 * @param value The input.
 * @return The output of RELU function.
 */
double Relu(const double value);

/**
 * @brief Solve quadratic equation.
 * @param coefficients The coefficients of quadratic equation.
 * @param roots Two roots of the equation if any.
 * @return An integer indicating the success of solving equation.
 */
int SolveQuadraticEquation(const std::vector<double>& coefficients,
                           std::pair<double, double>* roots);

}  // namespace math_util

namespace predictor_util {
/**
 * @brief Translate a point.
 * @param translate_x The translation along x-axis.
 * @param translate_y The translation along y-axis.
 * @param point The point to be translated.
 */
void TranslatePoint(const double translate_x, const double translate_y,
                    ::apollo::common::TrajectoryPoint* point);

/**
 * @brief Generate a set of free move trajectory points
 * @param state matrix
 * @param transition matrix
 * @param total number of generated trajectory points required
 * @param trajectory point interval frequency
 * @param generated trajectory points
 */
void GenerateFreeMoveTrajectoryPoints(
    Eigen::Matrix<double, 6, 1>* state,
    const Eigen::Matrix<double, 6, 6>& transition, const size_t num,
    const double freq, std::vector<::apollo::common::TrajectoryPoint>* points);

/**
 * @brief Generate a set of lane sequence trajectory points
 * @param state matrix
 * @param transition matrix
 * @param lane sequence
 * @param total number of generated trajectory points required
 * @param trajectory point interval frequency
 * @param generated trajectory points
 */
void GenerateLaneSequenceTrajectoryPoints(
    Eigen::Matrix<double, 4, 1>* state, Eigen::Matrix<double, 4, 4>* transition,
    const LaneSequence& sequence, const size_t num, const double freq,
    std::vector<::apollo::common::TrajectoryPoint>* points);

/**
 * @brief Draw trajectory points for still obstacle
 * @param obstacle obstacle
 * @param lane_sequence the specified lane sequence
 * @param total_time total time of prediction
 * @param freq time step between prediction trajectory points
 * @param points the output trajectory points
 */
void GenerateStillSequenceTrajectoryPoints(
    const double position_x, const double position_y, const double theta,
    const double total_time, const double freq,
    std::vector<apollo::common::TrajectoryPoint>* points);

}  // namespace predictor_util
}  // namespace prediction
}  // namespace apollo

#endif  // MODULES_PREDICTION_COMMON_PREDICTION_UTIL_H_
