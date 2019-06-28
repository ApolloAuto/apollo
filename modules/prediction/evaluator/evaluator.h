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
 * @brief Define the data container base class
 */

#pragma once

#include <string>
#include <utility>
#include <vector>

#include "modules/prediction/container/obstacles/obstacle.h"

/**
 * @namespace apollo::prediction
 * @brief apollo::prediction
 */
namespace apollo {
namespace prediction {

class Evaluator {
 public:
  /**
   * @brief Constructor
   */
  Evaluator() = default;

  /**
   * @brief Destructor
   */
  virtual ~Evaluator() = default;

  // TODO(all): Need to merge the following two functions into a single one.
  // Can try using proto to pass static/dynamic env info.

  /**
   * @brief Evaluate an obstacle
   * @param Obstacle pointer
   */
  virtual bool Evaluate(Obstacle* obstacle) = 0;

  /**
   * @brief Evaluate an obstacle
   * @param Obstacle pointer
   * @param vector of all Obstacles
   */
  virtual bool Evaluate(Obstacle* obstacle,
                        std::vector<Obstacle*> dynamic_env) {
    return Evaluate(obstacle);
  }

  /**
   * @brief Get the name of evaluator
   */
  virtual std::string GetName() = 0;

 protected:
  // Helper function to convert world coordinates to relative coordinates
  // around the obstacle of interest.
  std::pair<double, double> WorldCoordToObjCoord(
      std::pair<double, double> input_world_coord,
      std::pair<double, double> obj_world_coord, double obj_world_angle) {
    double x_diff = input_world_coord.first - obj_world_coord.first;
    double y_diff = input_world_coord.second - obj_world_coord.second;
    double rho = std::sqrt(x_diff * x_diff + y_diff * y_diff);
    double theta = std::atan2(y_diff, x_diff) - obj_world_angle;

    return std::make_pair(std::cos(theta) * rho, std::sin(theta) * rho);
  }

  double WorldAngleToObjAngle(double input_world_angle,
                              double obj_world_angle) {
    return common::math::NormalizeAngle(input_world_angle - obj_world_angle);
  }

  Eigen::MatrixXf VectorToMatrixXf(const std::vector<double>& nums,
                                   const int start_index, const int end_index) {
    CHECK_LT(start_index, end_index);
    CHECK_GE(start_index, 0);
    CHECK_LE(end_index, static_cast<int>(nums.size()));
    Eigen::MatrixXf output_matrix;
    output_matrix.resize(1, end_index - start_index);
    for (int i = start_index; i < end_index; ++i) {
      output_matrix(0, i - start_index) = static_cast<float>(nums[i]);
    }
    return output_matrix;
  }

  Eigen::MatrixXf VectorToMatrixXf(const std::vector<double>& nums,
                                   const int start_index, const int end_index,
                                   const int output_num_row,
                                   const int output_num_col) {
    CHECK_LT(start_index, end_index);
    CHECK_GE(start_index, 0);
    CHECK_LE(end_index, static_cast<int>(nums.size()));
    CHECK_EQ(end_index - start_index, output_num_row * output_num_col);
    Eigen::MatrixXf output_matrix;
    output_matrix.resize(output_num_row, output_num_col);
    int input_index = start_index;
    for (int i = 0; i < output_num_row; ++i) {
      for (int j = 0; j < output_num_col; ++j) {
        output_matrix(i, j) = static_cast<float>(nums[input_index]);
        ++input_index;
      }
    }
    CHECK_EQ(input_index, end_index);
    return output_matrix;
  }

 protected:
  ObstacleConf::EvaluatorType evaluator_type_;
};

}  // namespace prediction
}  // namespace apollo
