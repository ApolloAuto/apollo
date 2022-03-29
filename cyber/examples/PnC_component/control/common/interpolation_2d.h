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
 */

#pragma once

#include <map>
#include <memory>
#include <tuple>
#include <utility>
#include <vector>

/**
 * @namespace apollo::control
 * @brief apollo::control
 */
namespace apollo {
namespace control {
/**
 * @class Interpolation2D
 *
 * @brief linear interpolation from key (double, double) to one double value.
 */
class Interpolation2D {
 public:
  typedef std::vector<std::tuple<double, double, double>> DataType;
  typedef std::pair<double, double> KeyType;

  Interpolation2D() = default;

  /**
   * @brief initialize Interpolation2D internal table
   * @param xyz passing interpolation initialization table data
   * @return true if init is ok.
   */
  bool Init(const DataType &xyz);

  /**
   * @brief linear interpolate from 2D key (double, double) to one double value.
   * @param xyz passing interpolation initialization table data
   * @return true if init is ok.
   */
  double Interpolate(const KeyType &xy) const;

 private:
  double InterpolateYz(const std::map<double, double> &yz_table,
                       double y) const;

  double InterpolateValue(const double value_before, const double dist_before,
                          const double value_after,
                          const double dist_after) const;

  std::map<double, std::map<double, double>> xyz_;
};

}  // namespace control
}  // namespace apollo
