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
 * @brief Defines the DigitalFilter class.
 */

#pragma once

#include <deque>
#include <vector>

/**
 * @namespace apollo::common
 * @brief apollo::common
 */
namespace apollo {
namespace common {

/**
 * @class DigitalFilter
 * @brief The DigitalFilter class is used to pass signals with a frequency
 * lower than a certain cutoff frequency and attenuates signals with
 * frequencies higher than the cutoff frequency.
 */
class DigitalFilter {
 public:
  DigitalFilter() = default;

  /**
   * @brief Initializes a DigitalFilter with given denominators and numerators.
   * @param denominators The denominators of the DigitalFilter.
   * @param numerators The numerators of the DigitalFilter.
   */
  DigitalFilter(const std::vector<double> &denominators,
                const std::vector<double> &numerators);

  /**
   * @brief Default destructor.
   */
  ~DigitalFilter() = default;

  /**
   * @brief Processes a new measurement with the filter.
   * @param x_insert The new input to be processed by the filter.
   */
  double Filter(const double x_insert);
  /**
   * @desc: Filter by the input x_insert
   * Input: new value of x_insert
   * Remove x[n - 1], insert x_insert into x[0]
   * Remove y[n - 1],
   * Solve den[0] * y + den[1] * y[0] + ... + den[n - 1]*y[n - 2] =
   *   num[0] * x[0] + num[1] * x[1] + ... + num[n - 1] * x[n - 1] for y
   * Insert y into y[0]
   * Output: y[0]
   */

  /**
   * @brief set denominators by an input vector
   * @param denominators The denominators of filter
   */
  void set_denominators(const std::vector<double> &denominators);

  /**
   * @brief set numerators by an input vector
   * @param numerators The numerators of filter
   */
  void set_numerators(const std::vector<double> &numerators);

  /**
   * @brief set denominators and numerators
   * @param denominators The denominators of filter
   * @param numerators The numerators of filter
   */
  void set_coefficients(const std::vector<double> &denominators,
                        const std::vector<double> &numerators);

  /**
   * @brief set filter deadzone
   * @param deadzone The value of deadzone
   */
  void set_dead_zone(const double deadzone);

  /**
   * @brief get denominators
   * @return vector<double> The denominators of filter
   */
  const std::vector<double> &denominators() const;

  /**
   * @brief get numerators
   * @return vector<double> The numerators of filter
   */
  const std::vector<double> &numerators() const;

  /**
   * @brief get dead_zone
   * @return double The dead_zone
   */
  double dead_zone() const;

 private:
  /**
   * @desc: Update the last-filtered value,
   *        if the difference is less than dead_zone_
   */
  double UpdateLast(const double input);

  /**
   * @desc: Compute the inner product of values[coeff_start : coeff_end] and
   *        coefficients[coeff_start : coeff_end]
   */
  double Compute(const std::deque<double> &values,
                 const std::vector<double> &coefficients,
                 const std::size_t coeff_start, const std::size_t coeff_end);

  // Front is latest, back is oldest.
  std::deque<double> x_values_;

  // Front is latest, back is oldest.
  std::deque<double> y_values_;

  // Coefficients with y values
  std::vector<double> denominators_;

  // Coefficients with x values
  std::vector<double> numerators_;

  // threshold of updating last-filtered value
  double dead_zone_ = 0.0;

  // last-filtered value
  double last_ = 0.0;
};

}  // namespace common
}  // namespace apollo
