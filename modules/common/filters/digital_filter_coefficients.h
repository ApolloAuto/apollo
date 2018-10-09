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
 * @file digital_filter_coefficients.h
 * @brief Functions to generate coefficients for digital filter.
 */

#pragma once

#include <vector>

/**
 * @namespace apollo::common
 * @brief apollo::common
 */
namespace apollo {
namespace common {

/**
 * @brief Get low-pass coefficients for digital filter.
 * @param ts Time interval between signals.
 * @param cutoff_freq Cutoff of frequency to filter high-frequency signals out.
 * @param denominators Denominator coefficients for digital filter.
 * @param numerators Numerator coefficients for digital filter.
 */
void LpfCoefficients(const double ts, const double cutoff_freq,
                     std::vector<double> *denominators,
                     std::vector<double> *numerators);

}  // namespace common
}  // namespace apollo
