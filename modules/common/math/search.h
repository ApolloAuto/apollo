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
 * @brief Search-related functions.
 */

#ifndef MODULES_COMMON_MATH_SEARCH_H_
#define MODULES_COMMON_MATH_SEARCH_H_

#include <functional>

/**
 * @namespace apollo::common::math
 * @brief apollo::common::math
 */
namespace apollo {
namespace common {
namespace math {

/**
 * @brief Given a unimodal function defined on the interval,
 *        find a value on the interval to minimize the function.
 *        Reference: https://en.wikipedia.org/wiki/Golden-section_search
 * @param func The target single-variable function to minimize.
 * @param lower_bound The lower bound of the interval.
 * @param upper_bound The upper bound of the interval.
 * @param tol The tolerance of error.
 * @return The value that minimize the function fun.
 */
double GoldenSectionSearch(const std::function<double(double)> &func,
                           const double lower_bound, const double upper_bound,
                           const double tol = 1e-6);

}  // namespace math
}  // namespace common
}  // namespace apollo

#endif /* MODULES_COMMON_MATH_SEARCH_H_ */
