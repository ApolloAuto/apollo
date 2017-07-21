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
 * @brief Some util functions.
 */

#ifndef MODULES_COMMON_UTIL_H_
#define MODULES_COMMON_UTIL_H_

#include <iostream>
#include <string>

#include "modules/common/proto/path_point.pb.h"

/**
 * @namespace apollo::common::util
 * @brief apollo::common::util
 */
namespace apollo {
namespace common {
namespace util {

/**
 * @brief Check if a string ends with a pattern.
 * @param original The original string. To see if it ends with some
 *        specified pattern.
 * @param pattern The target pattern. To see if the original string ends
 *        with it.
 * @return Whether the original string ends with the specified pattern.
 */
bool EndWith(const std::string& original, const std::string& pattern);

/**
 * @brief create a SL point
 * @param s the s value
 * @param l the l value
 * @return a SLPoint instance
 */
apollo::common::SLPoint MakeSLPoint(const double s, const double l);

/**
 * calculate the distance beteween PathPoint a and PathPoint b
 * @param a one path point
 * @param b another path point
 * @return sqrt((a.x-b.x)^2 + (a.y-b.y)^2), i.e., the Euclid distance on XY
 * dimension
 */
double Distance2D(const PathPoint& a, const PathPoint& b);

}  // namespace util
}  // namespace common
}  // namespace apollo

template <typename A, typename B>
std::ostream& operator<<(std::ostream& os, std::pair<A, B>& p) {
  return os << "first: " << p.first << ", second: " << p.second;
}

#endif  // MODULES_COMMON_UTIL_H_
