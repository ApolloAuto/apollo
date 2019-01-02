/******************************************************************************
 * Copyright 2018 The Apollo Authors. All Rights Reserved.
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

#pragma once

#include <limits>

namespace apollo {
namespace perception {
namespace base {

// Integral type equal
template <typename T>
typename std::enable_if<std::is_integral<T>::value, bool>::type Equal(
    const T& lhs, const T& rhs) {
  return lhs == rhs;
}

// Floating point type equal
template <typename T>
typename std::enable_if<std::is_floating_point<T>::value, bool>::type Equal(
    const T& lhs, const T& rhs) {
  return std::fabs(lhs - rhs) < std::numeric_limits<T>::epsilon();
}

}  // namespace base
}  // namespace perception
}  // namespace apollo
