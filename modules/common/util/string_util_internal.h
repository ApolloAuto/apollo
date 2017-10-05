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
 * @brief Some string util internal helpers.
 */

#ifndef MODULES_COMMON_STRING_UTIL_INTERNAL_H_
#define MODULES_COMMON_STRING_UTIL_INTERNAL_H_

#include <iostream>
#include <string>

/**
 * @namespace apollo::common::util
 * @brief apollo::common::util
 */
namespace apollo {
namespace common {
namespace util {
namespace internal {

/**
 * @brief Iterator printer.
 */
template <typename Iter>
class IterPrinter {
 public:
  IterPrinter(const Iter& begin, const Iter& end, const std::string& delimiter)
      : begin_(begin), end_(end), delimiter_(delimiter) {}

  std::ostream& Print(std::ostream& os) const {
    for (Iter it = begin_; it != end_; ++it) {
      // Print first item without delimiter.
      if (it == begin_) {
        os << *it;
      } else {
        os << delimiter_ << *it;
      }
    }
    return os;
  }

 private:
  const Iter begin_;
  const Iter end_;
  const std::string& delimiter_;
};

/**
 * @brief Iterator printer which output iter->DebugString().
 */
template <typename Iter>
class DebugStringIterPrinter {
 public:
  DebugStringIterPrinter(const Iter& begin, const Iter& end,
                         const std::string& delimiter)
      : begin_(begin), end_(end), delimiter_(delimiter) {}

  std::ostream& Print(std::ostream& os) const {
    for (Iter it = begin_; it != end_; ++it) {
      // Print first item without delimiter.
      if (it == begin_) {
        os << it->DebugString();
      } else {
        os << delimiter_ << it->DebugString();
      }
    }
    return os;
  }

 private:
  const Iter begin_;
  const Iter end_;
  const std::string& delimiter_;
};

}  // namespace internal
}  // namespace util
}  // namespace common
}  // namespace apollo

#endif  // MODULES_COMMON_STRING_UTIL_INTERNAL_H_
