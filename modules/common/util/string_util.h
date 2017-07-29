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
 * @brief Some string util functions.
 */

#ifndef MODULES_COMMON_STRING_UTIL_H_
#define MODULES_COMMON_STRING_UTIL_H_

#include <iostream>
#include <sstream>
#include <string>

/**
 * @namespace apollo::common::util
 * @brief apollo::common::util
 */
namespace apollo {
namespace common {
namespace util {
namespace {

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
  }

 private:
  const Iter begin_;
  const Iter end_;
  const std::string& delimiter_;
};

}  // namespace

/**
 * @brief Check if a string ends with a pattern.
 * @param ori The original string. To see if it ends with a specified pattern.
 * @param pat The target pattern. To see if the original string ends with it.
 * @return Whether the original string ends with the specified pattern.
 */
inline bool EndWith(const std::string& ori, const std::string& pat) {
  return ori.length() >= pat.length() &&
      ori.compare(ori.length() - pat.length(), pat.length(), pat) == 0;
}

/**
 * @brief Concat parameters to a string, e.g.: StrCat("age = ", 32)
 * @return String of concated parameters.
 */
template <typename ...T>
std::string StrCat(const T& ...args) {
  std::ostringstream oss;
  // Expand args and pass to oss.
  std::initializer_list<char>{(oss << args, ' ') ... };
  return oss.str();
}

/**
 * @brief Make arrays, conatiners and iterators printable.
 *
 * Usage:
 *   vector<int> vec = {1, 2, 3};
 *   std::cout << PrintIter(vec);
 *   std::cout << PrintIter(vec, ",");
 *   std::cout << PrintIter(vec.begin(), vec.end());
 *   std::cout << PrintIter(vec.begin(), vec.end(), "|");
 *
 *   int array[] = {1, 2, 3};
 *   std::cout << PrintIter(array);
 *   std::cout << PrintIter(array, "|");
 *   std::cout << PrintIter(array + 0, array + 10, "|");
 */
template <typename Container,
          typename Iter = typename Container::const_iterator>
IterPrinter<Iter> PrintIter(const Container& container,
                            const std::string& delimiter = " ") {
  return IterPrinter<Iter>(container.begin(), container.end(), delimiter);
}

template <typename Iter>
IterPrinter<Iter> PrintIter(const Iter& begin, const Iter& end,
                            const std::string& delimiter = " ") {
  return IterPrinter<Iter>(begin, end, delimiter);
}

template <typename T, int length>
IterPrinter<T*> PrintIter(T (&array)[length],
                          const std::string& delimiter = " ") {
  return IterPrinter<T*>(array, array + length, delimiter);
}

template <typename T, int length>
IterPrinter<T*> PrintIter(T (&array)[length], T* end,
                          const std::string& delimiter = " ") {
  return IterPrinter<T*>(array, end, delimiter);
}

}  // namespace util
}  // namespace common

// Operators should be globally visible in apollo namespace.
template <typename Iter>
std::ostream& operator<<(std::ostream& os,
                         const common::util::IterPrinter<Iter>& printer) {
  return printer.Print(os);
}

}  // namespace apollo

#endif  // MODULES_COMMON_STRING_UTIL_H_
