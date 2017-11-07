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

#ifndef MODULES_COMMON_UTIL_STRING_UTIL_H_
#define MODULES_COMMON_UTIL_STRING_UTIL_H_

#include <sstream>
#include <string>

#include "modules/common/util/string_util_internal.h"

/**
 * @namespace apollo::common::util
 * @brief apollo::common::util
 */
namespace apollo {
namespace common {
namespace util {

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
internal::IterPrinter<Iter> PrintIter(const Container& container,
                                      const std::string& delimiter = " ") {
  return {container.begin(), container.end(), delimiter};
}

template <typename Iter>
internal::IterPrinter<Iter> PrintIter(const Iter& begin, const Iter& end,
                                      const std::string& delimiter = " ") {
  return {begin, end, delimiter};
}

template <typename T, int length>
internal::IterPrinter<T*> PrintIter(
    T (&array)[length], const std::string& delimiter = " ") {
  return {array, array + length, delimiter};
}

template <typename T, int length>
internal::IterPrinter<T*> PrintIter(
    T (&array)[length], T* end, const std::string& delimiter = " ") {
  return {array, end, delimiter};
}

/**
 * @brief Make conatiners and iterators printable. Similar to PrintIter but
 *        output the DebugString().
 */
template <typename Container,
          typename Iter = typename Container::const_iterator>
internal::DebugStringIterPrinter<Iter> PrintDebugStringIter(
    const Container& container, const std::string& delimiter = " ") {
  return {container.begin(), container.end(), delimiter};
}

template <typename Iter>
internal::DebugStringIterPrinter<Iter> PrintDebugStringIter(
    const Iter& begin, const Iter& end, const std::string& delimiter = " ") {
  return {begin, end, delimiter};
}

}  // namespace util
}  // namespace common

// Operators should be globally visible in apollo namespace.
template <typename Iter>
std::ostream& operator<<(
    std::ostream& os,
    const common::util::internal::IterPrinter<Iter>& printer) {
  return printer.Print(os);
}

template <typename Iter>
std::ostream& operator<<(
    std::ostream& os,
    const common::util::internal::DebugStringIterPrinter<Iter>& printer) {
  return printer.Print(os);
}

}  // namespace apollo

#endif  // MODULES_COMMON_UTIL_STRING_UTIL_H_
