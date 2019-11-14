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

#pragma once

#include <functional>
#include <sstream>
#include <string>
#include <vector>

#include "absl/strings/str_cat.h"
#include "boost/algorithm/string.hpp"
#include "google/protobuf/stubs/stringprintf.h"
#include "google/protobuf/stubs/strutil.h"

/**
 * @namespace apollo::common::util
 * @brief apollo::common::util
 */
namespace apollo {
namespace common {
namespace util {

// Expose some useful utils from protobuf.
using absl::StrCat;
using google::protobuf::Join;
// TODO(xiaoxq): Migrate to absl::StrFormat after absl upgraded.
using google::protobuf::StringPrintf;

template <typename T>
std::string Print(const T& val) {
  std::ostringstream oss;
  oss << val;
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
template <typename Iter>
std::string PrintIter(const Iter& begin, const Iter& end,
                      const std::string& delimiter = " ") {
  std::string result;
  Join(begin, end, delimiter.c_str(), &result);
  return result;
}

template <typename Iter>
std::string PrintIter(const Iter& begin, const Iter& end,
                      const std::function<std::string(Iter)> transformer,
                      const std::string& delimiter = " ") {
  std::string result;
  if (transformer) {
    for (auto iter = begin; iter != end; ++iter) {
      if (iter == begin) {
        absl::StrAppend(&result, transformer(*iter));
      } else {
        absl::StrAppend(&result, delimiter, transformer(*iter));
      }
    }
  } else {
    PrintIter(begin, end, delimiter);
  }
  return result;
}

template <typename Container, typename Iter>
std::string PrintIter(const Container& container,
                      const std::function<std::string(Iter)> transformer,
                      const std::string& delimiter = " ") {
  return PrintIter(container.begin(), container.end(), transformer, delimiter);
}

template <typename Container>
std::string PrintIter(const Container& container,
                      const std::string& delimiter = " ") {
  return PrintIter(container.begin(), container.end(), delimiter);
}

template <typename T, int length>
std::string PrintIter(T (&array)[length], T* end,
                      const std::string& delimiter = " ") {
  std::string result;
  Join(array, end, delimiter.c_str(), &result);
  return result;
}

template <typename T, int length>
std::string PrintIter(T (&array)[length], const std::string& delimiter = " ") {
  return PrintIter(array, array + length, delimiter);
}

/**
 * @brief Make conatiners and iterators printable. Similar to PrintIter but
 *        output the DebugString().
 */
template <typename Iter>
std::string PrintDebugStringIter(const Iter& begin, const Iter& end,
                                 const std::string& delimiter = " ") {
  std::string result;
  for (auto iter = begin; iter != end; ++iter) {
    if (iter == begin) {
      absl::StrAppend(&result, iter->DebugString());
    } else {
      absl::StrAppend(&result, delimiter, iter->DebugString());
    }
  }
  return result;
}

template <typename Container>
std::string PrintDebugStringIter(const Container& container,
                                 const std::string& delimiter = " ") {
  return PrintDebugStringIter(container.begin(), container.end(), delimiter);
}

std::string DecodeBase64(const std::string& base64_str);

std::string EncodeBase64(const std::string& in);

}  // namespace util
}  // namespace common
}  // namespace apollo
