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

// TODO(xiaoxq): Migrate to absl::StrFormat after absl upgraded.
using google::protobuf::StringPrintf;

/**
 * @brief Make conatiners and iterators printable.
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
