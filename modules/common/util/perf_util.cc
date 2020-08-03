/******************************************************************************
 * Copyright 2020 The Apollo Authors. All Rights Reserved.
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
#include "modules/common/util/perf_util.h"

namespace {
std::string func_name_simplified(const std::string& str) {
  constexpr char kLeftBracket = '(';
  constexpr char kSpace = ' ';

  auto end = str.find(kLeftBracket);
  auto begin = str.rfind(kSpace, end);

  if (begin == std::string::npos) {
    return str.substr(0, end);
  } else if (end == std::string::npos) {
    return str.substr(begin + 1);
  } else {
    return str.substr(begin + 1, end - begin - 1);
  }
}
}  // namespace

namespace apollo {
namespace common {
namespace util {

std::string function_signature(const std::string& func_name,
                               const std::string& indicator) {
  auto simplified_name = func_name_simplified(func_name);
  if (indicator.empty()) {
    return simplified_name;
  }
  return absl::StrCat(indicator, "_", simplified_name);
}

}  // namespace util
}  // namespace common
}  // namespace apollo
