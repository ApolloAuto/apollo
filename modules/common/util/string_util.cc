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

#include "modules/common/util/string_util.h"

#include <cmath>
#include <vector>

namespace apollo {
namespace common {
namespace util {

void split(const std::string& str, char ch, std::vector<std::string>* result) {
  result->clear();
  std::stringstream ss(str);
  std::string segment;
  while (std::getline(ss, segment, ch)) {
    result->push_back(segment);
  }
}

void trim(std::string* str) {
  ltrim(str);
  rtrim(str);
}

void ltrim(std::string* str) {
  if (!str) {
    return;
  }
  str->erase(str->begin(), std::find_if(str->begin(), str->end(), [](int ch) {
               return !std::isspace(ch);
             }));
}

void rtrim(std::string* str) {
  if (!str) {
    return;
  }
  str->erase(std::find_if(str->rbegin(), str->rend(),
                          [](int ch) { return !std::isspace(ch); })
                 .base(),
             str->end());
}

}  // namespace util
}  // namespace common
}  // namespace apollo
