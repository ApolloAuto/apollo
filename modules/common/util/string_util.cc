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
namespace {

// A table which maps a char to its value in Base64 mode.
std::vector<int> Base64CodeTable() {
  static const std::string kBase64Array =
      "ABCDEFGHIJKLMNOPQRSTUVWXYZabcdefghijklmnopqrstuvwxyz0123456789+/";

  std::vector<int> table(256, -1);
  for (size_t i = 0; i < kBase64Array.length(); ++i) {
    table[kBase64Array[i]] = i;
  }
  return table;
}

}  // namespace

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

std::string Base64Decode(const std::string &base64_str) {
  static const std::vector<int> kBase64CodeTable = Base64CodeTable();

  std::string bytes;
  // Binary string is generally 3/4 the length of base64 string
  bytes.reserve(base64_str.length() * 3 / 4 + 3);
  unsigned int sum = 0, sum_bits = 0;
  for (const char c : base64_str) {
    if (kBase64CodeTable[c] == -1) {
      break;
    }

    // Convert 6-bits Base64 chars to 8-bits general bytes.
    sum = (sum << 6) + kBase64CodeTable[c];
    sum_bits += 6;
    if (sum_bits >= 8) {
      bytes.push_back((sum >> (sum_bits - 8)) & 0xFF);
      sum_bits -= 8;
    }
  }
  return bytes;
}

}  // namespace util
}  // namespace common
}  // namespace apollo
