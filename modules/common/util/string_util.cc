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

static const char kBase64Array[] =
    "ABCDEFGHIJKLMNOPQRSTUVWXYZabcdefghijklmnopqrstuvwxyz0123456789+/";

// A table which maps a char to its value in Base64 mode.
std::vector<int> Base64CodeTable() {
  std::vector<int> table(256, -1);
  const size_t base64_array_length = strlen(kBase64Array);
  for (size_t i = 0; i < base64_array_length; ++i) {
    table[kBase64Array[i]] = static_cast<int>(i);
  }
  return table;
}

const char* tripletBase64(const int triplet) {
  static char result[4];
  result[0] = kBase64Array[(triplet >> 18) & 0x3f];
  result[1] = kBase64Array[(triplet >> 12) & 0x3f];
  result[2] = kBase64Array[(triplet >> 6) & 0x3f];
  result[3] = kBase64Array[triplet & 0x3f];
  return result;
}

}  // namespace

int Split(const std::string& str, char ch, std::vector<std::string>* result) {
  std::stringstream ss(str);
  std::string segment;
  int count = 0;
  while (std::getline(ss, segment, ch)) {
    result->push_back(segment);
    ++count;
  }
  return count;
}

std::string DecodeBase64(const std::string& base64_str) {
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
      bytes.push_back(static_cast<char>((sum >> (sum_bits - 8)) & 0xFF));
      sum_bits -= 8;
    }
  }
  return bytes;
}

std::string EncodeBase64(const std::string& in) {
  std::string out;
  if (in.empty()) {
    return out;
  }

  const size_t in_size = in.size();

  out.reserve(((in_size - 1) / 3 + 1) * 4);

  size_t i = 2;
  for (; i < in_size; i += 3) {
    out.append(tripletBase64((in[i - 2] << 16) | (in[i - 1] << 8) | in[i]), 4);
  }
  if (i == in_size) {
    out.append(tripletBase64((in[i - 2] << 16) | (in[i - 1] << 8)), 3);
    out.push_back('=');
  } else if (i == in_size + 1) {
    out.append(tripletBase64(in[i - 2] << 16), 2);
    out.append("==");
  }
  return out;
}

}  // namespace util
}  // namespace common
}  // namespace apollo
