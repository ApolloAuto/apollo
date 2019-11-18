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

const char* tripletBase64(const int triplet) {
  static char result[4];
  result[0] = kBase64Array[(triplet >> 18) & 0x3f];
  result[1] = kBase64Array[(triplet >> 12) & 0x3f];
  result[2] = kBase64Array[(triplet >> 6) & 0x3f];
  result[3] = kBase64Array[triplet & 0x3f];
  return result;
}

}  // namespace

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
