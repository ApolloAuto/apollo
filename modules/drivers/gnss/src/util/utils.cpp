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

#include <fcntl.h>
#include <unistd.h>
#include <string>

#include <google/protobuf/io/zero_copy_stream_impl.h>
#include <google/protobuf/text_format.h>

#include "util/utils.h"

namespace apollo {
namespace drivers {
namespace gnss {
namespace {

const char TABLE[] =
    "ABCDEFGHIJKLMNOPQRSTUVWXYZabcdefghijklmnopqrstuvwxyz0123456789+/";

const char* triplet_base64(int triplet) {
  static char result[4];
  result[0] = TABLE[(triplet >> 18) & 0x3f];
  result[1] = TABLE[(triplet >> 12) & 0x3f];
  result[2] = TABLE[(triplet >> 6) & 0x3f];
  result[3] = TABLE[triplet & 0x3f];
  return result;
}

}  // namespace

bool parse_config_text(const std::string& filename, config::Config* config) {
  int fd = open(filename.c_str(), O_RDONLY);
  if (-1 == fd) {
    return false;
  }

  google::protobuf::io::FileInputStream fs(fd);
  if (!::google::protobuf::TextFormat::Parse(&fs, config)) {
    close(fd);
    return false;
  }

  close(fd);
  return true;
}

std::string encode_base64(const std::string& in) {
  std::string out;
  if (in.empty()) {
    return out;
  }

  int in_size = in.size();

  out.reserve(((in_size - 1) / 3 + 1) * 4);

  int i = 2;
  for (; i < in_size; i += 3) {
    out.append(triplet_base64((in[i - 2] << 16) | (in[i - 1] << 8) | in[i]), 4);
  }
  if (i == in_size) {
    out.append(triplet_base64((in[i - 2] << 16) | (in[i - 1] << 8)), 3);
    out.push_back('=');
  } else if (i == in_size + 1) {
    out.append(triplet_base64(in[i - 2] << 16), 2);
    out.append("==");
  }
  return out;
}

}  // namespace gnss
}  // namespace drivers
}  // namespace apollo
