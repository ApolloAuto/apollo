/******************************************************************************
 * Copyright 2019 The CiDi Authors. All Rights Reserved.
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
#include <google/protobuf/io/zero_copy_stream_impl.h>
#include <google/protobuf/text_format.h>
#include <string>

#include "modules/drivers/cidiv2x/stream/util/utils.h"

namespace apollo {
namespace drivers {
namespace cidiv2x {
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


}  // namespace cidiv2x
}  // namespace drivers
}  // namespace apollo
