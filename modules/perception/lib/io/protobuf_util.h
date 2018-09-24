/******************************************************************************
 * Copyright 2018 The Apollo Authors. All Rights Reserved.
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
#ifndef PERCEPTION_LIB_IO_PROTOBUF_UTIL_H_
#define PERCEPTION_LIB_IO_PROTOBUF_UTIL_H_

#include <fcntl.h>
#include <google/protobuf/io/zero_copy_stream_impl.h>
#include <google/protobuf/text_format.h>
#include <string>

#include "modules/perception/base/log.h"

namespace apollo {
namespace perception {
namespace lib {

// @brief load protobuf(TXT) data from file.
template <typename T>
bool ParseProtobufFromFile(const std::string &file_name, T *pb) {
  int fd = open(file_name.c_str(), O_RDONLY);
  if (fd < 0) {
    LOG_ERROR << "ProtobufParser load file failed. file: " << file_name;
    return false;
  }

  google::protobuf::io::FileInputStream fs(fd);
  if (!google::protobuf::TextFormat::Parse(&fs, pb)) {
    LOG_ERROR << "ProtobufParser parse data failed. file:" << file_name;
    close(fd);
    return false;
  }
  close(fd);
  return true;
}

}  // namespace lib
}  // namespace perception
}  // namespace apollo

#endif  // PERCEPTION_LIB_IO_PROTOBUF_UTIL_H_
