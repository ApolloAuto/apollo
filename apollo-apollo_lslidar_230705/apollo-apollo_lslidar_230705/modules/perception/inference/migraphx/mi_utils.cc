/******************************************************************************
 * Copyright 2022 The Apollo Authors. All Rights Reserved.
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

#include "modules/perception/inference/migraphx/mi_utils.h"

#include <fcntl.h>

#include <limits>

#include "cyber/common/log.h"

namespace apollo {
namespace perception {
namespace inference {

bool ReadProtoFromTextFile(const std::string &filename,
                           google::protobuf::Message *proto) {
  int fd = open(filename.c_str(), O_RDONLY);
  if (fd < 0) {
    AERROR << "cannot open file " << filename;
    return false;
  }
  google::protobuf::io::FileInputStream raw_input(fd);

  bool success = google::protobuf::TextFormat::Parse(&raw_input, proto);

  close(fd);
  return success;
}

bool ReadProtoFromBinaryFile(const std::string &filename,
                             google::protobuf::Message *proto) {
  int fd = open(filename.c_str(), O_RDONLY);
  if (fd < 0) {
    AERROR << "cannot open file " << filename;
    return false;
  }
  google::protobuf::io::FileInputStream raw_input(fd);
  google::protobuf::io::CodedInputStream coded_input(&raw_input);
  coded_input.SetTotalBytesLimit(std::numeric_limits<int>::max(), 536870912);

  bool success = proto->ParseFromCodedStream(&coded_input);

  close(fd);
  return success;
}
bool loadNetParams(const std::string &param_file, NetParameter *param) {
  return ReadProtoFromTextFile(param_file, param);
}
std::string locateFile(const std::string &network, const std::string &input) {
  return network + "/" + input;
}
}  // namespace inference
}  // namespace perception
}  // namespace apollo
