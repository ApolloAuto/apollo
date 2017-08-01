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

#include "common/utils.h"
#include "ros/ros.h"

#include <fcntl.h>
#include <google/protobuf/io/coded_stream.h>
#include <google/protobuf/io/zero_copy_stream_impl.h>

namespace apollo {
namespace routing {

using ::google::protobuf::io::FileInputStream;
using ::google::protobuf::io::ZeroCopyInputStream;
using ::google::protobuf::io::CodedInputStream;

bool FileUtils::load_protobuf_data_from_file(
    const std::string& file_path,
    ::google::protobuf::Message* const proto_data) {
  int fd = open(file_path.c_str(), O_RDONLY);
  if (fd == -1) {
    ROS_ERROR("File %s not found", file_path.c_str());
    return false;
  }
  std::unique_ptr<ZeroCopyInputStream> raw_input(new FileInputStream(fd));
  std::unique_ptr<CodedInputStream> coded_input(
      new CodedInputStream(raw_input.get()));
  coded_input->SetTotalBytesLimit(INT_MAX, 536870912);  //  0..512M..2G
  bool ret = proto_data->ParseFromCodedStream(coded_input.get());
  close(fd);
  return ret;
}

bool FileUtils::dump_protobuf_data_to_file(
    const std::string& file_path,
    const ::google::protobuf::Message* const proto_data) {
  return dump_protobuf_data_to_file(file_path, *proto_data);
}

bool FileUtils::dump_protobuf_data_to_file(
    const std::string& file_path,
    const ::google::protobuf::Message& proto_data) {
  int fd = open(file_path.c_str(), O_CREAT | O_TRUNC | O_RDWR, 0644);
  if (fd < 0 || !proto_data.SerializeToFileDescriptor(fd)) {
    ROS_ERROR("Error occured while trying to write to file: %s",
              file_path.c_str());
    return false;
  }
  if (close(fd) != 0) {
    ROS_ERROR("Error occured while close fd. Please check the file!");
    return false;
  }
  ROS_INFO("Proto data dump done!");
  return true;
}

}  // namespace routing
}  // namespace apollo
