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

/**
 * @file
 * @brief Some string util functions.
 */

#pragma once

#include <memory>
#include <string>

#include "absl/strings/str_cat.h"
#include "google/protobuf/message.h"

#include "cyber/common/file.h"
#include "cyber/time/clock.h"

/**
 * @namespace apollo::common::util
 * @brief apollo::common::util
 */
namespace apollo {
namespace common {
namespace util {

template <typename T, typename std::enable_if<
                          std::is_base_of<google::protobuf::Message, T>::value,
                          int>::type = 0>
static void FillHeader(const std::string& module_name, T* msg) {
  static std::atomic<uint64_t> sequence_num = {0};
  auto* header = msg->mutable_header();
  double timestamp = ::apollo::cyber::Clock::NowInSeconds();
  header->set_module_name(module_name);
  header->set_timestamp_sec(timestamp);
  header->set_sequence_num(
      static_cast<unsigned int>(sequence_num.fetch_add(1)));
}

template <typename T, typename std::enable_if<
                          std::is_base_of<google::protobuf::Message, T>::value,
                          int>::type = 0>
bool DumpMessage(const std::shared_ptr<T>& msg,
                 const std::string& dump_dir = "/tmp") {
  if (!msg) {
    AWARN << "Message to be dumped is nullptr!";
  }

  auto type_name = T::descriptor()->full_name();
  std::string dump_path = dump_dir + "/" + type_name;
  if (!cyber::common::DirectoryExists(dump_path)) {
    if (!cyber::common::EnsureDirectory(dump_path)) {
      AERROR << "Cannot enable dumping for '" << type_name
             << "' because the path " << dump_path
             << " cannot be created or is not a directory.";
      return false;
    }
  }

  auto sequence_num = msg->header().sequence_num();
  return cyber::common::SetProtoToASCIIFile(
      *msg, absl::StrCat(dump_path, "/", sequence_num, ".pb.txt"));
}

inline size_t MessageFingerprint(const google::protobuf::Message& message) {
  static std::hash<std::string> hash_fn;
  std::string proto_bytes;
  message.SerializeToString(&proto_bytes);
  return hash_fn(proto_bytes);
}

}  // namespace util
}  // namespace common
}  // namespace apollo
