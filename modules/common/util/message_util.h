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

#ifndef MODULES_COMMON_UTIL_MESSAGE_UTIL_H_
#define MODULES_COMMON_UTIL_MESSAGE_UTIL_H_

#include <memory>
#include <string>

#include "google/protobuf/message.h"
#include "modules/common/time/time.h"
#include "modules/common/util/file.h"

/**
 * @namespace apollo::common::util
 * @brief apollo::common::util
 */
namespace apollo {
namespace common {
namespace util {

// Expose some useful utils from protobuf.
using ::google::protobuf::Message;

template <typename T, typename std::enable_if<
                          std::is_base_of<Message, T>::value, int>::type = 0>
static void FillHeader(const std::string& module_name, T* msg) {
  static std::atomic<uint64_t> sequence_num = {0};
  auto* header = msg->mutable_header();
  double timestamp = apollo::common::time::Clock::NowInSeconds();
  header->set_module_name(module_name);
  header->set_timestamp_sec(timestamp);
  header->set_sequence_num(sequence_num.fetch_add(1));
}

template <typename T, typename std::enable_if<
                          !std::is_base_of<Message, T>::value, int>::type = 0>
static bool DumpMessage(const std::shared_ptr<T>& msg,
                        const std::string& dump_dir = "/tmp") {
  return true;
}

template <typename T, typename std::enable_if<
                          !std::is_base_of<Message, T>::value, int>::type = 0>
static bool DumpMessage(const T& msg, const std::string& dump_dir = "/tmp") {
  return true;
}

template <typename T, typename std::enable_if<
                          std::is_base_of<Message, T>::value, int>::type = 0>
bool DumpMessage(const std::shared_ptr<T>& msg,
                 const std::string& dump_dir = "/tmp") {
  auto type_name = T::descriptor()->full_name();
  std::string dump_path = dump_dir + "/" + type_name;
  if (!DirectoryExists(dump_path)) {
    if (!EnsureDirectory(dump_path)) {
      AERROR << "Cannot enable dumping for '" << type_name
             << "' because the path " << dump_path
             << " cannot be created or is not a directory.";
      return false;
    }
  }

  auto sequence_num = msg->header().sequence_num();
  return util::SetProtoToASCIIFile(
      *msg, util::StrCat(dump_path, "/", sequence_num, ".pb.txt"));
}

// template <typename T, typename std::enable_if<
//                           std::is_base_of<Message, T>::value, int>::type = 0>
// double GetDelaySec(const std::shared_ptr<T>& msg) {
//   auto now = apollo::common::time::Clock::NowInSeconds();
//   return now - msg->header().timestamp_sec();

}  // namespace util
}  // namespace common
}  // namespace apollo

#endif  // MODULES_COMMON_UTIL_MESSAGE_UTIL_H_
