/******************************************************************************
 * Copyright 2021 The Apollo Authors. All Rights Reserved.
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
 * @file v2x_proxy_gflags.cc
 * @brief The gflags used by v2x proxy module
 */
#pragma once

#include <string>

#include <google/protobuf/util/json_util.h>

#include "nlohmann/json.hpp"

#include "cyber/common/log.h"

namespace apollo {
namespace v2x {
namespace JsonParse {

using google::protobuf::util::JsonStringToMessage;
using google::protobuf::util::MessageToJsonString;
using Json = nlohmann::json;

template <typename T>
bool toProto(std::string data, T* pb_msg, std::string descriptor) {
  Json json;
  try {
    json = Json::parse(data.begin(), data.end());
  } catch (const std::exception& e) {
    AERROR << "Failed to parse JSON data: " << e.what();
    return false;
  }
  if (json.find(descriptor) == json.end()) {
    AINFO << "Received JSON data without type field: ";
    return false;
  }
  auto json_field = json[descriptor];
  auto status = JsonStringToMessage(json_field.dump(), pb_msg);
  if (!status.ok()) {
    AERROR << "Can not parse spat  " << status.ToString();
    return false;
  }
  return true;
}

template <typename T>
bool toJsonString(const T& pb_message, std::string* json_str) {
  MessageToJsonString(pb_message, json_str);
  return true;
}

}  // namespace JsonParse
}  // namespace v2x
}  // namespace apollo
