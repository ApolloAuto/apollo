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

#include "modules/common/util/json_util.h"

#include "google/protobuf/util/json_util.h"

namespace apollo {
namespace common {
namespace util {
namespace {

using Json = nlohmann::json;
using google::protobuf::util::MessageToJsonString;

google::protobuf::util::JsonOptions JsonOption() {
  google::protobuf::util::JsonOptions json_option;
  json_option.always_print_primitive_fields = true;
  return json_option;
}

}  // namespace

nlohmann::json JsonUtil::ProtoToTypedJson(
    const std::string &json_type, const google::protobuf::Message &proto) {
  static const auto kJsonOption = JsonOption();
  std::string json_string;
  const auto status = MessageToJsonString(proto, &json_string, kJsonOption);
  ACHECK(status.ok()) << "Cannot convert proto to json:" << proto.DebugString();

  Json json_obj;
  json_obj["type"] = json_type;
  json_obj["data"] = Json::parse(json_string);
  return json_obj;
}

nlohmann::json JsonUtil::ProtoToJson(const google::protobuf::Message &proto) {
  static const auto kJsonOption = JsonOption();
  std::string json_string;
  const auto status = MessageToJsonString(proto, &json_string, kJsonOption);
  Json json_obj = Json::parse(json_string);
  return json_obj;
}

bool JsonUtil::GetString(const Json &json, const std::string &key,
                         std::string *value) {
  const auto iter = json.find(key);
  if (iter == json.end()) {
    AERROR << "The json has no such key: " << key;
    return false;
  }
  if (!iter->is_string()) {
    AERROR << "The value of json[" << key << "] is not a string";
    return false;
  }
  *value = *iter;
  return true;
}

bool JsonUtil::GetJsonByPath(const nlohmann::json &json,
                             const std::vector<std::string> &paths,
                             nlohmann::json *value) {
  Json upper_layer_json = json;
  for (auto &field : paths) {
    if (field.empty()) {
      AERROR << "Invalid path: " << field;
      return false;
    }
    const auto iter = upper_layer_json.find(field);
    if (iter == upper_layer_json.end()) {
      AERROR << "The json has no such key: " << field;
      return false;
    }
    if (!iter->is_object()) {
      AERROR << "Required json but not,invalid type.";
      return false;
    }
    upper_layer_json = *iter;
  }
  *value = upper_layer_json;
  return true;
}

bool JsonUtil::GetStringByPath(const Json &json, const std::string &path,
                               std::string *value) {
  std::vector<std::string> paths = absl::StrSplit(path, '.');
  std::string key = paths.back();
  paths.pop_back();
  Json upper_layer_json;
  if (!GetJsonByPath(json, paths, &upper_layer_json)) {
    return false;
  }
  return GetString(upper_layer_json, key, value);
}

bool JsonUtil::GetStringVector(const Json &json, const std::string &key,
                               std::vector<std::string> *value) {
  const auto iter = json.find(key);
  if (iter == json.end()) {
    AERROR << "The json has no such key: " << key;
    return false;
  }
  if (!iter->is_array()) {
    AERROR << "The value of json[" << key << "] is not an array";
    return false;
  }

  bool ret = true;
  value->clear();
  value->reserve(iter->size());
  for (const auto &elem : *iter) {
    // Note that we still try to get all string values though there are invalid
    // elements.
    if (!elem.is_string()) {
      AWARN << "The value of json[" << key << "] contains non-string element";
      ret = false;
    } else {
      value->push_back(elem);
    }
  }
  return ret;
}

bool JsonUtil::GetBoolean(const nlohmann::json &json, const std::string &key,
                          bool *value) {
  const auto iter = json.find(key);
  if (iter == json.end()) {
    AERROR << "The json has no such key: " << key;
    return false;
  }
  if (!iter->is_boolean()) {
    AERROR << "The value of json[" << key << "] is not a boolean";
    return false;
  }
  *value = *iter;
  return true;
}

bool JsonUtil::GetBooleanByPath(const Json &json, const std::string &path,
                                bool *value) {
  std::vector<std::string> paths = absl::StrSplit(path, '.');
  std::string key = paths.back();
  paths.pop_back();
  Json upper_layer_json;
  if (!GetJsonByPath(json, paths, &upper_layer_json)) {
    return false;
  }
  return GetBoolean(upper_layer_json, key, value);
}

}  // namespace util
}  // namespace common
}  // namespace apollo
