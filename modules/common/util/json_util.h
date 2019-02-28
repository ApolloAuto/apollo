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

#pragma once

#include <string>
#include <vector>

#include "google/protobuf/message.h"
#include "third_party/json/json.hpp"

#include "cyber/common/log.h"

namespace apollo {
namespace common {
namespace util {

class JsonUtil {
 public:
  /**
   * @brief Convert proto to a json string.
   * @return A json with two fields: {type:<json_type>, data:<proto_to_json>}.
   */
  static nlohmann::json ProtoToTypedJson(
      const std::string &json_type, const google::protobuf::Message &proto);

  /**
   * @brief Get a string value from the given json[key].
   * @return Whether the field exists and is a valid string.
   */
  static bool GetStringFromJson(const nlohmann::json &json,
                                const std::string &key, std::string *value);

  /**
   * @brief Get a number value from the given json[key].
   * @return Whether the field exists and is a valid number.
   */
  template <class T>
  static bool GetNumberFromJson(const nlohmann::json &json,
                                const std::string &key, T *value) {
    const auto iter = json.find(key);
    if (iter == json.end()) {
      AERROR << "The json has no such key: " << key;
      return false;
    }
    if (!iter->is_number()) {
      AERROR << "The value of json[" << key << "] is not a number";
      return false;
    }
    *value = *iter;
    return true;
  }

  /**
   * @brief Get a boolean value from the given json[key].
   * @return Whether the field exists and is a valid boolean.
   */
  static bool GetBooleanFromJson(const nlohmann::json &json,
                                 const std::string &key, bool *value);

  /**
   * @brief Get a string vector from the given json[key].
   * @return Whether the field exists and is a valid string vector.
   */
  static bool GetStringVectorFromJson(const nlohmann::json &json,
                                      const std::string &key,
                                      std::vector<std::string> *value);
};

}  // namespace util
}  // namespace common
}  // namespace apollo
