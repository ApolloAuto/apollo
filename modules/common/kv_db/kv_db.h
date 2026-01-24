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
#include <utility>
#include <vector>

#include "modules/common/util/future.h"

/**
 * @namespace apollo::common
 * @brief apollo::common
 */
namespace apollo {
namespace common {

/**
 * @class KVDB
 *
 * @brief Lightweight key-value database to store system-wide parameters.
 *        We prefer keys like "apollo:data:commit_id".
 */
class KVDB {
 public:
  /**
   * @brief Store {key, value} to DB.
   * @return Success or not.
   */
  static bool Put(std::string_view key, std::string_view value);

  /**
   * @brief Delete a key.
   * @return Success or not.
   */
  static bool Delete(std::string_view key);

  /**
   * @brief Get value of a key.
   * @return An optional value.
   *     Use `has_value()` to check if there is non-empty value.
   *     Use `value()` to get real value.
   *     Use `value_or("")` to get existing value or fallback to default.
   */
  static std::optional<std::string> Get(std::string_view key);

  /**
   * @brief Get the tuple whose key starts with start.
   * @return An optional value.
   *     Use `has_value()` to check if there is non-empty value.
   *     Use `value()` to get real value.
   *     Use `value_or("")` to get existing value or fallback to default.
   */
  static std::vector<std::pair<std::string, std::string>> GetWithStart(
      std::string_view start);
};

}  // namespace common
}  // namespace apollo
