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

#include "modules/common/status/status.h"
#include "third_party/json/json.hpp"

namespace apollo {
namespace common {
namespace util {

class HttpClient {
 public:
  /**
   * @brief post a json to target url, get response as string.
   */
  static Status Post(const std::string &url, const nlohmann::json &json,
                     std::string *result = nullptr);

  /**
   * @brief post a json to target url, get response as json.
   */
  static Status Post(const std::string &url, const nlohmann::json &json,
                     nlohmann::json *result);
};

}  // namespace util
}  // namespace common
}  // namespace apollo
