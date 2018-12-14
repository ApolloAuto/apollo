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

/**
 * @file
 * @brief This file provides the declaration of the class
 * "NavigationProvider".
 */

#ifndef MODULES_TOOLS_NAVI_GENERATOR_BACKEND_UTIL_NAVIGATION_PROVIDER_H_
#define MODULES_TOOLS_NAVI_GENERATOR_BACKEND_UTIL_NAVIGATION_PROVIDER_H_

#include <string>

#include "modules/tools/navi_generator/backend/util/file_operator.h"
#include "third_party/json/json.hpp"

namespace apollo {
namespace navi_generator {
namespace util {

class NavigationProvider {
 public:
  NavigationProvider() = default;
  ~NavigationProvider() = default;

  bool GetRoutePathAsJson(const nlohmann::json& expected_route,
                          nlohmann::json* const matched_route);
};

}  // namespace util
}  // namespace navi_generator
}  // namespace apollo

#endif  // MODULES_TOOLS_NAVI_GENERATOR_BACKEND_UTIL_NAVIGATION_PROVIDER_H_
