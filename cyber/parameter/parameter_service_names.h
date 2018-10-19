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

#ifndef CYBER_PARAMETER_PARAMETER_SERVICE_NAMES_H_
#define CYBER_PARAMETER_PARAMETER_SERVICE_NAMES_H_

#include <string>

namespace apollo {
namespace cyber {

constexpr auto SERVICE_NAME_DELIMITER = "/";
constexpr auto GET_PARAMETER_SERVICE_NAME = "get_parameter";
constexpr auto SET_PARAMETER_SERVICE_NAME = "set_parameter";
constexpr auto LIST_PARAMETERS_SERVICE_NAME = "list_parameters";

static inline std::string FixParameterServiceName(const std::string& node_name,
                                                  const char* service_name) {
  return node_name + std::string(SERVICE_NAME_DELIMITER) +
         std::string(service_name);
}

}  // namespace cyber
}  // namespace apollo

#endif  // CYBER_PARAMETER_PARAMETER_SERVICE_NAMES_H_
