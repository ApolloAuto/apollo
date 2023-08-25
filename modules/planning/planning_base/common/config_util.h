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

#pragma once

#include <algorithm>
#include <string>

#include "cyber/common/file.h"

namespace apollo {
namespace planning {

class ConfigUtil {
 public:
  /**
   * @brief Tranform the name to part of path. The path must be all in lower
   * cases.
   *
   * @param name The name to be transformed.
   * @return The transformed name in lower cases.
   */
  static std::string TransformToPathName(const std::string& name);

  /**
   * @brief Given the class name of planning module, combine the namespace
   * "apollo::planning::" with it to create an instance of the class.
   *
   * @param class_name The class name without namespace.
   * @return The planning class name with namespace.
   */
  static std::string GetFullPlanningClassName(const std::string& class_name);

  /**
   * @brief Load the config and merge the default and user defined config.
   *
   * @param default_config_path The default config path, if the parameter is not
   * defined in config_path, use that in this file as default.
   * @param config_path The config path
   * @param config The output config data.
   * @return Load result, return true for success.
   */
  template <typename T>
  static bool LoadMergedConfig(const std::string& default_config_path,
                               const std::string& config_path, T* config);

  /**
   * @brief Load the config and override the default config by the user defined
   * one. If the user defined config doesn't exit, use the default.
   *
   * @param default_config_path The default config path, if the parameter is not
   * defined in config_path, use that in this file as default.
   * @param config_path The config path
   * @param config The output config data.
   * @return Load result, return true for success.
   */
  template <typename T>
  static bool LoadOverridedConfig(const std::string& default_config_path,
                                  const std::string& config_path, T* config);
};

template <typename T>
bool ConfigUtil::LoadMergedConfig(const std::string& default_config_path,
                                  const std::string& config_path, T* config) {
  CHECK_NOTNULL(config);
  // Get the default config, will be merged by the user defined
  // "spcific_config".
  if (!apollo::cyber::common::LoadConfig<T>(default_config_path, config)) {
    AERROR << "Failed to load default config file:" << default_config_path;
  }
  // Get the user defined "spcific_config".
  T spcific_config;
  if (!apollo::cyber::common::LoadConfig<T>(config_path, &spcific_config)) {
    AWARN << "can not load user defined config file[" << config_path
          << "], use default config instead";
    return true;
  }
  config->MergeFrom(spcific_config);
  return true;
}

template <typename T>
bool ConfigUtil::LoadOverridedConfig(const std::string& default_config_path,
                                     const std::string& config_path,
                                     T* config) {
  CHECK_NOTNULL(config);
  // Get the user defined "spcific_config".
  if (apollo::cyber::common::GetProtoFromFile(config_path, config)) {
    return true;
  }
  // Get the default config.
  if (!apollo::cyber::common::GetProtoFromFile(default_config_path, config)) {
    AERROR << "Failed to load config file using the default config:"
           << default_config_path;
    return false;
  }
  return true;
}

}  // namespace planning
}  // namespace apollo
