/******************************************************************************
 * Copyright 2018 The Apollo Authors. All Rights Reserved.
 *
 * Licensed under the Apache License, Version 2.0 (the License);
 * you may not use this file except in compliance with the License.
 * You may obtain a copy of the License at
 *
 * http://www.apache.org/licenses/LICENSE-2.0
 *
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an AS IS BASIS,
 * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 * See the License for the specific language governing permissions and
 * limitations under the License.
 *****************************************************************************/
#pragma once

#include <string>

namespace apollo {
namespace perception {

/**
 * @brief Get the model path by model name, search from `APOLLO_MODEL_PATH`
 *
 * @param model_name The model name
 * @return std::string The model path, empty for not found
 */
std::string GetModelPath(const std::string &model_name);

/**
 * @brief Get the model file path by model path and file name
 *
 * @param model_name The model name, use `GetModelPath` to get the model path
 * @param file_name The file name
 * @return std::string The model file path
 */
std::string GetModelFile(const std::string &model_name,
                         const std::string &file_name);

/**
 * @brief Get the perception common config path
 *
 * @param config_path The config file path, if path is relative, search from
 * `${APOLLO_CONF_PATH}/modules/perception/data/conf`
 * @return std::string The common config path
 */
std::string GetCommonConfigFile(const std::string &config_file);

/**
 * @brief Get the config path
 *
 * @param config_path The config path, if path is relative, search from
 * `${APOLLO_CONF_PATH}`
 * @return std::string The config path
 */
std::string GetConfigPath(const std::string &config_path);

std::string GetConfigFile(const std::string &config_path,
                          const std::string &config_file);

}  // namespace perception
}  // namespace apollo
