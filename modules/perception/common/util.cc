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

#include "modules/perception/common/util.h"

#include "cyber/common/environment.h"
#include "cyber/common/file.h"

namespace apollo {
namespace perception {

using cyber::common::GetAbsolutePath;
using cyber::common::GetEnv;

std::string GetCommonConfigFile(const std::string &config_file) {
  std::string actual_config_path = "";
  std::string relative_config_path = config_file;
  if (!apollo::cyber::common::PathIsAbsolute(config_file)) {
    // If config_file is relative, then prepend "modules/" to it.
    relative_config_path = "modules/perception/data/conf/" + config_file;
  }
  if (!apollo::cyber::common::GetFilePathWithEnv(
          relative_config_path, "APOLLO_CONF_PATH", &actual_config_path)) {
    AERROR << "Failed to get config path of " << config_file << " please check "
           << "if config exists or `APOLLO_CONF_PATH` environment variable has "
           << "been set correctly.";
  }
  return actual_config_path;
}

std::string GetModelPath(const std::string &model_name) {
  std::string model_path = "";
  if (!apollo::cyber::common::GetFilePathWithEnv(
          model_name, "APOLLO_MODEL_PATH", &model_path)) {
    AERROR << "Failed to get model path of " << model_name << " please check "
           << "if model has been installed or `APOLLO_MODEL_PATH` environment "
           << "variable has been set correctly.";
  }
  return model_path;
}

std::string GetModelFile(const std::string &model_name,
                         const std::string &file_name) {
  std::string model_path = GetModelPath(model_name);
  if (model_path.empty()) {
    return "";
  }
  return model_path + "/" + file_name;
}

std::string GetConfigPath(const std::string &config_path) {
  std::string actual_config_path = "";
  std::string relative_config_path = config_path;
  if (!apollo::cyber::common::PathIsAbsolute(config_path)) {
    // If config_path is relative, then prepend "modules/" to it.
    relative_config_path = "modules/" + config_path;
  }
  if (!apollo::cyber::common::GetFilePathWithEnv(
          relative_config_path, "APOLLO_CONF_PATH", &actual_config_path)) {
    AERROR << "Failed to get config path of " << config_path << " please check "
           << "if config exists or `APOLLO_CONF_PATH` environment variable has "
           << "been set correctly.";
  }
  return actual_config_path;
}

std::string GetConfigFile(const std::string &config_path,
                          const std::string &config_file) {
  std::string absolute_config_path = GetConfigPath(config_path);
  std::string file_path =
      cyber::common::GetAbsolutePath(absolute_config_path, config_file);
  return file_path;
}

}  // namespace perception
}  // namespace apollo
