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

#include "modules/dreamview_plus/backend/updater/updater_manager.h"

namespace apollo {

namespace dreamview {

bool UpdaterManager::Start(const std::string& path_name,
                           const double& time_interval_ms,
                           const std::string& channel_name,
                           nlohmann::json* subscribe_param) {
  auto iter = updater_map_.find(path_name);
  if (iter == updater_map_.end()) {
    AERROR << "Related data updater for " << path_name
           << " is not exists, failed to start!";
    return false;
  }
  UpdaterBase* updater = iter->second;
  updater->StartStream(time_interval_ms, channel_name, subscribe_param);
  return true;
}

bool UpdaterManager::Stop(const std::string& path_name,
                          const std::string& channel_name) {
  auto iter = updater_map_.find(path_name);
  if (iter == updater_map_.end()) {
    AERROR << "Related data updater for " << path_name
           << " is not exists,failed to stop!";
    return false;
  }
  UpdaterBase* updater = iter->second;
  updater->StopStream(channel_name);
  return true;
}

void UpdaterManager::RegisterUpdater(std::string path_name,
                                     UpdaterBase* updater) {
  updater_map_[path_name] = updater;
}

UpdaterBase* UpdaterManager::GetUpdater(const std::string& path_name) {
  if (updater_map_.find(path_name) == updater_map_.end()) {
    return nullptr;
  }
  return updater_map_[path_name];
}
}  // namespace dreamview
}  // namespace apollo
