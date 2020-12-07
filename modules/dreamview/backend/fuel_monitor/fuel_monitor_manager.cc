/******************************************************************************
 * Copyright 2020 The Apollo Authors. All Rights Reserved.
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

#include "modules/dreamview/backend/fuel_monitor/fuel_monitor_manager.h"

#include <utility>

#include "cyber/common/log.h"

namespace apollo {
namespace dreamview {

FuelMonitorManager::FuelMonitorManager() {}

void FuelMonitorManager::RegisterFuelMonitor(
    std::string_view mode, std::unique_ptr<FuelMonitor>&& fuel_monitor) {
  if (monitors_.find(mode.data()) != monitors_.end()) {
    AWARN << "FuelMonitor for mode: " << mode << " has been exist!";
    return;
  }
  monitors_.emplace(mode, std::move(fuel_monitor));
}

FuelMonitor* FuelMonitorManager::GetMonitorOfMode(const std::string& mode) {
  if (monitors_.find(mode) != monitors_.end()) {
    return monitors_[mode].get();
  } else {
    return nullptr;
  }
}

void FuelMonitorManager::SetCurrentMode(const std::string& mode) {
  current_mode_ = mode;
  if (monitors_.find(mode) != monitors_.end()) {
    FuelMonitor* new_monitor = monitors_[mode].get();
    if (current_monitor_ != nullptr && current_monitor_ != new_monitor) {
      current_monitor_->Stop();
    }
    current_monitor_ = new_monitor;
    current_monitor_->Start();
  } else {
    if (current_monitor_ != nullptr) {
      current_monitor_->Stop();
    }
  }
}

}  // namespace dreamview
}  // namespace apollo
