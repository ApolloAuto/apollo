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

#include "modules/dreamview/backend/common/fuel_monitor/fuel_monitor_manager.h"

#include <utility>

#include "cyber/common/log.h"

namespace apollo {
namespace dreamview {

FuelMonitorManager::FuelMonitorManager() {}

void FuelMonitorManager::RegisterFuelMonitor(
    const std::string& mode, std::unique_ptr<FuelMonitor>&& fuel_monitor) {
  const auto& class_name = fuel_monitor->GetClassName();
  if (monitors_.find(mode) != monitors_.end() &&
      monitors_[mode].find(class_name) != monitors_[mode].end()) {
    AWARN << class_name << " for mode: " << mode << " has been exist!";
  } else {
    monitors_[mode].emplace(class_name, std::move(fuel_monitor));
    AINFO << "Registered " << class_name << " for mode: " << mode;
  }
}

void FuelMonitorManager::SetCurrentMode(const std::string& mode) {
  current_mode_ = mode;
  if (monitors_.find(mode) != monitors_.end()) {
    FuelMonitorMap* new_monitors = &monitors_[mode];
    if (current_monitors_ != nullptr && current_monitors_ != new_monitors) {
      for (const auto& monitor : *current_monitors_) {
        monitor.second->Stop();
      }
    }
    {
      boost::unique_lock<boost::shared_mutex> writer_lock(mutex_);
      current_monitors_ = new_monitors;
    }
    for (const auto& monitor : *current_monitors_) {
      monitor.second->Start();
    }
  } else if (current_monitors_ != nullptr) {
    for (const auto& monitor : *current_monitors_) {
      monitor.second->Stop();
    }
  }
}

FuelMonitorMap* FuelMonitorManager::GetCurrentMonitors() {
  boost::unique_lock<boost::shared_mutex> reader_lock(mutex_);
  return current_monitors_;
}

}  // namespace dreamview
}  // namespace apollo
