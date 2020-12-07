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

#pragma once

#include <memory>
#include <string>
#include <unordered_map>

#include "cyber/common/macros.h"
#include "modules/common/util/future.h"
#include "modules/dreamview/backend/fuel_monitor/fuel_monitor.h"

/**
 * @namespace apollo::dreamview
 * @brief apollo::dreamview
 */
namespace apollo {
namespace dreamview {

// Centralized monitor config and status manager.
class FuelMonitorManager {
 public:
  void Init();

  void RegisterFuelMonitor(std::string_view mode,
                           std::unique_ptr<FuelMonitor>&& fuel_monitor);

  void SetCurrentMode(const std::string& mode);

  // Getters
  FuelMonitor* GetMonitorOfMode(const std::string& mode);
  FuelMonitor* GetCurrentMonitor() const { return current_monitor_; }
  std::string GetCurrenrMode() const { return current_mode_; }

 private:
  std::unordered_map<std::string, std::unique_ptr<FuelMonitor>> monitors_;
  FuelMonitor* current_monitor_ = nullptr;
  std::string current_mode_;

  DECLARE_SINGLETON(FuelMonitorManager)
};

}  // namespace dreamview
}  // namespace apollo
