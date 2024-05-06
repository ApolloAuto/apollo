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

#include <boost/thread/shared_mutex.hpp>

#include "cyber/common/macros.h"
#include "modules/common/util/future.h"
#include "modules/dreamview/backend/common/fuel_monitor/fuel_monitor.h"

/**
 * @namespace apollo::dreamview
 * @brief apollo::dreamview
 */
namespace apollo {
namespace dreamview {

using FuelMonitorMap =
    std::unordered_map<std::string, std::unique_ptr<FuelMonitor>>;

class FuelMonitorManager {
 public:
  void RegisterFuelMonitor(const std::string& mode,
                           std::unique_ptr<FuelMonitor>&& fuel_monitor);

  void SetCurrentMode(const std::string& mode);

  // Getter
  FuelMonitorMap* GetCurrentMonitors();

 private:
  std::unordered_map<std::string, FuelMonitorMap> monitors_;
  FuelMonitorMap* current_monitors_ = nullptr;
  std::string current_mode_;
  // Mutex to protect concurrent access to current_progress_json_.
  // NOTE: Use boost until we have std version of rwlock support.
  boost::shared_mutex mutex_;

  DECLARE_SINGLETON(FuelMonitorManager)
};

}  // namespace dreamview
}  // namespace apollo
