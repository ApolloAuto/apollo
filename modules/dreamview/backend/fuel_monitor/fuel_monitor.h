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

/**
 * @file
 */

#pragma once

#include <memory>
#include <string>

#include "nlohmann/json.hpp"

/**
 * @namespace apollo::dreamview
 * @brief apollo::dreamview
 */
namespace apollo {
namespace dreamview {

/**
 * @class FuelMonitor
 * @brief A base class that monitor progress for Fuel client
 */
class FuelMonitor {
 public:
  /**
   * @brief Constructor of FuelMonitor.
   */
  explicit FuelMonitor(const std::string& name) : class_name_(name) {}
  virtual ~FuelMonitor() = default;

  bool IsEnabled() const { return enabled_; }

  /**
   * @brief start monitoring
   */
  virtual void Start() = 0;

  /**
   * @brief stop monitoring
   */
  virtual void Stop() = 0;

  /**
   * @brief get class name
   *
   */
  std::string GetClassName() const { return class_name_; }

  /**
   * @brief restart monitoring
   */
  void Restart() {
    Stop();
    Start();
  }

  /**
   * @brief return current progress of as json
   */
  virtual nlohmann::json GetProgressAsJson() = 0;

 protected:
  // Whether the fuel monitor is enabled.
  bool enabled_ = false;
  std::string class_name_;
};

}  // namespace dreamview
}  // namespace apollo
