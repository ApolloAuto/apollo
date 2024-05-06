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
#include <unordered_map>
#include <vector>

#include <boost/thread/shared_mutex.hpp>

#include "nlohmann/json.hpp"

#include "modules/dreamview/proto/preprocess_table.pb.h"

#include "cyber/cyber.h"
#include "modules/dreamview/backend/common/dreamview_gflags.h"
#include "modules/dreamview/backend/common/fuel_monitor/fuel_monitor.h"

/**
 * @namespace apollo::dreamview
 * @brief apollo::dreamview
 */
namespace apollo {
namespace dreamview {

/**
 * @class PreprocessMonitor
 * @brief A module that monitor data preprocess progress for sensor
 * calibration purpose.
 */
class PreprocessMonitor : public FuelMonitor {
 public:
  /**
   * @brief Constructor of PreprocessMonitor.
   */
  PreprocessMonitor();
  explicit PreprocessMonitor(const std::string& task_name);
  ~PreprocessMonitor();

  /**
   * @brief start monitoring preprocess progress
   */
  void Start() override;

  /**
   * @brief stop monitoring preprocess progress
   */
  void Stop() override;

  /**
   * @brief return preprocess progress as json
   */
  nlohmann::json GetProgressAsJson() override;

 private:
  void InitReaders();
  void OnProgress(const std::shared_ptr<Progress>& progress);
  void LoadConfiguration();

  PreprocessTable preprocess_table_;

  std::string task_name_ = "";
  std::unique_ptr<cyber::Node> node_;

  nlohmann::json current_status_json_;

  // Mutex to protect concurrent access to current_progress_json_.
  // NOTE: Use boost until we have std version of rwlock support.
  boost::shared_mutex mutex_;
};

}  // namespace dreamview
}  // namespace apollo
