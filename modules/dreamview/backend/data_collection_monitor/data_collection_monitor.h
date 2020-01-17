/******************************************************************************
 * Copyright 2019 The Apollo Authors. All Rights Reserved.
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

#include <boost/thread/shared_mutex.hpp>
#include <memory>
#include <string>
#include <unordered_map>
#include <vector>

#include "gtest/gtest_prod.h"
#include "third_party/json/json.hpp"

#include "cyber/cyber.h"
#include "modules/canbus/proto/chassis.pb.h"
#include "modules/dreamview/proto/data_collection_table.pb.h"

/**
 * @namespace apollo::dreamview
 * @brief apollo::dreamview
 */
namespace apollo {
namespace dreamview {

typedef std::vector<Range> Category;

/**
 * @class DataCollectionMonitor
 * @brief A module that monitor data collection progress for calibration
 * purpose.
 */
class DataCollectionMonitor {
 public:
  /**
   * @brief Constructor of DataCollectionMonitor.
   */
  DataCollectionMonitor();
  ~DataCollectionMonitor();

  bool IsEnabled() const { return enabled_; }

  /**
   * @brief start monitoring collection progress
   */
  void Start();

  /**
   * @brief stop monitoring collection progress
   */
  void Stop();

  /**
   * @brief restart monitoring collection progress
   */
  void Restart();

  /**
   * @brief return collection progress of categories and overall as json
   */
  nlohmann::json GetProgressAsJson();

 private:
  void InitReaders();
  void LoadConfiguration();
  void ConstructCategories();
  void ConstructCategoriesHelper(const std::string& scenario_name,
                                 const Scenario& scenario, int feature_idx,
                                 std::string current_category_name,
                                 const Category& current_category);
  void OnChassis(const std::shared_ptr<apollo::canbus::Chassis>& chassis);
  bool IsCompliedWithCriteria(
      const std::shared_ptr<apollo::canbus::Chassis>& chassis,
      const Category& category);

  std::unique_ptr<cyber::Node> node_;

  // Whether the calibration monitor is enabled.
  bool enabled_ = false;

  // The table defines data collection requirements for calibration
  DataCollectionTable data_collection_table_;

  // A map from scenario to its categories. Categories are collections
  // of ranges from all possible combination of Feature x Range in a Scenario.
  std::unordered_map<std::string, std::unordered_map<std::string, Category>>
      scenario_to_categories_;

  // Number of frames that has been collected for each (scenario, category)
  std::unordered_map<std::string, std::unordered_map<std::string, size_t>>
      category_frame_count_;

  // Number of consecutive frames that has been collected for each (scenario,
  // category).
  std::unordered_map<std::string, std::unordered_map<std::string, size_t>>
      category_consecutive_frame_count_;

  // Store overall and each category progress in percentage
  nlohmann::json current_progress_json_;

  // Mutex to protect concurrent access to current_progress_json_.
  // NOTE: Use boost until we have std version of rwlock support.
  boost::shared_mutex mutex_;

  FRIEND_TEST(DataCollectionMonitorTest, UpdateCollectionProgress);
  FRIEND_TEST(DataCollectionMonitorTest, ConstructCategories);
};

}  // namespace dreamview
}  // namespace apollo
