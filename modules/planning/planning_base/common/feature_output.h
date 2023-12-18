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
 * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or
 *implied. See the License for the specific language governing
 *permissions and limitations under the License.
 *****************************************************************************/

#pragma once

#include <string>

#include "modules/planning/planning_base/proto/learning_data.pb.h"

namespace apollo {
namespace planning {

class FeatureOutput {
 public:
  /**
   * @brief Constructor; disabled
   */
  FeatureOutput() = delete;

  /**
   * @brief Close the output stream
   */
  static void Close();

  /**
   * @brief Reset
   */
  static void Clear();

  /**
   * @brief Check if output is ready
   * @return True if output is ready
   */
  static bool Ready();

  /**
   * @brief Insert a a frame of learning data
   * @param A feature in proto
   */
  static void InsertLearningDataFrame(
      const std::string& record_filename,
      const LearningDataFrame& learning_data_frame);

  static void InsertPlanningResult();

  static LearningDataFrame* GetLatestLearningDataFrame();

  /**
   * @brief Write LearningData to a file
   */
  static void WriteLearningData(const std::string& record_file);
  static void WriteRemainderiLearningData(const std::string& record_file);

  /**
   * @brief Get the size of learning_data_
   * @return The size of learning_data_
   */
  static int SizeOfLearningData();

 private:
  static LearningData learning_data_;
  static int learning_data_file_index_;
};

}  // namespace planning
}  // namespace apollo
