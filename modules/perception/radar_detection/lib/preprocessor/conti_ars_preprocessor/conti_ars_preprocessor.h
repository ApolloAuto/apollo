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
#pragma once

#include <string>
#include <unordered_map>

#include "cyber/common/macros.h"
#include "modules/perception/radar_detection/interface/base_preprocessor.h"

namespace apollo {
namespace perception {
namespace radar {

class ContiArsPreprocessor : public BasePreprocessor {
 public:
  ContiArsPreprocessor() : BasePreprocessor(), delay_time_(0.0) {}
  virtual ~ContiArsPreprocessor() {}

  /**
   * @brief Init ContiArsPreprocessor config
   *
   * @param options init options
   * @return true
   * @return false
   */
  bool Init(const PreprocessorInitOptions& options) override;

  /**
   * @brief Correct radar raw obstacles.
   *
   * @param raw_obstacles raw obstacles from radar driver.
   * @param options preprocess options
   * @param corrected_obstacles corrected radar obstacles
   * @return true
   * @return false
   */
  bool Preprocess(const drivers::ContiRadar& raw_obstacles,
                  const PreprocessorOptions& options,
                  drivers::ContiRadar* corrected_obstacles) override;

  /**
   * @brief The name of the ContiArsPreprocessor
   *
   * @return std::string
   */
  std::string Name() const override { return "ContiArsPreprocessor"; }

  /**
   * @brief Get the Delay Time object
   *
   * @return double
   */
  inline double GetDelayTime() { return delay_time_; }

 private:
  void SkipObjects(const drivers::ContiRadar& raw_obstacles,
                   drivers::ContiRadar* corrected_obstacles);
  void ExpandIds(drivers::ContiRadar* corrected_obstacles);
  void CorrectTime(drivers::ContiRadar* corrected_obstacles);
  int GetNextId();

  float delay_time_ = 0.0f;
  static int current_idx_;
  static std::unordered_map<int, int> local2global_;

  friend class ContiArsPreprocessorTest;

  DISALLOW_COPY_AND_ASSIGN(ContiArsPreprocessor);
};

}  // namespace radar
}  // namespace perception
}  // namespace apollo
