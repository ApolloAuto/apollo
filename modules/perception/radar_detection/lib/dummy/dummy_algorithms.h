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

#include "cyber/common/macros.h"
#include "modules/perception/radar_detection/interface/base_detector.h"
#include "modules/perception/radar_detection/interface/base_preprocessor.h"
#include "modules/perception/radar_detection/interface/base_roi_filter.h"

namespace apollo {
namespace perception {
namespace radar {

class DummyPreprocessor : public BasePreprocessor {
 public:
  DummyPreprocessor() = default;
  virtual ~DummyPreprocessor() = default;

  /**
   * @brief Dummy Init
   *
   * @param options
   * @return true
   * @return false
   */
  bool Init(const PreprocessorInitOptions& options) override;

  /**
   * @brief Dummy preprocess
   *
   * @param raw_obstacles
   * @param options
   * @param corrected_obstacles
   * @return true
   * @return false
   */
  bool Preprocess(const drivers::ContiRadar& raw_obstacles,
                  const PreprocessorOptions& options,
                  drivers::ContiRadar* corrected_obstacles) override;

  /**
   * @brief The name of the DummyPreprocessor
   *
   * @return std::string
   */
  std::string Name() const override;

 private:
  DISALLOW_COPY_AND_ASSIGN(DummyPreprocessor);
};

class DummyDetector : public BaseDetector {
 public:
  DummyDetector() = default;
  virtual ~DummyDetector() = default;

  /**
   * @brief Dummy Init
   *
   * @param options
   * @return true
   * @return false
   */
  bool Init(const DetectorInitOptions& options) override;

  /**
   * @brief Dummy detect
   *
   * @param corrected_obstacles
   * @param options
   * @param detected_frame
   * @return true
   * @return false
   */
  bool Detect(const drivers::ContiRadar& corrected_obstacles,
              const DetectorOptions& options,
              base::FramePtr detected_frame) override;

  /**
   * @brief The name of the DummyDetector
   *
   * @return std::string
   */
  std::string Name() const override;

 private:
  void ContiObs2Frame(const drivers::ContiRadar& corrected_obstacles,
                      base::FramePtr radar_frame);

  DISALLOW_COPY_AND_ASSIGN(DummyDetector);
};

class DummyRoiFilter : public BaseRoiFilter {
 public:
  DummyRoiFilter() = default;
  virtual ~DummyRoiFilter() = default;

  /**
   * @brief Dummy Init
   *
   * @param options
   * @return true
   * @return false
   */
  bool Init(const RoiFilterInitOptions &options) override;

  /**
   * @brief Dummy roi filter
   *
   * @param options
   * @param radar_frame
   * @return true
   * @return false
   */
  bool RoiFilter(const RoiFilterOptions& options,
                 base::FramePtr radar_frame) override;

  /**
   * @brief The name of the DummyRoiFilter
   *
   * @return std::string
   */
  std::string Name() const override;

 private:
  DISALLOW_COPY_AND_ASSIGN(DummyRoiFilter);
};

}  // namespace radar
}  // namespace perception
}  // namespace apollo
