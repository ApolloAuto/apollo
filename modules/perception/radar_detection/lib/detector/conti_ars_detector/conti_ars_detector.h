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

#include <memory>
#include <string>

#include "cyber/common/macros.h"
#include "modules/perception/radar_detection/interface/base_detector.h"

namespace apollo {
namespace perception {
namespace radar {

class ContiArsDetector : public BaseDetector {
 public:
  ContiArsDetector() = default;
  virtual ~ContiArsDetector() = default;

  /**
   * @brief Init ContiArsDetector config.
   *
   * @param options init options
   * @return true
   * @return false
   */
  bool Init(const DetectorInitOptions& options) override;

  /**
   * @brief Detect the objects from the corrected obstacles
   *
   * @param corrected_obstacles raw data obtained from radar driver
   * @param options detect options
   * @param detected_frame radar data frame includes detection results
   * @return true
   * @return false
   */
  bool Detect(const drivers::ContiRadar& corrected_obstacles,
              const DetectorOptions& options,
              base::FramePtr detected_frame) override;

  /**
   * @brief The name of the ContiArsDetector
   *
   * @return std::string
   */
  std::string Name() const override { return "ContiArsDetector"; }

 private:
  void RawObs2Frame(const drivers::ContiRadar& corrected_obstacles,
                    const DetectorOptions& options, base::FramePtr radar_frame);

  DISALLOW_COPY_AND_ASSIGN(ContiArsDetector);
};

}  // namespace radar
}  // namespace perception
}  // namespace apollo
