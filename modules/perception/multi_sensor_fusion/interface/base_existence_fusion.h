/******************************************************************************
 * Copyright 2018 The Apollo Authors. All Rights Reserved.
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

#include <string>

#include "cyber/common/macros.h"
#include "modules/perception/common/lib/interface/base_init_options.h"
#include "modules/perception/common/lib/registerer/registerer.h"
#include "modules/perception/multi_sensor_fusion/base/base_forward_declaration.h"
#include "modules/perception/multi_sensor_fusion/base/scene.h"
#include "modules/perception/multi_sensor_fusion/base/sensor_frame.h"

namespace apollo {
namespace perception {
namespace fusion {

struct ExistenceFusionInitOptions : public BaseInitOptions {};

class BaseExistenceFusion {
 public:
  explicit BaseExistenceFusion(TrackPtr track) : track_ref_(track) {}
  virtual ~BaseExistenceFusion() = default;

  /**
   * @brief Init base existence fusion config
   *
   * @param options init options
   * @return true
   * @return false
   */
  static bool Init(const ExistenceFusionInitOptions &options);

  /**
   * @brief update track existence state with measurement
   *
   * @param measurement
   * @param target_timestamp
   * @param match_dist
   */
  virtual void UpdateWithMeasurement(const SensorObjectPtr measurement,
                                     double target_timestamp,
                                     double match_dist) = 0;

  /**
   * @brief update track existence state without measurement
   *
   * @param sensor_id
   * @param measurement_timestamp
   * @param target_timestamp
   * @param min_match_dist
   */
  virtual void UpdateWithoutMeasurement(const std::string& sensor_id,
                                        double measurement_timestamp,
                                        double target_timestamp,
                                        double min_match_dist) = 0;

 protected:
  TrackPtr track_ref_ = nullptr;

  DISALLOW_COPY_AND_ASSIGN(BaseExistenceFusion);
};

}  // namespace fusion
}  // namespace perception
}  // namespace apollo
