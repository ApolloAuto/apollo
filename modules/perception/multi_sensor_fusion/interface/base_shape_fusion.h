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
#include "modules/perception/common/lib/registerer/registerer.h"
#include "modules/perception/multi_sensor_fusion/base/base_forward_declaration.h"
#include "modules/perception/multi_sensor_fusion/base/scene.h"
#include "modules/perception/multi_sensor_fusion/base/sensor_frame.h"

namespace apollo {
namespace perception {
namespace fusion {

class BaseShapeFusion {
 public:
  explicit BaseShapeFusion(TrackPtr track) : track_ref_(track) {}
  virtual ~BaseShapeFusion() = default;

  /**
   * @brief Init base data association config
   *
   * @return true
   * @return false
   */
  virtual bool Init() = 0;

  /**
   * @brief update track shape state with measurement
   *
   * @param measurement
   * @param target_timestamp
   */
  virtual void UpdateWithMeasurement(const SensorObjectPtr measurement,
                                     double target_timestamp) = 0;

  /**
   * @brief update track shape state with measurement
   *
   * @param sensor_id
   * @param measurement_timestamp
   * @param target_timestamp
   */
  virtual void UpdateWithoutMeasurement(const std::string& sensor_id,
                                        double measurement_timestamp,
                                        double target_timestamp) = 0;

  /**
   * @brief The name of BaseShapeFusion
   *
   * @return std::string
   */
  virtual std::string Name() const = 0;

 protected:
  TrackPtr track_ref_;

  DISALLOW_COPY_AND_ASSIGN(BaseShapeFusion);
};

}  // namespace fusion
}  // namespace perception
}  // namespace apollo
