/******************************************************************************
 * Copyright 2022 The Apollo Authors. All Rights Reserved.
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

#include <vector>

#include "modules/perception/common/base/frame.h"
#include "modules/perception/common/base/object.h"
#include "modules/perception/multi_sensor_fusion/base/scene.h"
#include "modules/perception/multi_sensor_fusion/base/sensor_frame.h"

namespace apollo {
namespace perception {
namespace fusion {

struct FusionFrame {
  /// @brief Raw sensor input information
  base::FramePtr frame;

  /// @brief Converted sensor input information
  std::vector<SensorFramePtr> sensor_frames;

  /// @brief Obstacle information after fusion
  std::vector<base::ObjectPtr> fused_objects;

  /// @brief A container holds obstacles and their tracking information
  ScenePtr scene_ptr;
};

}  // namespace fusion
}  // namespace perception
}  // namespace apollo
