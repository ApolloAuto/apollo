/******************************************************************************
 * Copyright 2024 The Apollo Authors. All Rights Reserved.
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

#include "modules/perception/common/lidar/common/lidar_point_label.h"

namespace apollo {
namespace perception {
namespace camera {

using apollo::perception::lidar::PointSemanticLabel;

// save semantic label in last four digits
inline void SetSemanticLabel(PointSemanticLabel label, uint8_t* value) {
  *value &= 240;  // 240: 11110000
  *value |= static_cast<uint8_t>(label);
}

inline PointSemanticLabel GetSemanticLabel(uint8_t value) {
  return static_cast<PointSemanticLabel>(value & 15);
}

inline bool IsSemanticLabel(PointSemanticLabel label, uint8_t value) {
  return (value & 15) == static_cast<uint8_t>(label);
}

// save scene-flow label in first four digits
inline void SetSceneFlowLabel(PointSemanticLabel label, uint8_t* value) {
  *value &= 15;  // 15: 00001111
  *value = (static_cast<uint8_t>(label) << 4) | (*value);
}

inline PointSemanticLabel GetSceneFlowLabel(uint8_t value) {
  return static_cast<PointSemanticLabel>(value >> 4);
}

inline bool IsSceneFlowLabelEqual(PointSemanticLabel label, uint8_t value) {
  return (value >> 4) == static_cast<uint8_t>(label);
}

}  // namespace camera
}  // namespace perception
}  // namespace apollo
