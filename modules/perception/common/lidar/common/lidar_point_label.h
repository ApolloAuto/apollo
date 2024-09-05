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

namespace apollo {
namespace perception {
namespace lidar {

enum class LidarPointLabel {
  UNKNOWN = 0,
  ROI = 1,
  GROUND = 2,
  OBJECT = 3,
  MAX_LABEL,
};  // enum class LidarPointLabel

enum class PointSemanticLabel {
  UNKNOWN = 0,
  IGNORE = 1,
  GROUND = 2,
  OBJECT = 3,
  CURB = 4,
  VEGETATION = 5,
  FENCE = 6,
  NOISE = 7,
  WALL = 8,
  MAX_LABEL,
};  // enum class PointSemanticLabel

enum class PointMotionLabel {
  UNKNOWN = 0,
  MOVING = 1,
  STATIONARY = 2,
  MAX_LABEL,
};  // enum class PointMotionLabel

// save semantic label in last four bits
inline void SetSemanticLabel(PointSemanticLabel label, uint8_t* value) {
    *value &= 240;  // 240: 11110000
    *value |= static_cast<uint8_t>(label);
}

inline PointSemanticLabel GetSemanticLabel(uint8_t value) {
    return static_cast<PointSemanticLabel>(value & 15);
}

inline bool IsSemanticLabelEqual(PointSemanticLabel label, uint8_t value) {
    return (value & 15) == static_cast<uint8_t>(label);
}

// save motion label in first four bits
inline void SetMotionLabel(PointMotionLabel label, uint8_t* value) {
    *value &= 15;  // 15: 00001111
    *value = (static_cast<uint8_t>(label) << 4) | (*value);
}

inline PointMotionLabel GetMotionLabel(uint8_t value) {
    return static_cast<PointMotionLabel>(value >> 4);
}

inline bool IsMotionLabelEqual(PointMotionLabel label, uint8_t value) {
    return (value >> 4) == static_cast<uint8_t>(label);
}

}  // namespace lidar
}  // namespace perception
}  // namespace apollo
