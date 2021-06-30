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

enum class MetaType {
  META_UNKNOWN = 0,
  META_SMALLMOT = 1,
  META_BIGMOT = 2,
  META_NONMOT = 3,
  META_PEDESTRIAN = 4,
  MAX_META_TYPE
};

const double kPI = 3.1415926535897932384626433832795;

inline int F2I(float val, float ori, float scale) {
  return static_cast<int>(std::floor((ori - val) * scale));
}

// for axis rotated case
inline void GroupPc2Pixel(float pc_x, float pc_y, float scale, float range,
                          int* x, int* y) {
  float fx = (range - (0.707107f * (pc_x + pc_y))) * scale;
  float fy = (range - (0.707107f * (pc_x - pc_y))) * scale;
  *x = fx < 0 ? -1 : static_cast<int>(fx);
  *y = fy < 0 ? -1 : static_cast<int>(fy);
}

// for axis aligned case
inline int Pc2Pixel(float in_pc, float in_range, float out_size) {
  float inv_res = 0.5f * out_size / in_range;
  return static_cast<int>(std::floor((in_range - in_pc) * inv_res));
}

inline float Pixel2Pc(int in_pixel, float in_size, float out_range) {
  float res = 2.0f * out_range / in_size;
  return out_range - (static_cast<float>(in_pixel) + 0.5f) * res;
}

}  // namespace lidar
}  // namespace perception
}  // namespace apollo
