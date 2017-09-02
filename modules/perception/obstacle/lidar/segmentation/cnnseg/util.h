/******************************************************************************
 * Copyright 2017 The Apollo Authors. All Rights Reserved.
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

#ifndef MODULES_PERCEPTION_OBSTACLE_LIDAR_SEGMENTATION_CNNSEG_UTIL_H_
#define MODULES_PERCEPTION_OBSTACLE_LIDAR_SEGMENTATION_CNNSEG_UTIL_H_

#include <string>

namespace apollo {
namespace perception {
namespace cnnseg {

inline int F2I(float val, float ori, float scale) {
  return static_cast<int>(std::floor((ori - val) * scale));
}

inline int Pc2Pixel(float in_pc, float in_range, float out_size) {
  float inv_res = 0.5 * out_size / in_range;
  return static_cast<int>(std::floor((in_range - in_pc) * inv_res));
}

inline float Pixel2Pc(int in_pixel, float in_size, float out_range) {
  float res = 2.0 * out_range / in_size;
  return out_range - (static_cast<float>(in_pixel) + 0.5f) * res;
}

}  // namespace cnnseg
}  // namespace perception
}  // namespace apollo

#endif  // MODULES_PERCEPTION_OBSTACLE_LIDAR_SEGMENTATION_CNNSEG_UTIL_H_
