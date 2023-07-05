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

#include "modules/drivers/lidar/velodyne/parser/util.h"

namespace apollo {
namespace drivers {
namespace velodyne {

void init_sin_cos_rot_table(float* sin_rot_table, float* cos_rot_table,
                            uint16_t rotation, float rotation_resolution) {
  for (uint16_t i = 0; i < rotation; ++i) {
    // float rotation = angles::from_degrees(rotation_resolution * i);
    float rotation =
        rotation_resolution * static_cast<float>(i * M_PI) / 180.0f;
    cos_rot_table[i] = cosf(rotation);
    sin_rot_table[i] = sinf(rotation);
  }
}

}  // namespace velodyne
}  // namespace drivers
}  // namespace apollo
