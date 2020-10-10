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

#include <limits>

#include "modules/perception/common/i_lib/core/i_blas.h"

namespace apollo {
namespace perception {
namespace common {

template <typename T>
inline void IGetPointcloudsDimWBound(const T *threeds, int n, int start_offset,
                                     int element_size, T *dim_min_x,
                                     T *dim_max_x, T *dim_min_y, T *dim_max_y,
                                     T *dim_min_z, T *dim_max_z, T bound_min_x,
                                     T bound_max_x, T bound_min_y,
                                     T bound_max_y, T bound_min_z,
                                     T bound_max_z) {
  int i;
  T x, y, z;

  *dim_min_x = *dim_min_y = *dim_min_z = std::numeric_limits<T>::max() / 2;
  *dim_max_x = *dim_max_y = *dim_max_z = -(std::numeric_limits<T>::max() / 2);
  const T *cptr = threeds + start_offset;
  for (i = 0; i < n; i++) {
    x = cptr[0];
    y = cptr[1];
    z = cptr[2];
    cptr += element_size;

    if (x < bound_min_x || x > bound_max_x || y < bound_min_y ||
        y > bound_max_y || z < bound_min_z || z > bound_max_z) {
      continue;
    } else {
      *dim_min_x = IMin(dim_min_x, x);
      *dim_max_x = IMax(dim_max_x, x);
      *dim_min_y = IMin(dim_min_y, y);
      *dim_max_y = IMax(dim_max_y, y);
      *dim_min_z = IMin(dim_min_z, z);
      *dim_max_z = IMax(dim_max_z, z);
    }
  }
}

}  // namespace common
}  // namespace perception
}  // namespace apollo
