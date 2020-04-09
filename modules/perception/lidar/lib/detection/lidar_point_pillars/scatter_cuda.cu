/******************************************************************************
 * Copyright 2020 The Apollo Authors. All Rights Reserved.
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

/*
 * Copyright 2018-2019 Autoware Foundation. All rights reserved.
 *
 * Licensed under the Apache License, Version 2.0 (the "License");
 * you may not use this file except in compliance with the License.
 * You may obtain a copy of the License at
 *
 *     http://www.apache.org/licenses/LICENSE-2.0
 *
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS,
 * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 * See the License for the specific language governing permissions and
 * limitations under the License.
 */

// headers in local files
#include "modules/perception/lidar/lib/detection/lidar_point_pillars/scatter_cuda.h"

namespace apollo {
namespace perception {
namespace lidar {

__global__ void scatter_kernel(int *x_coors, int *y_coors, float *pfe_output,
                               float *scattered_feature,
                               const int MAX_NUM_PILLARS_,
                               const int GRID_X_SIZE, const int GRID_Y_SIZE) {
  int i_pillar = blockIdx.x;
  int i_feature = threadIdx.x;
  int x_ind = x_coors[i_pillar];
  int y_ind = y_coors[i_pillar];
  float feature = pfe_output[i_feature * MAX_NUM_PILLARS_ + i_pillar];
  scattered_feature[i_feature * GRID_Y_SIZE * GRID_X_SIZE +
                    y_ind * GRID_X_SIZE + x_ind] = feature;
}

ScatterCuda::ScatterCuda(const int NUM_THREADS, const int MAX_NUM_PILLARS,
                         const int GRID_X_SIZE, const int GRID_Y_SIZE)
    : NUM_THREADS_(NUM_THREADS),
      MAX_NUM_PILLARS_(MAX_NUM_PILLARS),
      GRID_X_SIZE_(GRID_X_SIZE),
      GRID_Y_SIZE_(GRID_Y_SIZE) {}

void ScatterCuda::doScatterCuda(const int pillar_count, int *x_coors,
                                int *y_coors, float *pfe_output,
                                float *scattered_feature) {
  scatter_kernel<<<pillar_count, NUM_THREADS_>>>(
      x_coors, y_coors, pfe_output, scattered_feature, MAX_NUM_PILLARS_,
      GRID_X_SIZE_, GRID_Y_SIZE_);
}

}  // namespace lidar
}  // namespace perception
}  // namespace apollo
