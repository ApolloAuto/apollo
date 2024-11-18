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

#include <cuda_runtime.h>

namespace apollo {
namespace perception {
namespace inference {

enum class GridSamplerInterpolation { Bilinear, Nearest };
enum class GridSamplerPadding { Zeros, Border, Reflection };

template <typename T>
void grid_sample(
    T *output, const T *input, const T *grid, int *output_dims, int *input_dims,
    int *grid_dims, int nb_dims, GridSamplerInterpolation interp,
    GridSamplerPadding padding, bool align_corners, cudaStream_t stream);

}  // namespace inference
}  // namespace perception
}  // namespace apollo
