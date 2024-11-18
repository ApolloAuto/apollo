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

#include <cublas_v2.h>
#include <cuda_runtime.h>

namespace apollo {
namespace perception {
namespace inference {

template <typename scalar_t>
void ModulatedDeformConvForwardCUDAKernelLauncher(
    const scalar_t* input, const scalar_t* weight, const scalar_t* bias,
    const scalar_t* offset, const scalar_t* mask, scalar_t* output,
    void* workspace, int batch, int channels, int height, int width,
    int channels_out, int kernel_w, int kernel_h, int stride_w,
    int stride_h, int pad_w, int pad_h, int dilation_w, int dilation_h,
    int group, int deformable_group, int im2col_step,
    cublasHandle_t cublas_handle, cudaStream_t stream);

}  // namespace inference
}  // namespace perception
}  // namespace apollo
