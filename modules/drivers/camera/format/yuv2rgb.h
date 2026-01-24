/******************************************************************************
 * Copyright 2023 The Apollo Authors. All Rights Reserved.
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

#ifndef __YUV2RGB_H__
#define __YUV2RGB_H__

#include <cuda_runtime.h>

#include "cyber/common/log.h"

namespace apollo {
namespace drivers {
namespace camera {

class CudaConvertHandler {
public:
    CudaConvertHandler(unsigned int width, unsigned int height);

    CudaConvertHandler() {}

    ~CudaConvertHandler();

    bool Init();

    void Process(unsigned char* src, unsigned char* dst, bool zero_copy=false);

    bool Destory();

private:
    unsigned int width_;
    unsigned int height_;

    unsigned int process_unit_len_ = 4;
    unsigned int width_factor_ = 2;
    unsigned int block_width_ = 12;
    unsigned int block_height_ = 10;

    dim3 block_size_;
    dim3 grid_size_;

    unsigned char* d_src_ = nullptr;

    size_t plane_size_;

    cudaStream_t stream_;
};

/**
 * @brief floating conversion of YUV to RGB implemented with cuda
 * @param src Pointer to image with yuv format
 * @param dst Pointer to converted image
 * @param width width of image
 * @param height height of image
 */
void cuda_convert_yuv_to_rgb(unsigned char* src, unsigned char* dst, unsigned int width, unsigned int height);

}  // namespace camera
}  // namespace drivers
}  // namespace apollo

#endif
