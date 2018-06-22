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

#ifndef MODULES_PERCEPTION_OBSTACLE_CAMERA_DETECTOR_COMMON_UTIL_H_
#define MODULES_PERCEPTION_OBSTACLE_CAMERA_DETECTOR_COMMON_UTIL_H_

#include <cuda_runtime.h>
#include <caffe/caffe.hpp>
#include <memory>

namespace apollo {
namespace perception {

#define CUDA_CHECK(condition)                                         \
  /* Code block avoids redefinition of cudaError_t error */           \
  do {                                                                \
    cudaError_t error = condition;                                    \
    CHECK_EQ(error, cudaSuccess) << " " << cudaGetErrorString(error); \
  } while (0)

void gpu_memcpy(const size_t N, const void *X, void *Y);

inline void perception_gpu_memset(const size_t N, const int alpha, void *X) {
  CUDA_CHECK(cudaMemset(X, alpha, N));
}
inline void perception_memset(const size_t N, const int alpha, void *X) {
  memset(X, alpha, N);
}

inline void PerceptionMallocHost(void **ptr, size_t size, bool *use_cuda) {
  CUDA_CHECK(cudaMallocHost(ptr, size));
  *use_cuda = true;
  return;
}

inline void PerceptionFreeHost(void *ptr, bool use_cuda) {
  if (use_cuda) {
    CUDA_CHECK(cudaFreeHost(ptr));
    return;
  }
  free(ptr);
  return;
}

int divup(int a, int b);

void resize(cv::Mat frame, caffe::Blob<float> *dst,
            std::shared_ptr<caffe::SyncedMemory> src_gpu, int start_axis);
// resize with mean and scale
void resize(cv::Mat frame, caffe::Blob<float> *dst,
            std::shared_ptr<caffe::SyncedMemory> src_gpu, int start_axis,
            const float mean_b, const float mean_g, const float mean_r,
            const float scale);

void yuyv2bgr(const uint8_t *yuv, uint8_t *rgb, const int pixel_num);
}  // namespace perception
}  // namespace apollo

#endif  // MODULES_PERCEPTION_OBSTACLE_CAMERA_DETECTOR_COMMON_UTIL_H_
