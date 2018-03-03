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

#define MODULES_PERCEPTION_OBSTACLE_CAMERA_DETECTOR_COMMON_UTIL_H_
#include <caffe/caffe.hpp>

#include <infer.h>
#include "modules/common/log.h"

namespace apollo {
namespace perception {
namespace obstacle {
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

class SyncedMemory {
 public:
  SyncedMemory()
      : cpu_ptr_(NULL),
        gpu_ptr_(NULL),
        size_(0),
        head_(UNINITIALIZED),
        own_cpu_data_(false),
        cpu_malloc_use_cuda_(false),
        own_gpu_data_(false),
        gpu_device_(-1) {}
  explicit SyncedMemory(size_t size)
      : cpu_ptr_(NULL),
        gpu_ptr_(NULL),
        size_(size),
        head_(UNINITIALIZED),
        own_cpu_data_(false),
        cpu_malloc_use_cuda_(false),
        own_gpu_data_(false),
        gpu_device_(-1) {}

  ~SyncedMemory();

  const void *cpu_data();

  void set_cpu_data(void *data);

  const void *gpu_data();

  void set_gpu_data(void *data);

  void *mutable_cpu_data();

  void *mutable_gpu_data();

  enum SyncedHead { UNINITIALIZED, HEAD_AT_CPU, HEAD_AT_GPU, SYNCED };
  SyncedHead head() { return head_; }
  size_t size() { return size_; }

  void async_gpu_push(const cudaStream_t &stream);

 private:
  void to_cpu();

  void to_gpu();

  void *cpu_ptr_;
  void *gpu_ptr_;
  size_t size_;
  SyncedHead head_;
  bool own_cpu_data_;
  bool cpu_malloc_use_cuda_;
  bool own_gpu_data_;
  int gpu_device_;
  DISABLE_COPY_AND_ASSIGN(SyncedMemory);
};  // class SyncedMemory
int divup(int a, int b);

void resize(cv::Mat frame, anakin::Tensor<float> *dst,
            std::shared_ptr<SyncedMemory> src_gpu, int start_axis);
void resize(cv::Mat frame, caffe::Blob<float> *dst,
            std::shared_ptr<SyncedMemory> src_gpu, int start_axis);
void resize(const uchar *src, std::shared_ptr<SyncedMemory> src_gpu,
            int origin_width, int origin_height, anakin::Tensor<float> *dst,
            int start_axis);

// resize with mean and scale
void resize(cv::Mat frame, anakin::Tensor<float> *dst,
            std::shared_ptr<SyncedMemory> src_gpu, int start_axis,
            const float mean_b, const float mean_g, const float mean_r,
            const float scale);
void resize(cv::Mat frame, caffe::Blob<float> *dst,
            std::shared_ptr<SyncedMemory> src_gpu, int start_axis,
            const float mean_b, const float mean_g, const float mean_r,
            const float scale);
void resize(const uchar *src, std::shared_ptr<SyncedMemory> src_gpu,
            int origin_width, int origin_height, anakin::Tensor<float> *dst,
            int start_axis, const float mean_b, const float mean_g,
            const float mean_r, const float scale);
}  // namespace obstacle
}  // namespace perception
}  // namespace apollo

#endif  // MODULES_PERCEPTION_OBSTACLE_CAMERA_DETECTOR_COMMON_UTIL_H_
