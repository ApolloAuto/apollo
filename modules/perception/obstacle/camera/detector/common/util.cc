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

#include "modules/obstacle/camera/detector/common/util.h"

namespace apollo {
namespace perception {
namespace obstacle {

SyncedMemory::~SyncedMemory() {
  if (cpu_ptr_ && own_cpu_data_) {
    PerceptionFreeHost(cpu_ptr_, cpu_malloc_use_cuda_);
  }
  if (gpu_ptr_ && own_gpu_data_) {
    int initial_device = -1;
    cudaGetDevice(&initial_device);
    if (gpu_device_ != -1) {
      CUDA_CHECK(cudaSetDevice(gpu_device_));
    }
    CUDA_CHECK(cudaFree(gpu_ptr_));
    cudaSetDevice(initial_device);
  }
}

inline void SyncedMemory::to_cpu() {
  switch (head_) {
    case UNINITIALIZED:
      PerceptionMallocHost(&cpu_ptr_, size_, &cpu_malloc_use_cuda_);
      perception_memset(size_, 0, cpu_ptr_);
      head_ = HEAD_AT_CPU;
      own_cpu_data_ = true;
      break;
    case HEAD_AT_GPU:
      if (cpu_ptr_ == NULL) {
        PerceptionMallocHost(&cpu_ptr_, size_, &cpu_malloc_use_cuda_);
        own_cpu_data_ = true;
      }
      gpu_memcpy(size_, gpu_ptr_, cpu_ptr_);
      head_ = SYNCED;
      break;
    case HEAD_AT_CPU:
    case SYNCED:
      break;
  }
}

inline void SyncedMemory::to_gpu() {
  switch (head_) {
    case UNINITIALIZED:
      CUDA_CHECK(cudaGetDevice(&gpu_device_));
      CUDA_CHECK(cudaMalloc(&gpu_ptr_, size_));
      perception_gpu_memset(size_, 0, gpu_ptr_);
      head_ = HEAD_AT_GPU;
      own_gpu_data_ = true;
      break;
    case HEAD_AT_CPU:
      if (gpu_ptr_ == NULL) {
        CUDA_CHECK(cudaGetDevice(&gpu_device_));
        CUDA_CHECK(cudaMalloc(&gpu_ptr_, size_));
        own_gpu_data_ = true;
      }
      gpu_memcpy(size_, cpu_ptr_, gpu_ptr_);
      head_ = SYNCED;
      break;
    case HEAD_AT_GPU:
    case SYNCED:
      break;
  }
}

const void *SyncedMemory::cpu_data() {
  to_cpu();
  return (const void *)cpu_ptr_;
}

void SyncedMemory::set_cpu_data(void *data) {
  if (data == nullptr) {
    return;
  }
  if (own_cpu_data_) {
    PerceptionFreeHost(cpu_ptr_, cpu_malloc_use_cuda_);
  }
  cpu_ptr_ = data;
  head_ = HEAD_AT_CPU;
  own_cpu_data_ = false;
}

const void *SyncedMemory::gpu_data() {
  to_gpu();
  return (const void *)gpu_ptr_;
}

void SyncedMemory::set_gpu_data(void *data) {
  if (data == nullptr) {
    return;
  }
  if (own_gpu_data_) {
    int initial_device = -1;
    cudaGetDevice(&initial_device);
    if (gpu_device_ != -1) {
      CUDA_CHECK(cudaSetDevice(gpu_device_));
    }
    CUDA_CHECK(cudaFree(gpu_ptr_));
    cudaSetDevice(initial_device);
  }
  gpu_ptr_ = data;
  head_ = HEAD_AT_GPU;
  own_gpu_data_ = false;
}

void *SyncedMemory::mutable_cpu_data() {
  to_cpu();
  head_ = HEAD_AT_CPU;
  return cpu_ptr_;
}

void *SyncedMemory::mutable_gpu_data() {
  to_gpu();
  head_ = HEAD_AT_GPU;
  return gpu_ptr_;
}

void SyncedMemory::async_gpu_push(const cudaStream_t &stream) {
  CHECK(head_ == HEAD_AT_CPU);
  if (gpu_ptr_ == NULL) {
    CUDA_CHECK(cudaGetDevice(&gpu_device_));
    CUDA_CHECK(cudaMalloc(&gpu_ptr_, size_));
    own_gpu_data_ = true;
  }
  const cudaMemcpyKind put = cudaMemcpyHostToDevice;
  CUDA_CHECK(cudaMemcpyAsync(gpu_ptr_, cpu_ptr_, size_, put, stream));
  // Assume caller will synchronize on the stream before use
  head_ = SYNCED;
}
}  // namespace obstacle
}  // namespace perception
}  // namespace apollo
