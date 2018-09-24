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
#include "modules/perception/base/syncedmem.h"
#include <string.h>

namespace apollo {
namespace perception {
namespace base {

SyncedMemory::SyncedMemory(bool use_cuda)
    : cpu_ptr_(NULL),
      gpu_ptr_(NULL),
      size_(0),
      head_(UNINITIALIZED),
      own_cpu_data_(false),
      cpu_malloc_use_cuda_(use_cuda),
      own_gpu_data_(false),
      device_(-1) {
#ifndef PERCEPTION_CPU_ONLY
#ifdef PERCEPTION_DEBUG
  BASE_CUDA_CHECK(cudaGetDevice(&device_));
#endif
#endif
}

SyncedMemory::SyncedMemory(size_t size, bool use_cuda)
    : cpu_ptr_(NULL),
      gpu_ptr_(NULL),
      size_(size),
      head_(UNINITIALIZED),
      own_cpu_data_(false),
      cpu_malloc_use_cuda_(use_cuda),
      own_gpu_data_(false),
      device_(-1) {
#ifndef PERCEPTION_CPU_ONLY
#ifdef PERCEPTION_DEBUG
  BASE_CUDA_CHECK(cudaGetDevice(&device_));
#endif
#endif
}

SyncedMemory::~SyncedMemory() {
  check_device();
  if (cpu_ptr_ && own_cpu_data_) {
    PerceptionFreeHost(cpu_ptr_, cpu_malloc_use_cuda_);
  }

#ifndef PERCEPTION_CPU_ONLY
  if (gpu_ptr_ && own_gpu_data_) {
    BASE_CUDA_CHECK(cudaFree(gpu_ptr_));
  }
#endif  // PERCEPTION_CPU_ONLY
}

inline void SyncedMemory::to_cpu() {
  check_device();
  switch (head_) {
    case UNINITIALIZED:
      PerceptionMallocHost(&cpu_ptr_, size_, cpu_malloc_use_cuda_);
      memset(cpu_ptr_, 0, size_);
      head_ = HEAD_AT_CPU;
      own_cpu_data_ = true;
      break;
    case HEAD_AT_GPU:
#ifndef PERCEPTION_CPU_ONLY
      if (cpu_ptr_ == NULL) {
        PerceptionMallocHost(&cpu_ptr_, size_, cpu_malloc_use_cuda_);
        own_cpu_data_ = true;
      }
      BASE_CUDA_CHECK(cudaMemcpy(cpu_ptr_, gpu_ptr_, size_, cudaMemcpyDefault));
      head_ = SYNCED;
#else
      NO_GPU;
#endif
      break;
    case HEAD_AT_CPU:
    case SYNCED:
      break;
  }
}

inline void SyncedMemory::to_gpu() {
  check_device();
#ifndef PERCEPTION_CPU_ONLY
  switch (head_) {
    case UNINITIALIZED:
      BASE_CUDA_CHECK(cudaMalloc(&gpu_ptr_, size_));
      BASE_CUDA_CHECK(cudaMemset(gpu_ptr_, 0, size_));
      head_ = HEAD_AT_GPU;
      own_gpu_data_ = true;
      break;
    case HEAD_AT_CPU:
      if (gpu_ptr_ == NULL) {
        BASE_CUDA_CHECK(cudaMalloc(&gpu_ptr_, size_));
        own_gpu_data_ = true;
      }
      BASE_CUDA_CHECK(cudaMemcpy(gpu_ptr_, cpu_ptr_, size_, cudaMemcpyDefault));
      head_ = SYNCED;
      break;
    case HEAD_AT_GPU:
    case SYNCED:
      break;
  }
#else
  NO_GPU;
#endif
}

const void* SyncedMemory::cpu_data() {
  check_device();
  to_cpu();
  return (const void*)cpu_ptr_;
}

void SyncedMemory::set_cpu_data(void* data) {
  check_device();
  CHECK(data);
  if (own_cpu_data_) {
    PerceptionFreeHost(cpu_ptr_, cpu_malloc_use_cuda_);
  }
  cpu_ptr_ = data;
  head_ = HEAD_AT_CPU;
  own_cpu_data_ = false;
}

const void* SyncedMemory::gpu_data() {
  check_device();
#ifndef PERCEPTION_CPU_ONLY
  to_gpu();
  return (const void*)gpu_ptr_;
#else
  NO_GPU;
  return NULL;
#endif
}

void SyncedMemory::set_gpu_data(void* data) {
  check_device();
#ifndef PERCEPTION_CPU_ONLY
  CHECK(data);
  if (own_gpu_data_) {
    BASE_CUDA_CHECK(cudaFree(gpu_ptr_));
  }
  gpu_ptr_ = data;
  head_ = HEAD_AT_GPU;
  own_gpu_data_ = false;
#else
  NO_GPU;
#endif
}

void* SyncedMemory::mutable_cpu_data() {
  check_device();
  to_cpu();
  head_ = HEAD_AT_CPU;
  return cpu_ptr_;
}

void* SyncedMemory::mutable_gpu_data() {
  check_device();
#ifndef PERCEPTION_CPU_ONLY
  to_gpu();
  head_ = HEAD_AT_GPU;
  return gpu_ptr_;
#else
  NO_GPU;
  return NULL;
#endif
}

#ifndef PERCEPTION_CPU_ONLY
void SyncedMemory::async_gpu_push(const cudaStream_t& stream) {
  check_device();
  CHECK(head_ == HEAD_AT_CPU);
  if (gpu_ptr_ == NULL) {
    BASE_CUDA_CHECK(cudaMalloc(&gpu_ptr_, size_));
    own_gpu_data_ = true;
  }
  const cudaMemcpyKind put = cudaMemcpyHostToDevice;
  BASE_CUDA_CHECK(cudaMemcpyAsync(gpu_ptr_, cpu_ptr_, size_, put, stream));
  // Assume caller will synchronize on the stream before use
  head_ = SYNCED;
}
#endif

void SyncedMemory::check_device() {
#ifndef PERCEPTION_CPU_ONLY
#ifdef PERCEPTION_DEBUG
  int device;
  cudaGetDevice(&device);
  CHECK(device == device_);
  if (gpu_ptr_ && own_gpu_data_) {
    cudaPointerAttributes attributes;
    BASE_CUDA_CHECK(cudaPointerGetAttributes(&attributes, gpu_ptr_));
    CHECK(attributes.device == device_);
  }
#endif
#endif
}

}  // namespace base
}  // namespace perception
}  // namespace apollo
