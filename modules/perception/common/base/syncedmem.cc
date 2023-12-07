/******************************************************************************
COPYRIGHT

All contributions by the University of California:
Copyright (c) 2014-2017 The Regents of the University of California (Regents)
All rights reserved.

All other contributions:
Copyright (c) 2014-2017, the respective contributors
All rights reserved.

Caffe uses a shared copyright model: each contributor holds copyright over
their contributions to Caffe. The project versioning records all such
contribution and copyright details. If a contributor wants to further mark
their specific copyright on a particular contribution, they should indicate
their copyright solely in the commit message of the change when it is
committed.

LICENSE

Redistribution and use in source and binary forms, with or without
modification, are permitted provided that the following conditions are met:

1. Redistributions of source code must retain the above copyright notice, this
   list of conditions and the following disclaimer.
2. Redistributions in binary form must reproduce the above copyright notice,
   this list of conditions and the following disclaimer in the documentation
   and/or other materials provided with the distribution.

THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS" AND
ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED
WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR
ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES
(INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND
ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT
(INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS
SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.

CONTRIBUTION AGREEMENT

By contributing to the BVLC/caffe repository through pull-request, comment,
or otherwise, the contributor releases their content to the
license and copyright terms herein.
 *****************************************************************************/

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
#include "modules/perception/common/base/syncedmem.h"

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
#if USE_GPU == 1
#ifdef PERCEPTION_DEBUG
  BASE_GPU_CHECK(cudaGetDevice(&device_));
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
#if USE_GPU == 1
#ifdef PERCEPTION_DEBUG
  BASE_GPU_CHECK(cudaGetDevice(&device_));
#endif
#endif
}

SyncedMemory::~SyncedMemory() {
  check_device();
  if (cpu_ptr_ && own_cpu_data_) {
    PerceptionFreeHost(cpu_ptr_, cpu_malloc_use_cuda_);
  }

#if USE_GPU == 1
  if (gpu_ptr_ && own_gpu_data_) {
    BASE_GPU_CHECK(cudaFree(gpu_ptr_));
  }
#endif  // USE_GPU
}

inline void SyncedMemory::to_cpu() {
  check_device();
  switch (head_) {
    case UNINITIALIZED:
      PerceptionMallocHost(&cpu_ptr_, size_, cpu_malloc_use_cuda_);
      if (cpu_ptr_ == nullptr) {
        AERROR << "cpu_ptr_ is null";
        return;
      }
      memset(cpu_ptr_, 0, size_);
      head_ = HEAD_AT_CPU;
      own_cpu_data_ = true;
      break;
    case HEAD_AT_GPU:
#if USE_GPU == 1
      if (cpu_ptr_ == nullptr) {
        PerceptionMallocHost(&cpu_ptr_, size_, cpu_malloc_use_cuda_);
        own_cpu_data_ = true;
      }
      BASE_GPU_CHECK(cudaMemcpy(cpu_ptr_, gpu_ptr_, size_, cudaMemcpyDefault));
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
#if USE_GPU == 1
  switch (head_) {
    case UNINITIALIZED:
      BASE_GPU_CHECK(cudaMalloc(&gpu_ptr_, size_));
      BASE_GPU_CHECK(cudaMemset(gpu_ptr_, 0, size_));
      head_ = HEAD_AT_GPU;
      own_gpu_data_ = true;
      break;
    case HEAD_AT_CPU:
      if (gpu_ptr_ == nullptr) {
        BASE_GPU_CHECK(cudaMalloc(&gpu_ptr_, size_));
        own_gpu_data_ = true;
      }
      BASE_GPU_CHECK(cudaMemcpy(gpu_ptr_, cpu_ptr_, size_, cudaMemcpyDefault));
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
  ACHECK(data);
  if (own_cpu_data_) {
    PerceptionFreeHost(cpu_ptr_, cpu_malloc_use_cuda_);
  }
  cpu_ptr_ = data;
  head_ = HEAD_AT_CPU;
  own_cpu_data_ = false;
}

const void* SyncedMemory::gpu_data() {
  check_device();
#if USE_GPU == 1
  to_gpu();
  return (const void*)gpu_ptr_;
#else
  NO_GPU;
  return nullptr;
#endif
}

void SyncedMemory::set_gpu_data(void* data) {
  check_device();
#if USE_GPU == 1
  ACHECK(data);
  if (own_gpu_data_) {
    BASE_GPU_CHECK(cudaFree(gpu_ptr_));
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
#if USE_GPU == 1
  to_gpu();
  head_ = HEAD_AT_GPU;
  return gpu_ptr_;
#else
  NO_GPU;
  return nullptr;
#endif
}

#if USE_GPU == 1
void SyncedMemory::async_gpu_push(const cudaStream_t& stream) {
  check_device();
  CHECK_EQ(head_, HEAD_AT_CPU);
  if (gpu_ptr_ == nullptr) {
    BASE_GPU_CHECK(cudaMalloc(&gpu_ptr_, size_));
    own_gpu_data_ = true;
  }
  const cudaMemcpyKind put = cudaMemcpyHostToDevice;
  BASE_GPU_CHECK(cudaMemcpyAsync(gpu_ptr_, cpu_ptr_, size_, put, stream));
  // Assume caller will synchronize on the stream before use
  head_ = SYNCED;
}
#endif

void SyncedMemory::check_device() {
#if USE_GPU == 1
#ifdef PERCEPTION_DEBUG
  int device;
  cudaGetDevice(&device);
  CHECK_EQ(device, device_);
  if (gpu_ptr_ && own_gpu_data_) {
    cudaPointerAttributes attributes;
    BASE_GPU_CHECK(cudaPointerGetAttributes(&attributes, gpu_ptr_));
    CHECK_EQ(attributes.device, device_);
  }
#endif
#endif
}

}  // namespace base
}  // namespace perception
}  // namespace apollo
