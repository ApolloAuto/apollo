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
#ifndef PERCEPTION_BASE_SYNCEDMEM_H_
#define PERCEPTION_BASE_SYNCEDMEM_H_

#include <cstdlib>
#include <iostream>

#include "glog/logging.h"
#include "modules/perception/base/common.h"

namespace apollo {
namespace perception {
namespace base {

inline void PerceptionMallocHost(void** ptr, size_t size, bool use_cuda) {
#ifndef PERCEPTION_CPU_ONLY
  if (use_cuda) {
    BASE_CUDA_CHECK(cudaMallocHost(ptr, size));
    return;
  }
#endif
  *ptr = malloc(size);
  CHECK(*ptr) << "host allocation of size " << size << " failed";
}

inline void PerceptionFreeHost(void* ptr, bool use_cuda) {
#ifndef PERCEPTION_CPU_ONLY
  if (use_cuda) {
    BASE_CUDA_CHECK(cudaFreeHost(ptr));
    return;
  }
#endif
  free(ptr);
}

/**
 * @brief Manages memory allocation and synchronization between the host (CPU)
 *        and device (GPU).
 *
 * TODO(dox): more thorough description.
 */
class SyncedMemory {
 public:
  explicit SyncedMemory(bool use_cuda);
  explicit SyncedMemory(size_t size, bool use_cuda);
  SyncedMemory(const SyncedMemory&) = delete;
  void operator=(const SyncedMemory&) = delete;

  ~SyncedMemory();
  const void* cpu_data();
  void set_cpu_data(void* data);
  const void* gpu_data();
  void set_gpu_data(void* data);
  void* mutable_cpu_data();
  void* mutable_gpu_data();
  enum SyncedHead { UNINITIALIZED, HEAD_AT_CPU, HEAD_AT_GPU, SYNCED };
  SyncedHead head() const { return head_; }
  void set_head(SyncedHead head) { head_ = head; }
  void set_head_gpu() { set_head(HEAD_AT_GPU); }
  void set_head_cpu() { set_head(HEAD_AT_CPU); }
  size_t size() { return size_; }

#ifndef PERCEPTION_CPU_ONLY
  void async_gpu_push(const cudaStream_t& stream);
#endif

 private:
  void check_device();

  void to_cpu();
  void to_gpu();
  void* cpu_ptr_;
  void* gpu_ptr_;
  size_t size_;
  SyncedHead head_;
  bool own_cpu_data_;
  bool cpu_malloc_use_cuda_;
  bool own_gpu_data_;
  int device_;
};  // class SyncedMemory

}  // namespace base
}  // namespace perception
}  // namespace apollo

#endif  // PERCEPTION_BASE_SYNCEDMEM_H_
