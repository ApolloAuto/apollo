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
#ifndef CYBER_TRANSPORT_SHM_ARENA_ADDRESS_ALLOCATOR_H_
#define CYBER_TRANSPORT_SHM_ARENA_ADDRESS_ALLOCATOR_H_

#include <atomic>
#include <cstdint>

#include "cyber/base/pthread_rw_lock.h"
#include "cyber/common/macros.h"

namespace apollo {
namespace cyber {
namespace transport {

struct ArenaAddressNode {
  uint64_t key_;
  uint64_t index_;
  std::atomic<uint64_t> ref_count_;
  struct ArenaAddressNode* parent_;
  struct ArenaAddressNode* left_;
  struct ArenaAddressNode* right_;
};

union ArenaAddressAllocatorMeta {
  struct {
    uint64_t version_;
    uint64_t capacity_;
    std::atomic<uint64_t> ref_count_;
    void* base_address_;
    // TODO(ALL): support multiple size
    uint64_t address_segment_size_;
    ArenaAddressNode* root_;
    uint64_t allocated_index_;
    uint64_t reclaim_stack_top_;
    // base::PthreadRWLock mutex_;
    std::atomic<bool> occupied_;
  } struct_;
  uint8_t bytes_[128];
};

class ArenaAddressAllocator {
 public:
  ArenaAddressAllocator();
  ~ArenaAddressAllocator();

  bool Init(uint64_t capacity, void* base_address,
            uint64_t address_segment_size);

  bool OpenOrCreate(uint64_t key, uint64_t size, void* base_address,
                    void** shm_address, bool* is_created);
  bool Open(uint64_t key, void* base_address, void** shm_address);

  bool OpenOrCreate(uint64_t capacity, void* base_address,
                    uint64_t address_segment_size);

  bool OpenMetaShm(uint64_t capacity, void* base_address,
                   uint64_t address_segment_size);

  bool OpenNodeShm(uint64_t capacity, void* base_address,
                   uint64_t address_segment_size);

  ArenaAddressNode* NewNode(uint64_t key);
  void ReclaimNode(ArenaAddressNode* node);
  ArenaAddressNode* FindNode(ArenaAddressNode* node, uint64_t key);
  ArenaAddressNode* FindOrInsertNode(ArenaAddressNode* node,
                                     ArenaAddressNode** node_p,
                                     ArenaAddressNode* parent, uint64_t key);
  void RemoveNode(ArenaAddressNode* node, ArenaAddressNode** node_addr,
                  uint64_t key);
  void SwapNodePosition(ArenaAddressNode* x, ArenaAddressNode** x_p,
                        ArenaAddressNode* y, ArenaAddressNode** y_p);
  uint64_t TreeHeight(ArenaAddressNode* node);
  ArenaAddressNode* TreeMax(ArenaAddressNode* node,
                            ArenaAddressNode*** node_pp);
  ArenaAddressNode* TreeMin(ArenaAddressNode* node,
                            ArenaAddressNode*** node_pp);
  int64_t TreeBalanceFactor(ArenaAddressNode* node);
  ArenaAddressNode* TreeRebalance(ArenaAddressNode* node,
                                  ArenaAddressNode** node_p);
  ArenaAddressNode* TreeRotateLeft(ArenaAddressNode* node,
                                   ArenaAddressNode** node_p);
  ArenaAddressNode* TreeRotateRight(ArenaAddressNode* node,
                                    ArenaAddressNode** node_p);
  ArenaAddressNode* TreeRotateLeftRight(ArenaAddressNode* node,
                                        ArenaAddressNode** node_p);
  ArenaAddressNode* TreeRotateRightLeft(ArenaAddressNode* node,
                                        ArenaAddressNode** node_p);
  void* Allocate(uint64_t key);
  void Deallocate(uint64_t key);

 private:
  uint64_t meta_shm_key_;
  uint64_t node_shm_key_;
  void* meta_shm_address_;
  void* node_shm_address_;
  ArenaAddressAllocatorMeta* meta_;
  ArenaAddressNode* nodes_;
  ArenaAddressNode* root_;
  uint64_t* reclaim_stack_;
  // DECLARE_SINGLETON(ArenaAddressAllocator)
};

}  // namespace transport
}  // namespace cyber
}  // namespace apollo

#endif
