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
#include "cyber/transport/shm/arena_address_allocator.h"

#include <sys/ipc.h>
#include <sys/shm.h>
#include <algorithm>
#include <string>

#include "cyber/base/pthread_rw_lock.h"
#include "cyber/base/rw_lock_guard.h"

namespace apollo {
namespace cyber {
namespace transport {

ArenaAddressAllocator::ArenaAddressAllocator()
    : meta_shm_key_(
          std::hash<std::string>{}("/apollo/__arena__/__address__/__meta__")),
      node_shm_key_(
          std::hash<std::string>{}("/apollo/__arena__/__address__/__node__")),
      meta_(nullptr),
      nodes_(nullptr),
      reclaim_stack_(nullptr) {
  // TODO(all): configurable and address management for shared memory of cyber
  meta_shm_address_ = nullptr;
  // use fixed address
  node_shm_address_ = reinterpret_cast<void*>(0x7f0000000000);

  // ensure the allocator's segment size is larger
  // than the actual arena's segment size
  // otherwise the shmat will fail
  Init(1024, reinterpret_cast<void*>(
    0x710000000000), 2ULL * 1024 * 1024 * 1024);
}

ArenaAddressAllocator::~ArenaAddressAllocator() {
  if (meta_ && meta_->struct_.ref_count_.fetch_sub(1) == 0) {
    shmdt(meta_);
    shmctl(shmget(meta_shm_key_, 0, 0), IPC_RMID, nullptr);
    shmdt(nodes_);
    shmctl(shmget(node_shm_key_, 0, 0), IPC_RMID, nullptr);
  }
}

bool ArenaAddressAllocator::Init(uint64_t capacity, void* base_address,
                                 uint64_t address_segment_size) {
  if (!OpenMetaShm(capacity, base_address, address_segment_size)) {
    return false;
  }
  if (!OpenNodeShm(capacity, base_address, address_segment_size)) {
    return false;
  }
  meta_->struct_.occupied_.store(false);
  return true;
}

bool ArenaAddressAllocator::Open(uint64_t key, void* base_address,
                                 void** shm_address) {
  auto shmid = shmget(static_cast<key_t>(key), 0, 0);

  shmid = shmget(static_cast<key_t>(key), 0, 0);
  if (shmid == -1) {
    // open failed
    return false;
  }
  *shm_address = shmat(shmid, base_address, 0);
  if (*shm_address == reinterpret_cast<void*>(-1)) {
    // shmat failed
    return false;
  }
  return true;
}

bool ArenaAddressAllocator::OpenOrCreate(uint64_t key, uint64_t size,
                                         void* base_address, void** shm_address,
                                         bool* is_created) {
  auto shmid =
      shmget(static_cast<key_t>(key), size, 0644 | IPC_CREAT | IPC_EXCL);

  if (shmid == -1) {
    if (errno == EINVAL) {
      // TODO(all): recreate larger shm
    } else if (errno == EEXIST) {
      *is_created = false;
      return Open(key, base_address, shm_address);
    } else {
      return false;
    }
  }
  *shm_address = shmat(shmid, base_address, 0);
  if (*shm_address == reinterpret_cast<void*>(-1)) {
    // shmat failed
    return false;
  }
  *is_created = true;
  return true;
}

bool ArenaAddressAllocator::OpenMetaShm(uint64_t capacity, void* base_address,
                                        uint64_t address_segment_size) {
  bool is_created = false;
  bool ret = false;
  for (uint32_t retry = 0; retry < 2 && !ret;
       ++retry, ret = OpenOrCreate(
                    meta_shm_key_, sizeof(ArenaAddressAllocatorMeta),
                    meta_shm_address_, reinterpret_cast<void**>(&meta_),
                    &is_created)) {
  }
  if (!ret) {
    // create or open failed
    return false;
  }
  if (!is_created) {
    meta_->struct_.ref_count_.fetch_add(1);
  } else {
    meta_->struct_.version_ = 1;
    meta_->struct_.capacity_ = capacity;
    meta_->struct_.base_address_ = base_address;
    meta_->struct_.address_segment_size_ = address_segment_size;
    meta_->struct_.root_ = nullptr;
    meta_->struct_.allocated_index_ = 0;
    meta_->struct_.reclaim_stack_top_ = 0;
    meta_->struct_.ref_count_.store(1);
  }
  return true;
}

bool ArenaAddressAllocator::OpenNodeShm(uint64_t capacity, void* base_address,
                                        uint64_t address_segment_size) {
  uint64_t size = sizeof(ArenaAddressAllocatorMeta) +
                  sizeof(ArenaAddressNode) * (capacity + 1) +
                  sizeof(uint64_t) * capacity;
  bool is_created = false;
  bool ret = false;
  for (uint32_t retry = 0; retry < 2 && !ret;
       ++retry, ret = OpenOrCreate(node_shm_key_, size, node_shm_address_,
                                   reinterpret_cast<void**>(&nodes_),
                                   &is_created)) {
  }
  if (!ret) {
    // create or open failed
    return false;
  }

  reclaim_stack_ = reinterpret_cast<uint64_t*>(
      reinterpret_cast<uint64_t>(nodes_) + sizeof(ArenaAddressNode) * capacity);
  return true;
}

ArenaAddressNode* ArenaAddressAllocator::NewNode(uint64_t key) {
  if (meta_->struct_.reclaim_stack_top_ > 0) {
    uint64_t index = reclaim_stack_[--meta_->struct_.reclaim_stack_top_];
    nodes_[index].left_ = nullptr;
    nodes_[index].right_ = nullptr;
    nodes_[index].key_ = key;
    return &nodes_[index];
  }
  if (meta_->struct_.allocated_index_ >= meta_->struct_.capacity_) {
    // TODO(all): expand nodes
    return nullptr;
  }
  nodes_[meta_->struct_.allocated_index_].left_ = nullptr;
  nodes_[meta_->struct_.allocated_index_].right_ = nullptr;
  nodes_[meta_->struct_.allocated_index_].key_ = key;
  nodes_[meta_->struct_.allocated_index_].index_ =
      meta_->struct_.allocated_index_;
  return &nodes_[meta_->struct_.allocated_index_++];
}

void ArenaAddressAllocator::ReclaimNode(ArenaAddressNode* node) {
  nodes_[node->index_].left_ = nullptr;
  nodes_[node->index_].right_ = nullptr;
  nodes_[node->index_].parent_ = nullptr;
  reclaim_stack_[meta_->struct_.reclaim_stack_top_++] = node->index_;
}

ArenaAddressNode* ArenaAddressAllocator::FindNode(ArenaAddressNode* node,
                                                  uint64_t key) {
  if (node == nullptr) {
    return nullptr;
  }
  if (key < node->key_) {
    return FindNode(node->left_, key);
  } else if (key > node->key_) {
    return FindNode(node->right_, key);
  }
  return node;
}

ArenaAddressNode* ArenaAddressAllocator::FindOrInsertNode(
    ArenaAddressNode* node, ArenaAddressNode** node_p, ArenaAddressNode* parent,
    uint64_t key) {
  if (node == nullptr) {
    auto target = NewNode(key);
    *node_p = target;
    target->parent_ = parent;
    return target;
  }
  if (key < node->key_) {
    auto target = FindOrInsertNode(node->left_, &(node->left_), node, key);
    TreeRebalance(node->left_, &(node->left_));
    return target;
  } else if (key > node->key_) {
    auto target = FindOrInsertNode(node->right_, &(node->right_), node, key);
    TreeRebalance(node->right_, &(node->right_));
    return target;
  }
  // key == node->key_
  return node;
}

void ArenaAddressAllocator::RemoveNode(ArenaAddressNode* node,
                                       ArenaAddressNode** node_p,
                                       uint64_t key) {
  if (!node) {
    return;
  }
  if (!FindNode(node, key)) {
    return;
  }
  if (key < node->key_) {
    RemoveNode(node->left_, &(node->left_), key);
    if (TreeBalanceFactor(node) < -1) {
      TreeRebalance(meta_->struct_.root_, &(meta_->struct_.root_));
    }
    return;
  } else if (key > node->key_) {
    RemoveNode(node->right_, &(node->right_), key);
    if (TreeBalanceFactor(node) > 1) {
      TreeRebalance(meta_->struct_.root_, &(meta_->struct_.root_));
    }
    return;
  }
  // node->key_ == key
  // left and right both exist
  if (node->left_ && node->right_) {
    if (TreeBalanceFactor(node) > 0) {
      auto max_p = &(node->left_);
      auto max = TreeMax(node->left_, &max_p);
      SwapNodePosition(node, node_p, max, max_p);
      RemoveNode(max->left_, &(max->left_), key);
      return;
    } else {
      auto min_p = &(node->right_);
      auto min = TreeMin(node->right_, &min_p);
      SwapNodePosition(node, node_p, min, min_p);
      RemoveNode(min->right_, &(min->right_), key);
      return;
    }
  }
  // left or right exist
  *node_p = node->left_ ? node->left_ : node->right_;
  ReclaimNode(node);
}

void ArenaAddressAllocator::SwapNodePosition(ArenaAddressNode* x,
                                             ArenaAddressNode** x_p,
                                             ArenaAddressNode* y,
                                             ArenaAddressNode** y_p) {
  if (x == nullptr || y == nullptr) {
    // cannot swap nullptr
    return;
  }

  // swap address
  *x_p = y;
  *y_p = x;
  // swap parent
  auto parent_x = x->parent_;
  auto parent_y = y->parent_;
  x->parent_ = parent_y;
  y->parent_ = parent_x;

  ArenaAddressNode* tmp = nullptr;

  // swap left
  tmp = x->left_;
  x->left_ = y->left_;
  if (x->left_) {
    x->left_->parent_ = x;
  }
  y->left_ = tmp;
  if (y->left_) {
    y->left_->parent_ = y;
  }

  // swap right
  tmp = x->right_;
  x->right_ = y->right_;
  if (x->right_) {
    x->right_->parent_ = x;
  }
  y->right_ = tmp;
  if (y->right_) {
    y->right_->parent_ = y;
  }
}

uint64_t ArenaAddressAllocator::TreeHeight(ArenaAddressNode* node) {
  if (node == nullptr) {
    return 0;
  }
  return 1 + std::max(TreeHeight(node->left_), TreeHeight(node->right_));
}

ArenaAddressNode* ArenaAddressAllocator::TreeMax(ArenaAddressNode* node,
                                                 ArenaAddressNode*** node_pp) {
  if (node == nullptr) {
    return nullptr;
  }
  for (; node->right_;) {
    *node_pp = &(node->right_);
    node = node->right_;
  }
  return node;
}

ArenaAddressNode* ArenaAddressAllocator::TreeMin(ArenaAddressNode* node,
                                                 ArenaAddressNode*** node_pp) {
  if (node == nullptr) {
    return nullptr;
  }
  for (; node->left_;) {
    *node_pp = &(node->left_);
    node = node->left_;
  }
  return node;
}

int64_t ArenaAddressAllocator::TreeBalanceFactor(ArenaAddressNode* node) {
  if (node == nullptr) {
    return 0;
  }
  return TreeHeight(node->left_) - TreeHeight(node->right_);
}

ArenaAddressNode* ArenaAddressAllocator::TreeRotateLeft(
    ArenaAddressNode* node, ArenaAddressNode** node_p) {
  auto parent = node->parent_;
  auto right = node->right_;

  node->right_ = right->left_;
  right->left_ = node;
  node->parent_ = right;
  *node_p = right;
  right->parent_ = parent;

  return right;
}

ArenaAddressNode* ArenaAddressAllocator::TreeRotateRight(
    ArenaAddressNode* node, ArenaAddressNode** node_p) {
  auto parent = node->parent_;
  auto left = node->left_;

  node->left_ = left->right_;
  left->right_ = node;
  node->parent_ = left;
  *node_p = left;
  left->parent_ = parent;

  return left;
}

ArenaAddressNode* ArenaAddressAllocator::TreeRotateLeftRight(
    ArenaAddressNode* node, ArenaAddressNode** node_p) {
  TreeRotateLeft(node->left_, &(node->left_));
  return TreeRotateRight(node, node_p);
}

ArenaAddressNode* ArenaAddressAllocator::TreeRotateRightLeft(
    ArenaAddressNode* node, ArenaAddressNode** node_p) {
  TreeRotateRight(node->right_, &(node->right_));
  return TreeRotateLeft(node, node_p);
}

ArenaAddressNode* ArenaAddressAllocator::TreeRebalance(
    ArenaAddressNode* node, ArenaAddressNode** node_p) {
  if (node == nullptr) {
    return nullptr;
  }
  auto balance_factor = TreeBalanceFactor(node);
  if (balance_factor > 1) {
    if (TreeBalanceFactor(node->left_) > 0) {
      return TreeRotateRight(node, node_p);
    } else {
      return TreeRotateLeftRight(node, node_p);
    }
  } else if (balance_factor < -1) {
    if (TreeBalanceFactor(node->right_) < 0) {
      return TreeRotateLeft(node, node_p);
    } else {
      return TreeRotateRightLeft(node, node_p);
    }
  }
  return node;
}

void* ArenaAddressAllocator::Allocate(uint64_t key) {
  bool idle_status = false;
  bool occupied_status = true;
  while (!meta_->struct_.occupied_.compare_exchange_strong(
                              idle_status, occupied_status)) {
    // exchange failed, idle_status is set to true.
    idle_status = false;
    std::this_thread::sleep_for(std::chrono::microseconds(1000));
  }
  auto node = FindOrInsertNode(meta_->struct_.root_, &(meta_->struct_.root_),
                               nullptr, key);
  meta_->struct_.occupied_.store(false);
  if (node == nullptr) {
    return nullptr;
  }

  node->ref_count_.fetch_add(1);

  return reinterpret_cast<void*>(
      reinterpret_cast<uint64_t>(meta_->struct_.base_address_) +
      meta_->struct_.address_segment_size_ * node->index_);
}

void ArenaAddressAllocator::Deallocate(uint64_t key) {
  bool idle_status = false;
  bool occupied_status = true;
  while (!meta_->struct_.occupied_.compare_exchange_strong(
                              idle_status, occupied_status)) {
    // exchange failed, idle_status is set to true.
    idle_status = false;
    std::this_thread::sleep_for(std::chrono::microseconds(1000));
  }
  auto node = FindNode(meta_->struct_.root_, key);
  meta_->struct_.occupied_.store(false);
  if (node == nullptr) {
    return;
  }

  if (node->ref_count_.fetch_sub(1) == 0) {
    RemoveNode(meta_->struct_.root_, &(meta_->struct_.root_), key);
  }
}

}  // namespace transport
}  // namespace cyber
}  // namespace apollo
