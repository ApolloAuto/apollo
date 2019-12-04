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

#pragma once

#include <iostream>
#include <mutex>
#include <unordered_map>
#include <utility>
#include <vector>

namespace apollo {
namespace common {
namespace util {

template <class K, class V>
struct Node {
  K key;
  V val;
  Node* prev;
  Node* next;
  Node() : prev(nullptr), next(nullptr) {
    key = {};
    val = {};
  }

  template <typename VV>
  Node(const K& key, VV&& val)
      : key(key), val(std::forward<VV>(val)), prev(nullptr), next(nullptr) {}
};

template <class K, class V>
class LRUCache {
 public:
  explicit LRUCache(const size_t capacity = kDefaultCapacity)
      : capacity_(capacity), map_(0), head_(), tail_() {
    Init();
  }

  ~LRUCache() { Clear(); }

  void GetCache(std::unordered_map<K, V>* cache) {
    for (auto it = map_.begin(); it != map_.end(); ++it) {
      cache->emplace(it->first, it->second.val);
    }
  }

  V& operator[](const K& key) {
    if (!Contains(key)) {
      K obsolete;
      GetObsolete(&obsolete);
    }
    return map_[key].val;
  }

  /*
   * Silently get all as vector
   */
  void GetAllSilently(std::vector<V*>* ret) {
    for (auto it = map_.begin(); it != map_.end(); ++it) {
      ret->push_back(&it->second.val);
    }
  }

  /*
   * for both add & update purposes
   */
  template <typename VV>
  bool Put(const K& key, VV&& val) {
    K tmp;
    return Update(key, std::forward<VV>(val), &tmp, false, false);
  }

  /*
   * update existing elements only
   */
  template <typename VV>
  bool Update(const K& key, VV&& val) {
    if (!Contains(key)) {
      return false;
    }
    K tmp;
    return Update(key, std::forward<VV>(val), &tmp, true, false);
  }

  /*
   * silently update existing elements only
   */
  template <typename VV>
  bool UpdateSilently(const K& key, VV* val) {
    if (!Contains(key)) {
      return false;
    }
    K tmp;
    return Update(key, std::forward<VV>(*val), &tmp, true, true);
  }

  /*
   * add new elements only
   */
  template <typename VV>
  bool Add(const K& key, VV* val) {
    K tmp;
    return Update(key, std::forward<VV>(*val), &tmp, true, false);
  }

  template <typename VV>
  bool PutAndGetObsolete(const K& key, VV* val, K* obs) {
    return Update(key, std::forward<VV>(*val), obs, false, false);
  }

  template <typename VV>
  bool AddAndGetObsolete(const K& key, VV* val, K* obs) {
    return Update(key, std::forward<VV>(*val), obs, true, false);
  }

  V* GetSilently(const K& key) { return Get(key, true); }

  V* Get(const K& key) { return Get(key, false); }

  bool GetCopySilently(const K& key, V* const val) {
    return GetCopy(key, val, true);
  }

  bool GetCopy(const K& key, V* const val) { return GetCopy(key, val, false); }

  size_t size() { return size_; }

  bool Full() { return size() > 0 && size() >= capacity_; }

  bool Empty() { return size() == 0; }

  size_t capacity() { return capacity_; }

  Node<K, V>* First() {
    if (size()) {
      return head_.next;
    }
    return nullptr;
  }

  bool Contains(const K& key) { return map_.find(key) != map_.end(); }

  bool Prioritize(const K& key) {
    if (Contains(key)) {
      auto* node = &map_[key];
      Detach(node);
      Attach(node);
      return true;
    }
    return false;
  }

  void Clear() {
    map_.clear();
    Init();
  }

 private:
  static constexpr size_t kDefaultCapacity = 10;

  const size_t capacity_;
  size_t size_;
  std::unordered_map<K, Node<K, V>> map_;
  Node<K, V> head_;
  Node<K, V> tail_;

  void Init() {
    head_.prev = nullptr;
    head_.next = &tail_;
    tail_.prev = &head_;
    tail_.next = nullptr;
    size_ = 0;
  }

  void Detach(Node<K, V>* node) {
    if (node->prev != nullptr) {
      node->prev->next = node->next;
    }
    if (node->next != nullptr) {
      node->next->prev = node->prev;
    }
    node->prev = nullptr;
    node->next = nullptr;
    --size_;
  }

  void Attach(Node<K, V>* node) {
    node->prev = &head_;
    node->next = head_.next;
    head_.next = node;
    if (node->next != nullptr) {
      node->next->prev = node;
    }
    ++size_;
  }

  template <typename VV>
  bool Update(const K& key, VV&& val, K* obs, bool add_only,
              bool silent_update) {
    if (obs == nullptr) {
      return false;
    }
    if (Contains(key)) {
      if (!add_only) {
        map_[key].val = std::forward<VV>(val);
        if (!silent_update) {
          auto* node = &map_[key];
          Detach(node);
          Attach(node);
        } else {
          return false;
        }
      }
    } else {
      if (Full() && !GetObsolete(obs)) {
        return false;
      }

      map_.emplace(key, Node<K, V>(key, std::forward<VV>(val)));
      Attach(&map_[key]);
    }
    return true;
  }

  V* Get(const K& key, bool silent) {
    if (Contains(key)) {
      auto* node = &map_[key];
      if (!silent) {
        Detach(node);
        Attach(node);
      }
      return &node->val;
    }
    return nullptr;
  }

  bool GetCopy(const K& key, V* const val, bool silent) {
    if (Contains(key)) {
      auto* node = &map_[key];
      if (!silent) {
        Detach(node);
        Attach(node);
      }
      *val = node->val;
      return true;
    }
    return false;
  }

  bool GetObsolete(K* key) {
    if (Full()) {
      auto* node = tail_.prev;
      Detach(node);
      *key = node->key;
      map_.erase(node->key);
      return true;
    }
    return false;
  }
};

}  // namespace util
}  // namespace common
}  // namespace apollo
