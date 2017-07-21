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

#ifndef MODULES_COMMON_UTIL_LRU_CACHE_H_
#define MODULES_COMMON_UTIL_LRU_CACHE_H_

#include <iostream>
#include <vector>
#include <unordered_map>
#include <mutex>
#include <utility>

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
      : key(key), val(std::forward<VV>(val)), prev(nullptr), next(nullptr){}
};

template <class K, class V>
class LRUCache {
 private:
  size_t _capacity;
  size_t _size;
  std::unordered_map<K, Node<K, V>> _map;
  Node<K, V> _head;
  Node<K, V> _tail;

 public:
  LRUCache() : _capacity(10), _map(0), _head(), _tail() {
    Init();
  }

  explicit LRUCache(size_t capacity)
      : _capacity(capacity), _map(0), _head(), _tail() {
    Init();
  }

  ~LRUCache() {
    Clear();
  }

  void GetCache(std::unordered_map<K, V>* cache) {
    for (auto it = _map.begin(); it != _map.end(); ++it) {
      cache->operator[](it->first) = it->second.val;
    }
  }

  V& operator[](const K& key) {
    if (!Contains(key)) {
      K obsolete;
      GetObsolete(&obsolete);
    }
    return _map[key].val;
  }

  /*
   * Silently get all as vector
   */
  void GetAllSilently(std::vector<V*>* ret) {
    for (auto it = _map.begin(); it != _map.end(); ++it) {
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

  V* GetSilently(const K& key) {
    return Get(key, true);
  }

  V* Get(const K& key) {
    return Get(key, false);
  }

  bool GetCopySilently(const K& key, const V* val) {
    return GetCopy(key, val, true);
  }

  bool GetCopy(const K& key, const V* val) {
    return GetCopy(key, val, false);
  }

  size_t size() {
    return _size;
  }

  bool Full() {
    return size() > 0 && size() >= _capacity;
  }

  bool Empty() {
    return size() == 0;
  }

  size_t capacity() {
    return _capacity;
  }

  void set_capacity(size_t capacity) {
    _capacity = capacity;
  }

  Node<K, V>* First() {
    if (size()) {
      return _head.next;
    }
    return nullptr;
  }

  bool Contains(const K& key) {
    return _map.find(key) != _map.end();
  }

  bool Prioritize(const K& key) {
    if (Contains(key)) {
      auto* node = &_map[key];
      Detach(node);
      Attach(node);
      return true;
    }
    return false;
  }

  void Clear() {
    _map.clear();
    _size = 0;
  }

 private:
  void Init() {
    _head.prev = nullptr;
    _head.next = &_tail;
    _tail.prev = &_head;
    _tail.next = nullptr;
    _size = 0;
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
    --_size;
  }

  void Attach(Node<K, V>* node) {
    node->prev = &_head;
    node->next = _head.next;
    _head.next = node;
    if (node->next != nullptr) {
      node->next->prev = node;
    }
    ++_size;
  }

  template <typename VV>
  bool Update(const K& key, VV&& val, K* obs, bool add_only,
              bool silent_update) {
    if (obs == nullptr) {
      return false;
    }
    if (Contains(key)) {
      if (!add_only) {
        _map[key].val = std::forward<VV>(val);
        if (!silent_update) {
          auto* node = &_map[key];
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

      _map.emplace(key, Node<K, V>(key, std::forward<VV>(val)));
      Attach(&_map[key]);
    }
    return true;
  }

  V* Get(const K& key, bool silent) {
    if (Contains(key)) {
      auto* node = &_map[key];
      if (!silent) {
        Detach(node);
        Attach(node);
      }
      return &node->val;
    }
    return nullptr;
  }

  bool GetCopy(const K& key, const V* val, bool silent) {
    if (Contains(key)) {
      auto* node = &_map[key];
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
      auto* node = _tail.prev;
      Detach(node);
      _map.erase(node->key);
      *key = node->key;
      return true;
    }
    return false;
  }
};


}  // namespace util
}  // namespace common
}  // namespace apollo

#endif  // MODULES_COMMON_UTIL_LRU_CACHE_H_
