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

/**
 * @file lru_cache.h
 **/

#ifndef MODULES_PLANNING_COMMON_LRU_CACHE_H
#define MODULES_PLANNING_COMMON_LRU_CACHE_H

#include <iostream>
#include <vector>
#include <unordered_map>
#include <utility>
#include <mutex>

namespace apollo {
namespace planning {

template<class K, class V>
struct Node {
  K key;
  V val;
  Node* prev;
  Node* next;
  Node(): prev(nullptr), next(nullptr) {
    key = {};
    val = {};
  }

  template <typename VV>
  Node(const K& key, VV&& val) : key(key), val(std::forward<VV>(val)),
                                 prev(nullptr), next(nullptr) {}
};

template<class K, class V>
class LRUCache {
 private:
  size_t _capacity;
  size_t _size;
  std::unordered_map< K, Node<K, V> >  _map;
  Node<K, V> _head;
  Node<K, V> _tail;

 public:
  LRUCache() : _capacity(10), _map(0), _head(), _tail() {
    init();
  }

  explicit LRUCache(size_t capacity) :
      _capacity(capacity), _map(0), _head(), _tail() {
    init();
  }

  ~LRUCache() {
    clear();
  }

  void get_cache(std::unordered_map<K, V>& cache) {
    for (auto it = _map.begin(); it != _map.end(); ++it) {
      cache[it->first] = it->second.val;
    }
  }

  V& operator[] (const K& key) {
    if (!contains(key)) {
      K obsolete;
      get_obsolete(&obsolete);
    }
    return _map[key].val;
  }

  /*
   * Silently get all as vector
   */
  void get_all_silently(std::vector<V*>& ret) {
    for (auto it = _map.begin(); it != _map.end(); ++it) {
      ret.push_back(&it->second.val);
    }
  }

  /*
   * for both add & update purposes
   */
  template <typename VV>
  bool put(const K& key, VV&& val) {
    K tmp;
    return update(key, std::forward<VV>(val), &tmp, false, false);
  }

  /*
   * update existing elements only
   */
  template <typename VV>
  bool update(const K& key, VV&& val) {
    if (!contains(key)) {
      return false;
    }
    K tmp;
    return update(key, std::forward<VV>(val), &tmp, true, false);
  }

  /*
   * silently update existing elements only
   */
  template <typename VV>
  bool update_silently(const K& key, VV& val) {
    if (!contains(key)) {
      return false;
    }
    K tmp;
    return update(key, std::forward<VV>(val), &tmp, true, true);
  }

  /*
   * add new elements only
   */
  template <typename VV>
  bool add(const K& key, VV& val) {
    K tmp;
    return update(key, std::forward<VV>(val), &tmp, true, false);
  }

  template <typename VV>
  bool put_and_get_obsolete(const K& key, VV& val, K* obs) {
    return update(key, std::forward<VV>(val), obs, false, false);
  }

  template <typename VV>
  bool add_and_get_obsolete(const K& key, VV& val, K* obs) {
    return update(key, std::forward<VV>(val), obs, true, false);
  }

  V* get_silently(const K& key) {
    return get(key, true);
  }

  V* get(const K& key) {
    return get(key, false);
  }

  bool get_copy_silently(const K& key, const V* val) {
    return get_copy(key, val, true);
  }

  bool get_copy(const K& key, const V* val) {
    return get_copy(key, val, false);
  }

  size_t size() {
    return _size;
  }

  bool full() {
    return size() > 0 && size() >= _capacity;
  }

  bool empty() {
    return size() == 0;
  }

  size_t capacity() {
    return _capacity;
  }

  void set_capacity(size_t capacity) {
    _capacity = capacity;
  }

  Node<K, V>* first() {
    if (size()) {
      return _head.next;
    }
    return nullptr;
  }

  bool contains(const K& key) {
    return _map.find(key) != _map.end();
  }

  bool prioritize(const K& key) {
    if (contains(key)) {
      auto *node = &_map[key];
      detach(node);
      attach(node);
      return true;
    }
    return false;
  }

  void clear() {
    _map.clear();
    _size = 0;
  }

 private:
  void init() {
    _head.prev = nullptr;
    _head.next = &_tail;
    _tail.prev = &_head;
    _tail.next = nullptr;
    _size = 0;
  }

  void detach(Node<K, V> *node) {
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

  void attach(Node<K, V> *node) {
    node->prev = &_head;
    node->next = _head.next;
    _head.next = node;
    if (node->next != nullptr) {
      node->next->prev = node;
    }
    ++_size;
  }

  template <typename VV>
  bool update(const K& key, VV&& val, K* obs, bool add_only,
              bool silent_update) {
    if (obs == nullptr) {
      return false;
    }
    if (contains(key)) {
      if (!add_only) {
        _map[key].val = std::forward<VV>(val);
        if (!silent_update) {
          auto *node = &_map[key];
          detach(node);
          attach(node);
        } else {
          return false;
        }
      }
    } else {
      if (full() && !get_obsolete(obs)) {
        return false;
      }

      _map.emplace(key, Node<K, V>(key, std::forward<VV>(val)));
      attach(&_map[key]);
    }
    return true;
  }

  V* get(const K& key, bool silent) {
    if (contains(key)) {
      auto *node = &_map[key];
      if (!silent) {
        detach(node);
        attach(node);
      }
      return &node->val;
    }
    return nullptr;
  }

  bool get_copy(const K& key, const V* val, bool silent) {
    if (contains(key)) {
      auto *node = &_map[key];
      if (!silent) {
        detach(node);
        attach(node);
      }
      *val = node->val;
      return true;
    }
    return false;
  }

  bool get_obsolete(K* key) {
    if (full()) {
      auto *node = _tail.prev;
      detach(node);
      _map.erase(node->key);
      *key = node->key;
      return true;
    }
    return false;
  }
};

}  // namespace planning
}  // namespace apollo

#endif  // MODULES_PLANNING_COMMON_LRU_CACHE_H
