/******************************************************************************
 * Copyright 2020 The Apollo Authors. All Rights Reserved.
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

#include <functional>
#include <utility>
#include "cyber/common/log.h"

#include "modules/common/util/lru_cache.h"

namespace apollo {
namespace localization {
namespace msf {

template <class Key, class Element>
using LRUCache = ::apollo::common::util::LRUCache<Key, Element*>;

/**@brief The data structure of the LRUCache. */
template <class Key, class Element, class MapLRUCache = LRUCache<Key, Element>>
class MapNodeCache {
 public:
  using DestroyFunc = std::function<bool(Element*)>;
  static bool CacheL1Destroy(Element* value) {
    value->SetIsReserved(false);
    return true;
  }
  static bool CacheL2Destroy(Element* value) {
    return !(value->GetIsReserved());
  }

 public:
  /**@brief The constructor. */
  MapNodeCache(unsigned int capacity, const DestroyFunc& destroy_func)
      : destroy_func_(destroy_func), lru_map_nodes_(capacity) {}
  /**@brief The destructor. */
  ~MapNodeCache() {}
  /**@brief Find element for key if it exists in the cache. If not exist, return
   * false. */
  bool Get(const Key& key, Element** value);
  /**@brief Find element for key if it exists in the cache. If not exist, return
   * false. This function is thread safe, but don't change position of element
   * in LRU queue. */
  bool GetSilent(const Key& key, Element** value);
  /**@brief Caches element for key. If cache is full,
   * return the removed element, otherwise return null. */
  Element* Put(const Key& key, Element* value);
  /**@brief Remove element for key. if it exist in the cache,
   * return the element, otherwise return null. */
  Element* Remove(const Key& key);
  /**@brief Remove the Least Recently Used element in the cache.
   * return the removed element or null. */
  Element* ClearOne();
  // bool clear();
  /**@brief Find element for key in the cache. If it exists, move it to the head
   * of queue. */
  bool IsExist(const Key& key);

  /**@brief Change cache's max capacity. New capacity must be larger than size
   * in use. */
  bool ChangeCapacity(int capacity) {
    return lru_map_nodes_.ChangeCapacity(capacity);
  }
  /**@brief return cache's in use. */
  unsigned int Size() { return lru_map_nodes_.size(); }
  /**@brief return cache's max capacity. */
  unsigned Capacity() { return lru_map_nodes_.capacity(); }

 private:
  /**@brief do something before remove an element from cache.
   * Return true if the element can be removed. Return false if the element
   * can't be removed. Then the cache will try to find another element to
   * remove. */
  const DestroyFunc destroy_func_;
  MapLRUCache lru_map_nodes_;
};

template <class Key, class Element, class MapLRUCache>
bool MapNodeCache<Key, Element, MapLRUCache>::Get(const Key& key,
                                                  Element** value) {
  auto value_ptr = lru_map_nodes_.Get(key);
  if (!value_ptr) {
    return false;
  }
  *value = *value_ptr;
  return true;
}

template <class Key, class Element, class MapLRUCache>
bool MapNodeCache<Key, Element, MapLRUCache>::GetSilent(const Key& key,
                                                        Element** value) {
  auto value_ptr = lru_map_nodes_.GetSilently(key);
  if (!value_ptr) {
    return false;
  }
  *value = *value_ptr;
  return true;
}

template <class Key, class Element, class MapLRUCache>
Element* MapNodeCache<Key, Element, MapLRUCache>::Put(const Key& key,
                                                      Element* value) {
  if (value == nullptr) {
    AINFO << "LRUCache Warning: put a NULL";
    return nullptr;
  }

  auto* value_ptr = lru_map_nodes_.Get(key);
  Element* node_remove = nullptr;
  if (value_ptr) {
    node_remove = *value_ptr;
    if (destroy_func_(node_remove)) {
      *value_ptr = value;
    } else {
      node_remove = value;
    }
    return node_remove;
  }

  if (lru_map_nodes_.size() >= lru_map_nodes_.capacity()) {
    auto* node = lru_map_nodes_.Last();
    node_remove = node->val;
    Key key_tmp;
    lru_map_nodes_.PutAndGetObsolete(key, &value, &key_tmp);
    return node_remove;
  }

  lru_map_nodes_.Put(key, std::move(value));
  return node_remove;
}

template <class Key, class Element, class MapLRUCache>
Element* MapNodeCache<Key, Element, MapLRUCache>::Remove(const Key& key) {
  auto* node_remove = lru_map_nodes_.GetSilently(key);
  if (node_remove && lru_map_nodes_.Remove(key)) {
    return *node_remove;
  }

  return nullptr;
}

template <class Key, class Element, class MapLRUCache>
Element* MapNodeCache<Key, Element, MapLRUCache>::ClearOne() {
  auto* node_remove = lru_map_nodes_.Last();
  if (!node_remove) {
    return nullptr;
  }
  while (node_remove != lru_map_nodes_.First()) {
    if (destroy_func_(node_remove->val)) {
      lru_map_nodes_.Remove(node_remove->key);
      return node_remove->val;
    }
    node_remove = node_remove->prev;
  }
  if (node_remove == lru_map_nodes_.First() &&
      destroy_func_(node_remove->val)) {
    lru_map_nodes_.Remove(node_remove->key);
    return node_remove->val;
  }
  return nullptr;
}

template <class Key, class Element, class MapLRUCache>
bool MapNodeCache<Key, Element, MapLRUCache>::IsExist(const Key& key) {
  return lru_map_nodes_.Prioritize(key);
}

}  // namespace msf
}  // namespace localization
}  // namespace apollo
