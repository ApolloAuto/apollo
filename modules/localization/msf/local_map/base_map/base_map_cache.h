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

#pragma once

#include <boost/thread.hpp>
#include <list>
#include <map>
#include <utility>

#include "cyber/common/log.h"
#include "modules/localization/msf/local_map/base_map/base_map_fwd.h"

namespace apollo {
namespace localization {
namespace msf {

/**@brief The data structure of the LRUCache. */
template <class Key, class Element>
class LRUCache {
 public:
  typedef typename std::list<std::pair<Key, Element *>>::iterator ListIterator;
  typedef typename std::list<std::pair<Key, Element *>>::reverse_iterator
      ListReverseIterator;
  typedef typename std::map<Key, ListIterator>::iterator MapIterator;
  /**@brief The constructor. */
  explicit LRUCache(int capacity) : capacity_(capacity) {}
  /**@brief The destructor. */
  virtual ~LRUCache() {}
  /**@brief Find element for key if it exists in the cache. If not exist, return
   * false. */
  bool Get(const Key &key, Element **value);
  /**@brief Caches element for key. If cache is full,
   * return the removed element, otherwise return null. */
  Element *Put(const Key &key, Element *value);
  /**@brief Remove element for key. if it exist in the cache,
   * return the element, otherwise return null. */
  Element *Remove(const Key &key);
  /**@brief Remove the Least Recently Used element in the cache.
   * return the removed element or null. */
  Element *ClearOne();
  // bool clear();
  /**@brief Find element for key in the cache. If it exists, move it to the head
   * of queue. */
  bool IsExist(const Key &key);
  /**@brief Change cache's max capacity. New capacity must be larger than size
   * in use. */
  bool ChangeCapacity(int capacity) {
    if (static_cast<int>(list_.size()) > capacity) {
      return false;
    }
    capacity_ = capacity;
    return true;
  }
  /**@brief return cache's in use. */
  int Size() { return static_cast<int>(list_.size()); }
  /**@brief return cache's max capacity. */
  int Capacity() { return capacity_; }

 protected:
  /**@brief do something before remove an element from cache.
   * Return true if the element can be removed. Return false if the element
   * can't be removed. Then the cache will try to find another element to
   * remove. */
  virtual bool Destroy(Element **value);

 private:
  /**@brief The max caoacity of LRUCache. */
  int capacity_;
  /**@brief Increase the search speed in queue. */
  std::map<Key, ListIterator> map_;
  /**@brief The least recently used queue. */
  std::list<std::pair<Key, Element *>> list_;
};

template <class Key, class Element>
bool LRUCache<Key, Element>::Get(const Key &key, Element **value) {
  MapIterator found_iter = map_.find(key);
  if (found_iter == map_.end()) {
    return false;
  }
  // move the corresponding key to list front
  list_.splice(list_.begin(), list_, found_iter->second);
  *value = found_iter->second->second;
  return true;
}

template <class Key, class Element>
Element *LRUCache<Key, Element>::Put(const Key &key, Element *value) {
  if (value == nullptr) {
    AWARN << "LRUCache Warning: put a nullptr";
    return nullptr;
  }
  Element *node_remove = nullptr;
  MapIterator found_iter = map_.find(key);
  if (found_iter != map_.end()) {
    // move the corresponding key to list front
    list_.splice(list_.begin(), list_, found_iter->second);
    node_remove = found_iter->second->second;
    if (node_remove == value) {
      return nullptr;
    }
    if (Destroy(&node_remove)) {
      found_iter->second->second = value;
    } else {
      node_remove = value;
    }
    return node_remove;
  }
  // reached capacity, remove value in list, remove key in map
  if (static_cast<int>(map_.size()) >= capacity_) {
    node_remove = ClearOne();
    // ListReverseIterator ritr = list_.rbegin();
    // while(ritr != list_.rend()) {
    //     if(Destroy(ritr->second)) {
    //         node_remove = ritr->second;
    //         map_.erase(ritr->first);
    //         list_.erase(ritr);
    //         break;
    //     }
    //     ++ritr;
    // }
  }
  if (static_cast<int>(map_.size()) >= capacity_) {
    AWARN << "LRUCache Warning: the cache size is temporarily increased!";
  }
  list_.emplace_front(key, value);  // push_front
  map_[key] = list_.begin();
  return node_remove;
}

template <class Key, class Element>
bool LRUCache<Key, Element>::IsExist(const Key &key) {
  MapIterator found_iter = map_.find(key);
  if (found_iter != map_.end()) {
    list_.splice(list_.begin(), list_, found_iter->second);
  }
  return found_iter != map_.end();
}

template <class Key, class Element>
Element *LRUCache<Key, Element>::Remove(const Key &key) {
  Element *node_remove = nullptr;
  MapIterator found_iter = map_.find(key);

  if (found_iter == map_.end()) {
    // don't find key in cache
    return node_remove;
  } else if (Destroy(&(found_iter->second->second))) {
    // try to destruct element
    node_remove = found_iter->second->second;
    list_.erase(found_iter->second);
    map_.erase(found_iter);
    return node_remove;
  }
  return node_remove;
}

template <class Key, class Element>
Element *LRUCache<Key, Element>::ClearOne() {
  Element *node_remove = nullptr;
  ListReverseIterator ritr = list_.rbegin();
  while (ritr != list_.rend()) {
    if (Destroy(&(ritr->second))) {
      node_remove = ritr->second;
      map_.erase(ritr->first);
      ritr = ListReverseIterator(list_.erase((++ritr).base()));
      break;
    }
    ++ritr;
  }
  return node_remove;
}

template <class Key, class Element>
bool LRUCache<Key, Element>::Destroy(Element **value) {
  return true;
}

/**@brief The data structure of the MapNodeCacheL1. */
template <class Key, class MapNode>
class MapNodeCacheL1 : public LRUCache<Key, MapNode> {
 public:
  explicit MapNodeCacheL1(int capacity) : LRUCache<Key, MapNode>(capacity) {}

 protected:
  /**@brief do something before remove an element from cache.
   * Return true if the element can be removed. Return false if the element
   * can't be removed. Then the cache will try to find another element to
   * remove. */
  virtual bool Destroy(MapNode **value) {
    (*value)->SetIsReserved(false);
    return true;
  }
};

/**@brief The data structure of the MapNodeCacheL2. */
template <class Key, class MapNode>
class MapNodeCacheL2 : public LRUCache<Key, MapNode> {
 public:
  explicit MapNodeCacheL2(int capacity) : LRUCache<Key, MapNode>(capacity) {}

 protected:
  /**@brief do something before remove an element from cache.
   * Return true if the element can be removed. Return false if the element
   * can't be removed. Then the cache will try to find another element to
   * remove. */
  virtual bool Destroy(MapNode **value) { return !((*value)->GetIsReserved()); }
};

}  // namespace msf
}  // namespace localization
}  // namespace apollo
