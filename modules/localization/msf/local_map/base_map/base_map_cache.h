#ifndef MODULES_LOCALIZATION_MSF_LOCAL_MAP_BASE_MAP_BASE_MAP_CACHE_H
#define MODULES_LOCALIZATION_MSF_LOCAL_MAP_BASE_MAP_BASE_MAP_CACHE_H

#include <deque>
#include <list>
#include <map>
#include "boost/thread.hpp"
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
  explicit LRUCache(int capacity) : _capacity(capacity) {}
  /**@brief The destructor. */
  virtual ~LRUCache() {}
  /**@brief Find element for key if it exists in the cache. If not exist, return
   * false. */
  bool get(const Key &key, Element *&value);
  /**@brief Find element for key if it exists in the cache. If not exist, return
   * false. */
  bool get(Key &key, Element *&value);
  /**@brief Find element for key if it exists in the cache. If not exist, return
   * false. This function is thread safe, but DONT change position of element in
   * LRU queue. */
  bool get_silent(const Key &key, Element *&value);
  /**@brief Find element for key if it exists in the cache. If not exist, return
   * false. This function is thread safe, but DONT change position of element in
   * LRU queue. */
  bool get_silent(Key &key, Element *&value);
  /**@brief Caches element for key. If cache is full,
   * return the removed element, otherwise return null. */
  Element *put(const Key &key, Element *value);
  /**@brief Caches element for key. If cache is full,
   * return the removed element, otherwise return null. */
  Element *put(Key &key, Element *value);
  /**@brief Remove element for key. if it exist in the cache,
   * return the element, otherwise return null. */
  Element *remove(const Key &key);
  /**@brief Remove element for key. if it exist in the cache,
   * return the element, otherwise return null. */
  Element *remove(Key &key);
  /**@brief Remove the Least Recently Used element in the cache.
   * return the removed element or null. */
  Element *clear_one();
  // bool clear();
  /**@brief Find element for key in the cache. If it exists, move it to the head
   * of queue. */
  bool is_exist(const Key &key);
  /**@brief Find element for key in the cache. If it exists, move it to the head
   * of queue. */
  bool is_exist(Key &key);
  /**@brief Change cache's max capacity. New capacity must be larger than size
   * in use. */
  bool change_capacity(int capacity) {
    if (_list.size() > capacity) {
      return false;
    }
    _capacity = capacity;
    return true;
  }
  /**@brief return cache's in use. */
  int size() {
    return _list.size();
  }
  /**@brief return cache's max capacity. */
  int capacity() {
    return _capacity;
  }

 protected:
  /**@brief do something before remove a element from cache.
   * Return true if the element can be removed. Return false if the element
   * can't be removed. Then the cache will try to find another element to
   * remove. */
  virtual bool destroy(Element *&value);

 private:
  /**@brief The max caoacity of LRUCache. */
  int _capacity;
  /**@brief Increse the search speed in queue. */
  std::map<Key, ListIterator> _map;
  /**@brief The least recently used queue. */
  std::list<std::pair<Key, Element *>> _list;
};

template <class Key, class Element>
bool LRUCache<Key, Element>::get(const Key &key, Element *&value) {
  MapIterator found_iter = _map.find(key);
  if (found_iter == _map.end()) {
    return false;
  }
  // move the corresponding key to list front
  _list.splice(_list.begin(), _list, found_iter->second);
  value = found_iter->second->second;
  return true;
}

template <class Key, class Element>
bool LRUCache<Key, Element>::get(Key &key, Element *&value) {
  const Key &key_const = key;
  return get(key_const, value);
}

template <class Key, class Element>
bool LRUCache<Key, Element>::get_silent(const Key &key, Element *&value) {
  MapIterator found_iter = _map.find(key);
  if (found_iter == _map.end()) {
    return false;
  }
  value = found_iter->second->second;
  return true;
}

template <class Key, class Element>
bool LRUCache<Key, Element>::get_silent(Key &key, Element *&value) {
  const Key &key_const = key;
  return get_silent(key_const, value);
}

template <class Key, class Element>
Element *LRUCache<Key, Element>::put(const Key &key, Element *value) {
  if (value == NULL) {
    std::cout << "LRUCache Warning: put a NULL" << std::endl;
    return NULL;
  }
  Element *node_remove = NULL;
  MapIterator found_iter = _map.find(key);
  if (found_iter != _map.end()) {
    // move the corresponding key to list front
    _list.splice(_list.begin(), _list, found_iter->second);
    node_remove = found_iter->second->second;
    if (node_remove == value) return NULL;
    if (destroy(node_remove)) {
      found_iter->second->second = value;
    } else {
      node_remove = value;
    }
    return node_remove;
  }
  // reached capacity, remove value in list, remove key in map
  if (_map.size() >= _capacity) {
    node_remove = clear_one();
    // ListReverseIterator ritr = _list.rbegin();
    // while(ritr != _list.rend()) {
    //     if(destroy(ritr->second)) {
    //         node_remove = ritr->second;
    //         _map.erase(ritr->first);
    //         _list.erase(ritr);
    //         break;
    //     }
    //     ++ritr;
    // }
  }
  if (_map.size() >= _capacity) {
    std::cout << "LRUCache Warning: the cache size is temporarily increased!"
              << std::endl;
  }
  _list.emplace_front(key, value);  // push_front
  _map[key] = _list.begin();
  return node_remove;
}

template <class Key, class Element>
Element *LRUCache<Key, Element>::put(Key &key, Element *value) {
  const Key &key_const = key;
  return put(key_const, value);
}

template <class Key, class Element>
bool LRUCache<Key, Element>::is_exist(const Key &key) {
  MapIterator found_iter = _map.find(key);
  if (found_iter != _map.end()) {
    _list.splice(_list.begin(), _list, found_iter->second);
  }
  return found_iter != _map.end();
}

template <class Key, class Element>
bool LRUCache<Key, Element>::is_exist(Key &key) {
  const Key &key_const = key;
  return is_exist(key_const);
}

template <class Key, class Element>
Element *LRUCache<Key, Element>::remove(const Key &key) {
  Element *node_remove = NULL;
  MapIterator found_iter = _map.find(key);
  // don't find key in cache
  if (found_iter == _map.end()) {
    return node_remove;
  }  // try to destruct element
  else if (destroy(found_iter->second->second)) {
    node_remove = found_iter->second->second;
    _list.erase(found_iter->second);
    _map.erase(found_iter);
    return node_remove;
  }
  return node_remove;
}

template <class Key, class Element>
Element *LRUCache<Key, Element>::remove(Key &key) {
  const Key &key_const = key;
  remove(key_const);
}

template <class Key, class Element>
Element *LRUCache<Key, Element>::clear_one() {
  // std::cout << "clear_one start" << std::endl;
  Element *node_remove = NULL;
  ListReverseIterator ritr = _list.rbegin();
  while (ritr != _list.rend()) {
    if (destroy(ritr->second)) {
      node_remove = ritr->second;
      _map.erase(ritr->first);
      ritr = ListReverseIterator(_list.erase((++ritr).base()));
      break;
    }
    ++ritr;
  }
  // std::cout << "clear_one end" << std::endl;
  return node_remove;
}

template <class Key, class Element>
bool LRUCache<Key, Element>::destroy(Element *&value) {
  return true;
}

/**@brief The data structure of the MapNodeCacheL1. */
template <class Key, class MapNode>
class MapNodeCacheL1 : public LRUCache<Key, MapNode> {
 public:
  MapNodeCacheL1(int capacity) : LRUCache<Key, MapNode>(capacity) {}

 protected:
  /**@brief do something before remove a element from cache.
   * Return true if the element can be removed. Return false if the element
   * can't be removed. Then the cache will try to find another element to
   * remove. */
  virtual bool destroy(MapNode *&value) {
    value->set_is_reserved(false);
    return true;
  }
};

/**@brief The data structure of the MapNodeCacheL2. */
template <class Key, class MapNode>
class MapNodeCacheL2 : public LRUCache<Key, MapNode> {
 public:
  MapNodeCacheL2(int capacity) : LRUCache<Key, MapNode>(capacity) {}

 protected:
  /**@brief do something before remove a element from cache.
   * Return true if the element can be removed. Return false if the element
   * can't be removed. Then the cache will try to find another element to
   * remove. */
  virtual bool destroy(MapNode *&value) {
    return !(value->get_is_reserved());
  }
};

}  // namespace msf
}  // namespace localization
}  // namespace apollo

#endif  // MODULES_LOCALIZATION_MSF_LOCAL_MAP_BASE_MAP_BASE_MAP_CACHE_H
