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
 * @file
 * @brief Defines the Dropbox class.
 */

#ifndef MODULES_COMMON_UTIL_DROPBOX_H_
#define MODULES_COMMON_UTIL_DROPBOX_H_

#include <string>
#include <unordered_map>

#include "modules/common/log.h"
#include "modules/common/macro.h"

namespace apollo {
namespace common {
namespace util {

using KeyType = std::string;

/**
 * @brief Dropbox class is a map based key-value storage container utility.
 */
template <class T>
class Dropbox {
 public:
  const T* get(const KeyType& key) const {
    auto iter = _store.find(key);
    if (iter == _store.end()) {
      return nullptr;
    } else {
      return &(iter->second);
    }
  }

  T* get(const KeyType& key) {
    auto iter = _store.find(key);
    if (iter == _store.end()) {
      return nullptr;
    } else {
      return &(iter->second);
    }
  }

  void set(const KeyType& key, const T& t) { _store[key] = t; }

  void remove(const KeyType& key) { _store.erase(key); }

  static Dropbox<T>* open() {
    static Dropbox<T> _static_store;
    return &_static_store;
  }

 private:
  std::unordered_map<KeyType, T> _store;
  Dropbox<T>() {}
  Dropbox<T>(const Dropbox& other) = delete;
};

}  // namespace util
}  // namespace common
}  // namespace apollo

#endif  // MODULES_COMMON_UTIL_DROPBOX_H_
