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

#ifndef MODULES_PERCEPTION_LIB_BASE_SINGLETON_H_
#define MODULES_PERCEPTION_LIB_BASE_SINGLETON_H_

#include <pthread.h>

#include "modules/common/macro.h"

namespace apollo {
namespace perception {

// @brief Thread-safe, no-manual destroy Singleton template
template <typename T>
class Singleton {
 public:
  // @brief Get the singleton instance
  static T* get() {
    pthread_once(&p_once_, &Singleton::new_);
    return instance_;
  }

 private:
  Singleton();
  ~Singleton();

  // @brief Construct the singleton instance
  static void new_() { instance_ = new T(); }

  // @brief  Destruct the singleton instance
  // @note Only work with gcc
  __attribute__((destructor)) static void delete_() {
    typedef char T_must_be_complete[sizeof(T) == 0 ? -1 : 1];
    (void)sizeof(T_must_be_complete);
    delete instance_;
  }

  static pthread_once_t p_once_;  // Initialization once control
  static T* instance_;            // The singleton instance
};

template <typename T>
pthread_once_t Singleton<T>::p_once_ = PTHREAD_ONCE_INIT;

template <typename T>
T* Singleton<T>::instance_ = nullptr;

}  // namespace perception
}  // namespace apollo

#endif  // MODULES_PERCEPTION_LIB_BASE_SINGLETON_H
