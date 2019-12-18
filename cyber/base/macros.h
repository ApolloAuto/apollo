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

#ifndef CYBER_BASE_MACROS_H_
#define CYBER_BASE_MACROS_H_

#include <cstdlib>
#include <new>

#if __GNUC__ >= 3
#define cyber_likely(x) (__builtin_expect((x), 1))
#define cyber_unlikely(x) (__builtin_expect((x), 0))
#else
#define cyber_likely(x) (x)
#define cyber_unlikely(x) (x)
#endif

#define CACHELINE_SIZE 64

#define DEFINE_TYPE_TRAIT(name, func)                     \
  template <typename T>                                   \
  struct name {                                           \
    template <typename Class>                             \
    static constexpr bool Test(decltype(&Class::func)*) { \
      return true;                                        \
    }                                                     \
    template <typename>                                   \
    static constexpr bool Test(...) {                     \
      return false;                                       \
    }                                                     \
                                                          \
    static constexpr bool value = Test<T>(nullptr);       \
  };                                                      \
                                                          \
  template <typename T>                                   \
  constexpr bool name<T>::value;

inline void cpu_relax() {
#if defined(__aarch64__)
  asm volatile("yield" ::: "memory");
#else
  asm volatile("rep; nop" ::: "memory");
#endif
}

inline void* CheckedMalloc(size_t size) {
  void* ptr = std::malloc(size);
  if (!ptr) {
    throw std::bad_alloc();
  }
  return ptr;
}

inline void* CheckedCalloc(size_t num, size_t size) {
  void* ptr = std::calloc(num, size);
  if (!ptr) {
    throw std::bad_alloc();
  }
  return ptr;
}

#endif  // CYBER_BASE_MACROS_H_
