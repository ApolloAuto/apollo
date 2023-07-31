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

#include <iostream>

#include "modules/perception/common/algorithm/i_lib/core/i_basic.h"

//
// Alignment power->alignment offset in bytes:
// 0->1
// 1->2
// 2->4
// 3->8
// 4->16
// 5->32
//

// #define I_DEFAULT_ALIGNMENT_POWER  4 by default use 16-byte memory
// alignment option
namespace apollo {
namespace perception {
namespace algorithm {

template <typename T>
inline T *IAlloc(int memory_size) {
  T *mem = nullptr;
  if (!memory_size) {
    return mem;
  }
  mem = new (std::nothrow) T[memory_size];
  if (mem == nullptr) {
    return nullptr;
  }
  return mem;
}

template <typename T>
inline void IFree(T **mem) {
  if (mem != nullptr && *mem != nullptr) {
    delete[] * mem;
    *mem = nullptr;
  }
}

// Memory allocated with this function must be released with IFree2
template <typename T>
inline T **IAlloc2(int m, int n) {
  T *mem = nullptr;
  T **head = nullptr;
  mem = new (std::nothrow) T[m * n];
  if (mem == nullptr) {
    return nullptr;
  }
  head = new (std::nothrow) T *[m];
  if (head == nullptr) {
    delete[] mem;
    return nullptr;
  }
  IMakeReference<T>(mem, head, m, n);
  return head;
}

// Free memory allocated with function IAlloc2
template <typename T>
inline void IFree2(T ***A) {
  if (A != nullptr && *A != nullptr) {
    delete[](*A)[0];
    delete[] * A;
    *A = nullptr;
  }
}

// Allocate an (l x m x n) T tensor
template <class T>
inline T ***IAlloc3(int l, int m, int n) {
  T *mem = nullptr;
  T ***head = nullptr;
  int i, j;
  mem = new (std::nothrow) T[l * m * n];
  if (mem == nullptr) {
    return nullptr;
  }
  head = new (std::nothrow) T **[l];
  if (head == nullptr) {
    delete[] mem;
    return nullptr;
  }
  for (i = 0; i < l; i++) {
    head[i] = new (std::nothrow) T *[m];
    if (head[i] == nullptr) {
      for (j = 0; j < i; j++) {
        delete[] head[j];
      }
      delete[] head;
      delete[] mem;
      return nullptr;
    }
  }
  IMakeReference(mem, head, l, m, n);
  return head;
}

// Memory allocated with this function must be released with IFreeAligned
template <typename T>
inline T *IAllocAligned(int memory_size, int alignment_power = 4) {
  if (memory_size <= 0) {
    return (reinterpret_cast<T *>(NULL));
  }
  int actual_alignment_power = (alignment_power >= 0) ? alignment_power : 0;
  std::size_t memory_size_in_byte =
      static_cast<size_t>(memory_size) * sizeof(T);
  std::size_t mask = static_cast<size_t>((1 << actual_alignment_power) - 1);
  std::size_t pointer_size_in_byte = sizeof(char *);
  char *mem_begin = new (
      std::nothrow) char[memory_size_in_byte + mask + pointer_size_in_byte];
  if (!mem_begin) {
    return (reinterpret_cast<T *>(NULL));
  }
  char *mem_actual = reinterpret_cast<char *>(
      (reinterpret_cast<size_t>(mem_begin) + mask + pointer_size_in_byte) &
      (~mask));
  (reinterpret_cast<char **>(mem_actual))[-1] = mem_begin;
  return (reinterpret_cast<T *>(mem_actual));
}

// Free memory allocated with function IAllocAligned
template <typename T>
inline void IFreeAligned(T **mem) {
  if (mem != nullptr && *mem != nullptr) {
    delete[](reinterpret_cast<char **>(*mem))[-1];
    *mem = nullptr;
  }
}

template <typename T>
inline int IVerifyAlignment(const T *mem, int alignment_power = 4) {
  std::size_t mask = static_cast<size_t>((1 << alignment_power) - 1);
  if (((size_t)mem) & mask) {
    return 0;
  } else {
    return 1;
  }
}

}  //  namespace algorithm
}  //  namespace perception
}  //  namespace apollo
