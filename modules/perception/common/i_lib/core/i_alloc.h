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
#ifndef I_LIB_CORE_I_ALLOC_H
#define I_LIB_CORE_I_ALLOC_H

#include <new>
#include "i_basic.h"
/*
Alignment power->alignment offset in bytes:
0->1
1->2
2->4
3->8
4->16
5->32
*/

//#define I_DEFAULT_ALIGNMENT_POWER  4 /*by default use 16-byte memory alignment
//option*/

namespace idl {
template <typename T>
inline T *i_alloc(int memory_size) {
  T *mem = NULL;
  if (!memory_size) {
    return mem;
  }
  mem = new (std::nothrow) T[memory_size];
  if (mem == NULL) {
    return (NULL);
  }
  return (mem);
}

template <typename T>
inline void i_free(T *&mem) {
  if (mem != NULL) {
    delete[] mem;
    mem = NULL;
  }
}

/*Memory allocated with this function must be released with i_free2*/
template <typename T>
inline T **i_alloc2(int m, int n) {
  T *mem, **head;
  mem = new (std::nothrow) T[m * n];
  if (mem == NULL) {
    return (NULL);
  }
  head = new (std::nothrow) T *[m];
  if (head == NULL) {
    delete[] mem;
    return (NULL);
  }
  i_make_reference<T>(mem, head, m, n);
  return (head);
}

/*Free memory allocated with function i_alloc2*/
template <typename T>
inline void i_free2(T **&A) {
  if (A != NULL) {
    delete[] A[0];
    delete[] A;
    A = NULL;
  }
}

/*Allocate an (l x m x n) T tensor*/
template <class T>
inline T ***i_alloc3(int l, int m, int n) {
  T *mem, ***head;
  int i, j;
  mem = new (std::nothrow) T[l * m * n];
  if (mem == NULL) {
    return (NULL);
  }
  head = new (std::nothrow) T **[l];
  if (head == NULL) {
    delete[] mem;
    return (NULL);
  }
  for (i = 0; i < l; i++) {
    head[i] = new (std::nothrow) T *[m];
    if (head[i] == NULL) {
      for (j = 0; j < i; j++) delete[] head[j];
      delete[] head;
      delete[] mem;
      return (NULL);
    }
  }
  i_make_reference(mem, head, l, m, n);
  return (head);
}

template <class T>
inline void i_free3(T ***&A, int l) {
  if (A != NULL) {
    delete[](A[0][0]);
    for (int i = 0; i < l; i++) delete[] A[i];
    delete[] A;
    A = NULL;
  }
}

/*Memory allocated with this function must be released with i_free_aligned*/
template <typename T>
inline T *i_alloc_aligned(int memory_size, int alignment_power = 4) {
  if (memory_size <= 0) {
    return ((T *)NULL);
  }
  int actual_alignment_power = (alignment_power >= 0) ? alignment_power : 0;
  std::size_t memory_size_in_byte = (std::size_t)memory_size * sizeof(T);
  std::size_t mask = (std::size_t)((1 << actual_alignment_power) - 1);
  std::size_t pointer_size_in_byte = sizeof(char *);
  char *mem_begin = new (
      std::nothrow) char[memory_size_in_byte + mask + pointer_size_in_byte];
  if (!mem_begin) {
    return ((T *)NULL);
  }
  char *mem_actual =
      (char *)(((std::size_t)mem_begin + mask + pointer_size_in_byte) &
               (~mask));
  ((char **)mem_actual)[-1] = mem_begin;
  return ((T *)mem_actual);
}

/*Free memory allocated with function i_alloc_aligned*/
template <typename T>
inline void i_free_aligned(T *&mem) {
  if (mem) {
    delete[]((char **)mem)[-1];
    mem = NULL;
  }
}

template <typename T>
inline int i_verify_alignment(const T *mem, int alignment_power = 4) {
  std::size_t mask = (std::size_t)((1 << alignment_power) - 1);
  if (((size_t)mem) & mask) {
    return (0);
  } else {
    return (1);
  }
}

template <typename T>
inline T *i_align_pointer(T *mem, unsigned long byte_count) {
  unsigned long long inu, d;
  T *out;
  inu = ((unsigned long)mem);
  d = (inu % byte_count);
  if (d) {
    out = (T *)(inu - d + byte_count);
  } else {
    out = mem;
  }
  return (out);
}

} /* namespace idl */

#endif
