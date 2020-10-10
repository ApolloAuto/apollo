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

#include "modules/perception/common/i_lib/core/i_basic.h"

namespace apollo {
namespace perception {
namespace common {

// Strictly less
template <typename T>
inline bool ILess(const T &a, const T &b) {
  return a < b;
}

// Strictly larger
template <typename T>
inline bool ILarger(const T &a, const T &b) {
  return a > b;
}

template <typename T>
inline T IMedian3(const T &a, const T &b, const T &c) {
  if (a <= b) {
    if (c > a) {
      if (c <= b) {
        return (c);
      }
      return (b);
    }
    return (a);
  }
  if (c > b) {
    if (c <= a) {
      return (c);
    }
    return (a);
  }
  return (b);
}

// Insertion sort
template <typename T, bool Compare(const T &, const T &)>
inline void IInsertionSort(T *a, int n) {
  int i, j;
  T tmp;
  for (i = 1; i < n; i++) {
    IMove(a[i], &tmp);
    for (j = i; j > 0; j--) {
      if (Compare(a[j - 1], tmp)) {
        break;
      }
      IMove(a[j - 1], &a[j]);
    }
    IMove(tmp, &a[j]);
  }
}

// Move the indexed elements to the left hand side of array "a" in place.
// Array "a" has size "n * element_size".
// Array "indices" has valid size "nr_indexed_elements",
// its length should be >= "nr_indexed_elements".
// Array a and indices will be destroyed after calling this routine
// Move the indexed elements to the left hand side of array "a" in place
template <typename T>
inline void IIndexedShuffle(T *a, int *indices, int n, int element_size,
                            int nr_indexed_elements,
                            bool is_sort_indices = true) {
  // pre-sort indices array into ascending order
  if (is_sort_indices) {
    IInsertionSort<int, ILess>(indices, nr_indexed_elements);
  }
  // move indexed elements to the left hand side of a
  int j = IMin(n, nr_indexed_elements);
  int m = IMin(n, nr_indexed_elements);
  for (int i = 0; i < m; ++i) {
    j = indices[i];
    if (j != i) {
      ISwap(a + i * element_size, a + j * element_size, element_size);
      // swap indexed elements to the left
    }
  }
}

// Move the indexed elements to the left hand side of array "a" and "b" in
// place.
// Array "a" has size "n*element_size_a".
// Array "b" has size "n*element_size_b".
// Array "indices" has valid size "nr_indexed_elements",
// its length should be >= "nr_indexed_elements".
// Array a, b, and indices will be destroyed after calling this routine
template <typename T>
inline void IIndexedShuffle(T *a, T *b, int *indices, int n, int element_size_a,
                            int element_size_b, int nr_indexed_elements,
                            bool is_sort_indices = true) {
  // pre-sort indices array into ascending order
  if (n <= 1) {
    return;
  }
  if (is_sort_indices) {
    IInsertionSort<int, ILess>(indices, nr_indexed_elements);
  }
  // move indexed elements to the left hand side of a
  int j = IMin(n, nr_indexed_elements);
  int m = IMin(n, nr_indexed_elements);
  for (int i = 0; i < m; ++i) {
    j = indices[i];
    if (j != i) {
      ISwap(a + i * element_size_a, a + j * element_size_a, element_size_a);
      // swap indexed elements to the left
      ISwap(b + i * element_size_b, b + j * element_size_b, element_size_b);
      // swap indexed elements to the left
    }
  }
}

}  // namespace common
}  // namespace perception
}  // namespace apollo
