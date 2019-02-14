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
const int I_DEFAULT_SEED = 432;

// Generate random number between 0 and 1, the seed s should not be 0
inline double IRandCoreD(int *s) {
  int a;
  a = *s / 127773;
  *s = 16807 * (*s - a * 127773) - 2836 * a;
  if (*s < 0) {
    *s += 2147483647;
  }
  return ((1.0 / (static_cast<double>(2147483647))) * (*s));
}

// Generate random integer in half-open interval [0,pool_size), the seed s
// should not be 0
inline int IRandI(int pool_size, int *s) {
  return (IIntervalHalfopen<int>(static_cast<int>(IRandCoreD(s) * pool_size), 0,
                                 pool_size));
}

// Generate n random integers from half open interval [0,pool_size)
// note that if pool_size is smaller than sample Size n, the sample will contain
// duplicate integers
inline void IRandomSample(int *sample, int n, int pool_size, int *s) {
  int temp, curr;
  if (pool_size < n) {
    for (int i = 0; i < n; i++) {
      sample[i] = IRandI(pool_size, s);
    }
  } else {
    for (int i = 0; i < n; i++) {
      // generate integer for an integer pool that descreases with i
      temp = IRandI(pool_size - i, s);
      // go through previous storted integers, if smaller, store, and move the
      // rest of the array one step. if larger or equal, increase by 1 and go
      // to the next. This ensures even distribution
      for (int j = 0; j < i; j++) {
        curr = sample[j];
        if (temp < curr) {
          sample[j] = temp;
          temp = curr;
        } else {
          temp++;
        }
      }
      // store the final integer
      sample[i] = temp;
    }
  }
}

// Generate a random permutation of array elements in place - Fisher and Yates
// algorithm Array A has n elements, each element has Size l
template <typename T>
inline void IRandomizedShuffle(T *A, int n, int l, int *s) {
  if (A == reinterpret_cast<T *>(NULL) || n <= 1 || l < 1) {
    return;
  }
  int i, r;
  for (i = n - 1; i > 0; i--) {
    r = IRandI(i + 1, s);  // pick a random index from 0 to i
    ISwap(A + r * l, A + i * l, l);
  }
}

// Generate a random permutation of two corresponding array elements
// in place - Fisher and Yates algorithm
// Array A and B has n elements, A's element has size la,
// B's element has size lb
template <typename T>
inline void IRandomizedShuffle(T *A, T *B, int n, int la, int lb, int *s) {
  if (A == reinterpret_cast<T *>(NULL) || B == reinterpret_cast<T *>(NULL) ||
      n <= 1 || la < 1 || lb < 1) {
    return;
  }
  int i, r;
  for (i = n - 1; i > 0; i--) {
    r = IRandI(i + 1, s); /*pick a random index from 0 to i*/
    ISwap(A + r * la, A + i * la, la);
    ISwap(B + r * lb, B + i * lb, lb);
  }
}

// Generate a random permutation of array elements in place - Fisher and Yates
// algorithm Array A has n elements, each element has Size 1
template <typename T>
inline void IRandomizedShuffle1(T *A, int n, int *s) {
  if (A == reinterpret_cast<T *>(NULL) || n <= 1) {
    return;
  }
  int i, r;
  for (i = n - 1; i > 0; i--) {
    // pick a random index from 0 to i
    r = IRandI(i + 1, s);
    ISwap(A[r], A[i]);
  }
}

}  // namespace common
}  // namespace perception
}  // namespace apollo
