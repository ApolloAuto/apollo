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
#ifndef I_LIB_CORE_I_RAND_H
#define I_LIB_CORE_I_RAND_H

#include "i_basic.h"

namespace idl {
const int I_DEFAULT_SEED = 432;

/*Random number between 0 and 1. Seed s should not be 0*/
template <typename T>
inline T i_rand_core(int &s) {
  int a;
  a = s / 127773;
  s = 16807 * (s - a * 127773) - 2836 * a;
  if (s < 0) {
    s += 2147483647;
  }
  return ((T)1.0 / ((T)2147483647) * s);
}

/*Generate random number between 0 and 1, the seed s should not be 0*/
inline double i_rand_core_d(int &s) {
  int a;
  a = s / 127773;
  s = 16807 * (s - a * 127773) - 2836 * a;
  if (s < 0) {
    s += 2147483647;
  }
  return ((1.0 / ((double)2147483647)) * s);
}

/*Generate random integer in half-open interval [0,pool_size), the seed s should
 * not be 0*/
inline int i_rand_i(int pool_size, int &s) {
  return (i_interval_halfopen<int>((int)(i_rand_core_d(s) * pool_size), 0,
                                   pool_size));
}

/*Random vector of size n with entries in interval [0,1]. Seed s should not be
 * 0*/
template <typename T>
inline void i_rand_vector(T *a, int n, int &s) {
  for (int i = 0; i < n; i++) {
    a[i] = i_rand_core<T>(s);
  }
}

/*Generate random matrix of size (m x n) with entries in interval [0,1]. Seed s
 * should not be 0*/
template <typename T>
inline void i_rand_matrix(T *A, int m, int n, int &s) {
  for (int i = 0; i < m; i++) {
    i_rand_vector<T>(A + i * n, n, s);
  }
}

/*Generate random matrix of size (m x n) with entries in interval [0,1]. Seed s
 * should not be 0*/
template <typename T>
inline void i_rand_matrix(T **A, int m, int n, int &s) {
  for (int i = 0; i < m; i++) {
    i_rand_vector<T>(A[i], n, s);
  }
}

/*Generate n random integers from half open interval [0,pool_size)
  note that if pool_size is smaller than sample size n, the sample will contain
  duplicate integers*/
inline void i_random_sample(int *sample, int n, int pool_size, int &s) {
  int temp, curr;
  if (pool_size < n) {
    for (int i = 0; i < n; i++) {
      sample[i] = i_rand_i(pool_size, s);
    }
  } else /*no duplicates*/
  {
    for (int i = 0; i < n; i++) {
      /*generate integer for an integer pool that descreases with i*/
      temp = i_rand_i(pool_size - i, s);
      /*go through previous storted integers, if smaller, store, and move the
        rest of the array one step.
        if larger or equal, increase by 1 and go to the next. This ensures even
        distribution*/
      for (int j = 0; j < i; j++) {
        curr = sample[j];
        if (temp < curr) {
          sample[j] = temp;
          temp = curr;
        } else {
          temp++;
        }
      }
      /*store the final integer*/
      sample[i] = temp;
    }
  }
}

/*Generate a random permutation of array elements in place - Fisher and Yates
 * algorithm*/
/*Array A has n elements, each element has size l*/
template <typename T>
inline void i_randomized_shuffle(T *A, int n, int l, int &s /*random seed*/) {
  if (A == (T *)NULL || n <= 1 || l < 1) {
    return;
  }
  int i, r;
  for (i = n - 1; i > 0; i--) {
    r = i_rand_i(i + 1, s); /*pick a random index from 0 to i*/
    i_swap(A + r * l, A + i * l, l);
  }
}

/*Generate a random permutation of array elements in place - Fisher and Yates
 * algorithm*/
/*Array A has n elements, each element has size 1*/
template <typename T>
inline void i_randomized_shuffle1(T *A, int n, int &s /*random seed*/) {
  if (A == (T *)NULL || n <= 1) {
    return;
  }
  int i, r;
  for (i = n - 1; i > 0; i--) {
    /*pick a random index from 0 to i*/
    r = i_rand_i(i + 1, s);
    i_swap(A[r], A[i]);
  }
}

/*Generate a random permutation of two corresponding array elements in place -
 * Fisher and Yates algorithm*/
/*Array A and B has n elements, A's element has size la, B's element has size
 * lb*/
template <typename T>
inline void i_randomized_shuffle(T *A, T *B, int n, int la, int lb,
                                 int &s /*random seed*/) {
  if (A == (T *)NULL || B == (T *)NULL || n <= 1 || la < 1 || lb < 1) {
    return;
  }
  int i, r;
  for (i = n - 1; i > 0; i--) {
    r = i_rand_i(i + 1, s); /*pick a random index from 0 to i*/
    i_swap(A + r * la, A + i * la, la);
    i_swap(B + r * lb, B + i * lb, lb);
  }
}

/*Generate a random permutation of two corresponding array elements in place -
 * Fisher and Yates algorithm*/
/*Array A and B has n elements, A's element has size 2, B's element also has
 * size 2*/
template <typename T>
inline void i_randomized_shuffle22(T *A, T *B, int n, int &s /*random seed*/) {
  if (A == (T *)NULL || B == (T *)NULL || n <= 1) {
    return;
  }
  int i, r, i2, r2;
  for (i = n - 1; i > 0; i--) {
    r = i_rand_i(i + 1, s); /*pick a random index from 0 to i*/
    r2 = (r << 1);
    i2 = (i << 1);
    i_swap2(A + r2, A + i2);
    i_swap2(B + r2, B + i2);
  }
}

/*The modified Box-Muller transform generates values from any normal
distribution with mean \mu and variance \sigma^2.
reference: http://en.wikipedia.org/wiki/Box¨CMuller_transform
*/
inline double i_rand_gaussian_core_d(int &s, double mu, double sigma) {
  const double epsilon = Constant<double>::MIN_VAL();
  const double two_pi = Constant<double>::TWO_PI();
  static double z0, z1;
  static bool generate;
  generate = !generate;
  if (!generate) {
    return z1 * sigma + mu;
  }
  double u1, u2, sqrtneg2logu1, two_pi_u2;
  do {
    u1 = i_rand_core_d(s);
    u2 = i_rand_core_d(s);
  } while (u1 <= epsilon);
  sqrtneg2logu1 = i_sqrt(-2.0 * i_log(u1));
  two_pi_u2 = two_pi * u2;
  z0 = sqrtneg2logu1 * i_cos(two_pi_u2);
  z1 = sqrtneg2logu1 * i_sin(two_pi_u2);
  return z0 * sigma + mu;
}
} /*namespace idl*/

#endif
