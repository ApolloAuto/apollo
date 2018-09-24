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
#ifndef I_LIB_NUMERIC_I_QR_H
#define I_LIB_NUMERIC_I_QR_H

#include "../core/i_rand.h"
#include "i_householder.h"

namespace idl {
/*Perform explicit LQ-decomposition (meaning L is lower triangular and Q is
orthogonal) of the m x n matrix A
using Householder transformations, returning the (n x n) Q in Q and L in A
(zeroes are not filled in).
The function needs n scratch space in h, htA, ss and iv*/
template <typename T>
inline void i_lq_decompose_explicit(T **Q, T **A, T *h, T *htA, T *ss, int m,
                                    int n, int *iv) {
  int i_max, pivot_element;
  T diag_el;
  /*compute square-sums of rows for pivoting*/
  for (int i = 0; i < m; i++) {
    ss[i] = i_squaresum(A[i], n);
  }
  if (iv) {
    i_ramp(iv, m);
  }
  i_max = i_min(m, n - 1);
  if (i_max <= 0) {
    if (n == 1) {
      Q[0][0] = (T)1.0;
    }
  }
  for (int i = 0; i < i_max; i++) {
    /*pivot*/
    pivot_element = i_max_index(ss + i, m - i /*the # of rows left*/) + i;
    i_swap(ss[i], ss[pivot_element]);
    i_swap(A[i], A[pivot_element], n); /*swap rows*/
    i_swap(iv[i], iv[pivot_element]);

    /*compute Householder vector for remainder of row i*/
    diag_el = i_householder(A[i] + i, h, n - i);
    /*apply the Householder transformation to the remaining block of A*/
    A[i][i] = diag_el;
    i_apply_householder_right(h, A, m, n, i + 1, i);
    /*Update Q accordingly*/
    if (i == 0) {
      i_householder_transformation(h, Q, n);
    } else {
      i_apply_householder_left(h, Q, htA, n, n, i, 0);
    }
    /*update the square sums of rows*/
    for (int j = i + 1; j < m; j++) {
      ss[j] -= i_sqr(A[j][i]);
    }
  }
}

/*Perform explicit LQ-decomposition (meaning L is lower triangular and Q is
orthogonal) of the 3 x 3 matrix A
using Householder transformations, returning the (3 x 3) Q in Q and L in A
(zeroes are not filled in).
The function needs 3 scratch space in h, htA, ss and iv*/
template <typename T>
inline void i_lq_decompose_explicit_3x3(T **Q, T **A, T h[3], T htA[3], T ss[3],
                                        int iv[3]) {
  int pivot_element;
  T diag_el;
  /*compute square-sums of rows for pivoting*/
  ss[0] = i_squaresum3(A[0]);
  ss[1] = i_squaresum3(A[1]);
  ss[2] = i_squaresum3(A[2]);

  iv[0] = 0;
  iv[1] = 1;
  iv[2] = 2;

  /*unloop case i = 0:*/
  pivot_element = i_max_index(ss, 3 /*the # of rows left*/);
  i_swap(ss[0], ss[pivot_element]);
  i_swap3(A[0], A[pivot_element]); /*swap rows*/
  i_swap(iv[0], iv[pivot_element]);

  /*compute Householder vector for remainder of row i*/
  diag_el = i_householder(A[0], h, 3);
  /*apply the Householder transformation to the remaining block of A*/
  A[0][0] = diag_el;
  i_apply_householder_right(h, A, 3, 3, 1, 0);
  /*Update Q accordingly*/
  i_householder_transformation(h, Q, 3);
  /*update the square sums of rows*/
  ss[1] -= i_sqr(A[1][0]);
  ss[2] -= i_sqr(A[2][0]);

  /*unloop case i = 1:*/
  pivot_element = i_max_index(ss + 1, 2 /*the # of rows left*/) + 1;
  i_swap(ss[1], ss[pivot_element]);
  i_swap3(A[1], A[pivot_element]); /*swap rows*/
  i_swap(iv[1], iv[pivot_element]);

  /*compute Householder vector for remainder of row i*/
  diag_el = i_householder(A[1] + 1, h, 2);
  /*apply the Householder transformation to the remaining block of A*/
  A[1][1] = diag_el;
  i_apply_householder_right(h, A, 3, 3, 2, 1);
  /*Update Q accordingly*/
  i_apply_householder_left(h, Q, htA, 3, 3, 1, 0);
  /*update the square sums of rows*/
  ss[2] -= i_sqr(A[2][1]);
}

/*Random (m x n) matrix where the rows are orthonormal if m<=n, and the columns
are orthonormal otherwise.
The function needs n scratch space in h, htA, ss and iv, mxn scratch space in R
and A*/
template <typename T>
inline void i_rand_orthogonal(T **Q, T **A, T **R, T *h, T *htA, T *ss, int m,
                              int n, int *iv, int &s) {
  int mmn = i_max(m, n);
  /*Random matrix*/
  i_rand_matrix(A, mmn, mmn, s);
  /*LQ-factorize it*/
  i_lq_decompose_explicit(Q, A, h, htA, ss, m, n, iv);
  /*copy appropriate portion to R*/
  i_copy(Q, R, m, n);
}

} /* namespace idl */

#endif