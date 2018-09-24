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

#include "../core/i_blas.h"

namespace idl {
/*Pivot by finding element in row with largest magnitude element starting at
position i of matrix A with size m x n.
keep A recording in iv*/
template <typename T>
inline void i_pivot_column(T *A, int i, int m, int n, int *iv) {
  int index = i_max_abs_index_interval(A + i * n, i, n);
  if (index != i) {
    i_swap_cols(A, i, index, m, n);
    i_swap(iv[i], iv[index]);
  }
}

/*Pivot by finding element in columen with largest magnitude element starting at
column i of matrix A with size m x n.
keep A recording in iv*/
template <typename T>
inline void i_pivot_row(T *A, int i, int m, int n, int *iv) {
  int index = i_max_abs_index_interval_column(A + i, i, m, n);
  if (index != i) {
    i_swap_rows_interval(A, i, n, i, index, m, n);
    i_swap(iv[i], iv[index]);
  }
}

/*Perform the row reduce step of Gauss-Jordan elimination*/
template <typename T>
inline void i_gauss_jordan_row_reduce(T *A, int i, int m, int n) {
  T *p1, *p2, *p3;
  int nr_cols_left = n - i - 1;
  int offset = i * n;
  /*normalize row*/
  p1 = A + offset + i;
  p2 = p1 + 1;
  i_scale(p2, nr_cols_left, i_rec(*p1));
  offset += n;
  /*subtract outer product of row and column*/
  for (int j = i + 1; j < m; j++, offset += n) {
    p3 = A + offset + i;
    i_sub_scaled(p2, p3 + 1, nr_cols_left, *p3);
  }
}

/*Perform the upper half of Gauss-Jordan elimination*/
template <typename T>
inline void i_gauss_jordan_reduce_upper(T *A, int m, int n) {
  int o = i_min(m, n);
  T *p1;
  /*reduce upper half and remaining columns*/
  int nr_col_left = n - o;
  int jn;
  for (int i = o - 1; i > 0; i--) {
    /*subtract outer product of row and column*/
    p1 = A + i * n + o;
    for (int j = i - 1; j >= 0; j--) {
      jn = j * n;
      i_sub_scaled(p1, A + jn + o, nr_col_left, A[jn + i]);
    }
  }
}

/*Perform Gauss-Jordan elimination of A, which is of size mxn.
use column pivoting, i.e., switching in the column with the largest
pivot element in each step. assume full rank. iv needs to be
pre-allocated to size n*/
template <typename T>
inline void i_gauss_jordan_column_pivot(T *A, int m, int n, int *iv) {
  int o = i_min(m, n);
  i_ramp(iv, n);
  /*triangulate system*/
  for (int i = 0; i < o; i++) {
    /*pivot*/
    i_pivot_column(A, i, m, n, iv);
    /*reduce*/
    i_gauss_jordan_row_reduce(A, i, m, n);
  }
  /*reduce upper*/
  i_gauss_jordan_reduce_upper(A, m, n);
}

/*Perform Gauss-Jordan elimination of A, which is of size mxn.
use row pivoting, i.e. switching in the row with the largest
pivot element in each step. assume full rank. iv needs to be
pre-allocated to size m*/
template <typename T>
inline void i_gauss_jordan_row_pivot(T *A, int m, int n, int *iv) {
  int o = i_min(m, n);
  i_ramp(iv, m);
  /*triangulate system*/
  for (int i = 0; i < o; i++) {
    /*pivot*/
    i_pivot_row(A, i, m, n, iv);
    /*Reduce*/
    i_gauss_jordan_row_reduce(A, i, m, n);
  }
  /*Reduce Upper*/
  i_gauss_jordan_reduce_upper(A, m, n);
}

/*Perform LU-decomposition on a square (n x n) matrix A with row pivoting.
The L Matrix is written into the lower part of A, including the diagonal.
The U Matrix has ones on the diagonal, while the remainder is the written
into the super-diagonal part of A. Record pivoting in index, where index[i]
(which needs to be allocated to n positions) indicates which row of A is
equal to row i of LU*/
template <typename T>
inline void i_lu_decompose_square(T *A, int n, int *index) {
  int i, j, ni, nj, pivot, nm1;
  T *Aiip1, *Ajpi;
  i_ramp(index, n);
  /*Upper Triangulate*/
  for (i = 0; i < n; i++) {
    /*Row pivot*/
    ni = n * i;
    pivot = i_max_abs_index_subdiagonal_column(A, i, n);
    i_swap(A + ni, A + pivot * n, n);
    i_swap(index[i], index[pivot]); /*swap indices*/
    /*rescale row to unitize diagonal element*/
    Aiip1 = A + ni + i + 1;
    nm1 = n - i - 1;
    i_scale(Aiip1, nm1, i_rec(A[ni + i]));
    /*Subtract scaled version of row from rows below to zero out subdiagonal
     * elements*/
    for (j = i + 1; j < n; j++) {
      nj = n * j;
      Ajpi = A + nj + i;
      /*Compute y=y-x*k where x and y are n-dimensional vectors, k is constant*/
      i_sub_scaled(Aiip1, Ajpi + 1, nm1, *Ajpi);
    }
  }
}

/*Backsubstitute an LU-decomposition LU=A of a square (n x n) matrix A to
find x such that Ax=y. It is NOT ok to use x=y*/
template <typename T>
inline void i_lu_backsub(const T *LU, const T *y, const int *index, T *x,
                         int n) {
  int i, j;
  const T *LUi;
  T xi;

  /*Move y to x while unshuffling the pivoting*/
  for (i = 0; i < n; i++) x[i] = y[index[i]];

  /*Backsubstitute through the L-factor*/
  for (i = 0; i < n; i++) {
    LUi = LU + n * i;
    for (xi = x[i], j = 0; j < i; j++) xi -= x[j] * LUi[j];
    x[i] = i_div(xi, LUi[i]);
  }

  /*Backsubstitute through the U-factor*/
  for (i = n - 1; i >= 0; i--) {
    LUi = LU + n * i;
    for (xi = x[i], j = i + 1; j < n; j++) xi -= x[j] * LUi[j];
    x[i] = xi;
  }
}

template <typename T>
inline void i_lu_backsub_2x2(const T LU[4], const T y[2], const int index[2],
                             T x[2]) {
  x[0] = y[index[0]];
  x[1] = y[index[1]];

  x[0] = i_div(x[0], LU[0]);
  x[1] = i_div(x[1] - x[0] * LU[2], LU[3]);

  x[0] = x[0] - x[1] * LU[1];
}

template <typename T>
inline void i_lu_backsub_3x3(const T LU[9], const T y[3], const int index[3],
                             T x[3]) {
  x[0] = y[index[0]];
  x[1] = y[index[1]];
  x[2] = y[index[2]];

  x[0] = i_div(x[0], LU[0]);
  x[1] = i_div(x[1] - x[0] * LU[3], LU[4]);
  x[2] = i_div(x[2] - x[0] * LU[6] - x[1] * LU[7], LU[8]);

  x[1] = x[1] - x[2] * LU[5];
  x[0] = x[0] - x[1] * LU[1] - x[2] * LU[2];
}

template <typename T>
inline void i_lu_backsub_4x4(const T LU[16], const T y[4], const int index[4],
                             T x[4]) {
  x[0] = y[index[0]];
  x[1] = y[index[1]];
  x[2] = y[index[2]];
  x[3] = y[index[3]];

  x[0] = i_div(x[0], LU[0]);
  x[1] = i_div(x[1] - x[0] * LU[4], LU[5]);
  x[2] = i_div(x[2] - x[0] * LU[8] - x[1] * LU[9], LU[10]);
  x[3] = i_div(x[3] - x[0] * LU[12] - x[1] * LU[13] - x[2] * LU[14], LU[15]);

  x[2] = x[2] - x[3] * LU[11];
  x[1] = x[1] - x[2] * LU[6] - x[3] * LU[7];
  x[0] = x[0] - x[1] * LU[1] - x[2] * LU[2] - x[3] * LU[3];
}

template <typename T>
inline void i_lu_backsub_5x5(const T LU[25], const T y[5], const int index[5],
                             T x[5]) {
  x[0] = y[index[0]];
  x[1] = y[index[1]];
  x[2] = y[index[2]];
  x[3] = y[index[3]];
  x[4] = y[index[4]];

  x[0] = i_div(x[0], LU[0]);
  x[1] = i_div(x[1] - x[0] * LU[5], LU[6]);
  x[2] = i_div(x[2] - x[0] * LU[10] - x[1] * LU[11], LU[12]);
  x[3] = i_div(x[3] - x[0] * LU[15] - x[1] * LU[16] - x[2] * LU[17], LU[18]);
  x[4] = i_div(
      x[4] - x[0] * LU[20] - x[1] * LU[21] - x[2] * LU[22] - x[3] * LU[23],
      LU[24]);

  x[3] = x[3] - x[4] * LU[19];
  x[2] = x[2] - x[3] * LU[13] - x[4] * LU[14];
  x[1] = x[1] - x[2] * LU[7] - x[3] * LU[8] - x[4] * LU[9];
  x[0] = x[0] - x[1] * LU[1] - x[2] * LU[2] - x[3] * LU[3] - x[4] * LU[4];
}

/*Compute the inverse of a 2x2 matrix A using the LU decomposition*/
template <typename T>
inline void i_lu_invert_2x2(const T A[4], T Ai[4]) {
  int index[2];
  T LU[4], x[2], y[2];
  i_copy4(A, LU);
  i_lu_decompose_square(LU, 2, index);
  y[0] = (T)1.0;
  y[1] = (T)0.0;
  i_lu_backsub_2x2(LU, y, index, x);
  Ai[0] = x[0];
  Ai[2] = x[1];

  y[0] = (T)0.0;
  y[1] = (T)1.0;
  i_lu_backsub_2x2(LU, y, index, x);
  Ai[1] = x[0];
  Ai[3] = x[1];
}

/*Compute the inverse of a 3x3 matrix A using the LU decomposition*/
template <typename T>
inline void i_lu_invert_3x3(const T A[9], T Ai[9]) {
  int index[3];
  T LU[9], x[3], y[3];
  i_copy9(A, LU);
  i_lu_decompose_square(LU, 3, index);
  y[0] = (T)1.0;
  y[1] = (T)0.0;
  y[2] = (T)0.0;
  i_lu_backsub_3x3(LU, y, index, x);
  Ai[0] = x[0];
  Ai[3] = x[1];
  Ai[6] = x[2];

  y[0] = (T)0.0;
  y[1] = (T)1.0;
  i_lu_backsub_3x3(LU, y, index, x);
  Ai[1] = x[0];
  Ai[4] = x[1];
  Ai[7] = x[2];

  y[1] = (T)0.0;
  y[2] = (T)1.0;
  i_lu_backsub_3x3(LU, y, index, x);
  Ai[2] = x[0];
  Ai[5] = x[1];
  Ai[8] = x[2];
}

/*Compute the inverse of a 3x3 matrix A using the LU decomposition, A will be
 * destroyed in the process*/
template <typename T>
inline void i_lu_invert_3x3_destroy(T A[9], T Ai[9]) {
  int index[3];
  T x[3], y[3];
  i_lu_decompose_square(A, 3, index);

  y[0] = (T)1.0;
  y[1] = (T)0.0;
  y[2] = (T)0.0;
  i_lu_backsub_3x3(A, y, index, x);
  Ai[0] = x[0];
  Ai[3] = x[1];
  Ai[6] = x[2];

  y[0] = (T)0.0;
  y[1] = (T)1.0;
  i_lu_backsub_3x3(A, y, index, x);
  Ai[1] = x[0];
  Ai[4] = x[1];
  Ai[7] = x[2];

  y[1] = (T)0.0;
  y[2] = (T)1.0;
  i_lu_backsub_3x3(A, y, index, x);
  Ai[2] = x[0];
  Ai[5] = x[1];
  Ai[8] = x[2];
}

/*Compute the inverse of a 3x3 (2D) affine transform matrix A (both A and Ai's
 * last rows are 0,0,1) using the LU decomposition*/
template <typename T>
inline void i_lu_invert_affine_3x3(const T A[9], T Ai[9]) {
  int index[2];
  T LU[4], x[2], y[2];
  LU[0] = A[0];
  LU[1] = A[1];
  LU[2] = A[3];
  LU[3] = A[4];
  i_lu_decompose_square(LU, 2, index);

  y[0] = (T)1.0;
  y[1] = (T)0.0;
  i_lu_backsub_2x2(LU, y, index, x);
  Ai[0] = x[0];
  Ai[3] = x[1];

  y[0] = (T)0.0;
  y[1] = (T)1.0;
  i_lu_backsub_2x2(LU, y, index, x);
  Ai[1] = x[0];
  Ai[4] = x[1];

  y[0] = -(T)A[2];
  y[1] = -(T)A[5];
  i_lu_backsub_2x2(LU, y, index, x);
  Ai[2] = x[0];
  Ai[5] = x[1];

  Ai[6] = Ai[7] = (T)0;
  Ai[8] = (T)1.0;
}

/*Compute the inverse of a 4x4 matrix A using the LU decomposition*/
template <typename T>
inline void i_lu_invert_4x4(const T A[16], T Ai[16]) {
  int index[4];
  T LU[16], x[4], y[4];
  i_copy16(A, LU);
  i_lu_decompose_square(LU, 4, index);

  y[0] = (T)1.0;
  y[1] = (T)0.0;
  y[2] = (T)0.0;
  y[3] = (T)0.0;
  i_lu_backsub_4x4(LU, y, index, x);
  Ai[0] = x[0];
  Ai[4] = x[1];
  Ai[8] = x[2];
  Ai[12] = x[3];

  y[0] = (T)0.0;
  y[1] = (T)1.0;
  i_lu_backsub_4x4(LU, y, index, x);
  Ai[1] = x[0];
  Ai[5] = x[1];
  Ai[9] = x[2];
  Ai[13] = x[3];

  y[1] = (T)0.0;
  y[2] = (T)1.0;
  i_lu_backsub_4x4(LU, y, index, x);
  Ai[2] = x[0];
  Ai[6] = x[1];
  Ai[10] = x[2];
  Ai[14] = x[3];

  y[2] = (T)0.0;
  y[3] = (T)1.0;
  i_lu_backsub_4x4(LU, y, index, x);
  Ai[3] = x[0];
  Ai[7] = x[1];
  Ai[11] = x[2];
  Ai[15] = x[3];
}

/*Compute the inverse of a 5x5 matrix A using the LU decomposition*/
template <typename T>
inline void i_lu_invert_5x5(const T A[25], T Ai[25]) {
  int index[5];
  T LU[25], x[5], y[5];
  i_copy(A, LU, 25);
  i_lu_decompose_square(LU, 5, index);

  y[0] = (T)1.0;
  y[1] = (T)0.0;
  y[2] = (T)0.0;
  y[3] = (T)0.0;
  y[4] = (T)0.0;
  i_lu_backsub_5x5(LU, y, index, x);
  Ai[0] = x[0];
  Ai[5] = x[1];
  Ai[10] = x[2];
  Ai[15] = x[3];
  Ai[20] = x[4];

  y[0] = (T)0.0;
  y[1] = (T)1.0;
  i_lu_backsub_5x5(LU, y, index, x);
  Ai[1] = x[0];
  Ai[6] = x[1];
  Ai[11] = x[2];
  Ai[16] = x[3];
  Ai[21] = x[4];

  y[1] = (T)0.0;
  y[2] = (T)1.0;
  i_lu_backsub_5x5(LU, y, index, x);
  Ai[2] = x[0];
  Ai[7] = x[1];
  Ai[12] = x[2];
  Ai[17] = x[3];
  Ai[22] = x[4];

  y[2] = (T)0.0;
  y[3] = (T)1.0;
  i_lu_backsub_5x5(LU, y, index, x);
  Ai[3] = x[0];
  Ai[8] = x[1];
  Ai[13] = x[2];
  Ai[18] = x[3];
  Ai[23] = x[4];

  y[3] = (T)0.0;
  y[4] = (T)1.0;
  i_lu_backsub_5x5(LU, y, index, x);
  Ai[4] = x[0];
  Ai[9] = x[1];
  Ai[14] = x[2];
  Ai[19] = x[3];
  Ai[24] = x[4];
}

/*Compute the inverse of a nxn square matrix A using the LU decomposition*/
template <typename T, int n>
inline void i_lu_invert(const T *A, T *Ai) {
  static const int nn = n * n;
  int i, j, index[n], jn[n];
  T LU[nn], x[n], y[n + 1];
  i_copy(A, LU, nn);
  i_lu_decompose_square(LU, n, index);

  /*initialize y: 1, 0, 0, ... and jn: 0, n , 2n , ...*/
  for (i = 0; i < n; ++i) {
    y[i] = i == 0 ? (T)1.0 : (T)0.0;
    jn[i] = i * n;
  }

  /*solve:*/
  for (i = 0; i < n; ++i) {
    i_lu_backsub(LU, y, index, x, n);

    for (j = 0; j < n; ++j) {
      Ai[jn[j] + i] = x[j];
    }

    /*shift yi, not we alloc y with n+1 entries therefore this line is ok even
     * if i = n-1*/
    i_swap(y[i], y[i + 1]);
  }
}

} /* namespace idl */