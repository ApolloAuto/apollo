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
/*Compute a Householder vector h from an n-dimensional vector x (vector x should
have non-zero magnitude).
This function returns h such that the transformation H=I-hh' is a
reflection that transforms x to a vector y of the same
magnitude along the first dimension, and with sign of the first element
equal to sign(x[0]). The function returns the first element of y.
H is always symmetric and orthogonal*/
template <typename T>
inline T i_householder(const T *x, T *h, int n) {
  T mag_squ = i_squaresum(x, n);
  T mag = i_sqrt(mag_squ);
  T x0 = x[0];
  T y0 = (x0 >= (T)0.0) ? -mag : mag;
  /*compute the scale factor for the denominator, note that x0*y0 must <= 0*/
  T sf = i_rec(i_sqrt(mag_squ - x0 * y0));
  h[0] = (x0 - y0) * sf;
  for (int i = 1; i < n; i++) {
    h[i] = x[i] * sf;
  }
  return y0;
}

/*Compute a Householder vector h from the cth column of matrix A (the cth column
should have non-zero magnitude).
This function returns h such that the transformation H=I-hh' is a
reflection that transforms the cth column to a vector y of the same
magnitude along the first dimension, and with sign of the first element
equal to sign(first element of the cth column). The function returns the first
element of y.
H is always symmetric and orthogonal*/
template <typename T>
inline T i_householder_column(const T *const *A, T *h, int c, int nr_row) {
  T mag_squ = (T)0.0;
  for (int i = 0; i < nr_row; i++) {
    mag_squ += i_sqr(A[i][c]);
  }
  T mag = i_sqrt(mag_squ);
  T x0 = A[0][c];
  T y0 = (x0 >= (T)0.0) ? -mag : mag;
  /*compute the scale factor for the denominator, note that x0*y0 must <= 0*/
  T sf = i_rec(i_sqrt(mag_squ - x0 * y0));
  h[0] = (x0 - y0) * sf;
  for (int i = 1; i < nr_row; i++) {
    h[i] = A[i][c] * sf;
  }
  return y0;
}

/*Form the Householder transformation represented by the n-dimensional vector h
 * in the nxn matrix H*/
template <typename T>
inline void i_householder_transformation(const T *h, T **H, int n) {
  T *Hi, sf;
  for (int i = 0; i < n; i++) {
    Hi = H[i];
    sf = -h[i];
    for (int j = 0; j < i; j++) {
      Hi[j] = h[j] * sf;
    }
    Hi[i] = (T)1.0 - sf * sf;
    for (int j = i + 1; j < n; j++) {
      Hi[j] = h[j] * sf;
    }
  }
}

/*Form the Householder transformation represented by the (n-1)-dimensional
 * vector h in the nxn matrix H*/
template <typename T>
inline void i_householder_transformation_hessenberg(const T *h, T **H, int n) {
  T *Hi, sf;
  H[0][0] = (T)1.0;
  i_zero(H[0] + 1, n - 1);
  for (int i = 1; i < n; i++) {
    Hi = H[i];
    sf = -h[i - 1];
    Hi[0] = (T)0.0;
    for (int j = 1; j < i; j++) {
      Hi[j] = h[j - 1] * sf;
    }
    Hi[i] = (T)1.0 - sf * sf;
    for (int j = i + 1; j < n; j++) {
      Hi[j] = h[j - 1] * sf;
    }
  }
}

/*Apply the Householder transformation H=I-hh' from the right on the
mxn matrix A. The operation is only applied starting with row r and column c.
n and c define the actual size of h because it is assumed that the
transformation
is applied to the last (n-c) columns of A, and it is assumed that h is input as
(n-c)-dimensional vector with the entries of h that are actually used*/
template <typename T>
inline void i_apply_householder_right(const T *h, T **A, int m, int n, int r,
                                      int c) {
  int i, nmc;
  T *Ai;
  nmc = n - c;
  for (i = r; i < m; i++) {
    Ai = A[i] + c;
    i_sub_scaled(h, Ai, nmc, i_dot(Ai, h, nmc));
  }
}

/*Apply the Householder transformation H=I-hh' from the left on the
mxn matrix A. The operation is only applied starting with row r and column c.
m and r define the actual size of h because it is assumed that the
transformation
is applied to the last (m-r) rows of A, and it is assumed that h is input as
(m-r)-dimensional vector with the entries of h that are actually used.
The function needs n-dimensional vector htA as scratch space*/
template <typename T>
inline void i_apply_householder_left(const T *h, T **A, T *htA, int m, int n,
                                     int r, int c) {
  T hi, *Ai, *htAp;
  Ai = A[r];
  hi = h[0];
  for (int j = c; j < n; j++) {
    htA[j] = hi * Ai[j];
  }
  for (int i = r + 1; i < m; i++) {
    Ai = A[i];
    hi = h[i - r];
    for (int j = c; j < n; j++) {
      htA[j] += hi * Ai[j];
    }
  }
  int nmc = n - c;
  htAp = htA + c;
  for (int i = r; i < m; i++) {
    i_sub_scaled(htAp, A[i] + c, nmc, h[i - r]);
  }
}

/*compute a Givens rotation R=
[ c s]
[-s c]
such that the second element of R[x0 x1]' is zero.*/
template <typename T>
inline void i_givens(T *c, T *s, T x0, T x1) {
  T f;
  /*The rotation we are looking for is simply
  [ c s] = [ x0 x1]
  [-s c]   [-x1 x0]
  but scaled so that c^2+s^2=1. To normalize, we need to divide
  out sqrt(x0^2+x1^2). For numerical purposes, the largest of
  |x0| and |x1| is found. Take the biggest and divide it
  out of the square root.*/
  if (i_abs(x0) >= i_abs(x1)) {
    /*sqrt(x0^2+x1^2)= |x0| sqrt(1+(x1/x0)^2) gives
    c=x0/(|x0| sqrt(1+(x1/x0)^2)) and
    s=x1/(|x0| sqrt(1+(x1/x0)^2))
    If we multiply by the arbitrary sign of x0, we get
    c=1/(sqrt(1+(x1/x0)^2)) and
    s=(x1/x0)/(sqrt(1+(x1/x0)^2))
    or
    f=x1/x0 followed by
    c=1/(sqrt(1+f^2)) and
    s=f*c
    */
    if (x0 != (T)0.0) {
      f = x1 / x0;
      *c = ((T)1.0) / i_sqrt(((T)1.0) + f * f);
      *s = f * (*c);
    } else {
      *c = (T)1.0;
      *s = (T)0.0;
    }
  } else {
    /*sqrt(x0^2+x1^2)= |x1| sqrt(1+(x0/x1)^2) gives
    c=x0/(|x1| sqrt(1+(x0/x1)^2)) and
    s=x1/(|x1| sqrt(1+(x0/x1)^2))
    If we multiply by the arbitrary sign of x1, we get
    c=(x0/x1)/(sqrt(1+(x0/x1)^2)) and
    s=1/(sqrt(1+(x0/x1)^2))
    or
    f=x0/x1 followed by
    s=1/(sqrt(1+f^2)) and
    c=f*s
    */
    if (x1 != (T)0.0) {
      f = x0 / x1;
      *s = ((T)1.0) / i_sqrt(((T)1.0) + f * f);
      *c = f * (*s);
    } else {
      *c = (T)1.0;
      *s = (T)0.0;
    }
  }
}

/*Apply the Givens rotation
[ c s]
[-s c]
to [x0 x1]'*/
template <typename T>
inline void i_apply_givens(T c, T s, T &x0, T &x1) {
  T t1, t2;
  t1 = c * x0 + s * x1;
  t2 = -s * x0 + c * x1;
  x0 = t1;
  x1 = t2;
}

/*Apply the Givens rotation
[ c s]
[-s c]
to [x0 x1]', but return only first element*/
template <typename T>
inline T i_apply_givens_truncated(T c, T s, T x0, T x1) {
  return (c * x0 + s * x1);
}

/*Apply Givens rotation to a matrix's row ai and aj of length n*/
template <typename T>
inline void i_apply_givens(T c, T s, T *ai, T *aj, int n) {
  for (int i = 0; i < n; i++) {
    i_apply_givens(c, s, ai[i], aj[i]);
  }
}

} /* namespace idl */