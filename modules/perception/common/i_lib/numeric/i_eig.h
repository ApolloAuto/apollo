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
#ifndef I_LIB_NUMERIC_I_EIG_H
#define I_LIB_NUMERIC_I_EIG_H

#include "i_poly.h"
#include "i_qr.h"

namespace idl {
/*Construct the characteristic polynomial equation for a 2x2 eigen system*/
template <typename T>
inline void i_characteristic_poly_2x2(const T A[4], T p[3]) {
  /*determinant of [AI - x]
  [A[0]-x     A[1]]
  [A[2]     A[3]-x]
  is x^2-(A[0]+A[3])*x+(A[0]*A[3]-A[1]*A[2])*/
  p[0] = A[0] * A[3] - A[1] * A[2];
  p[1] = -A[0] - A[3];
  p[2] = (T)1.0;
}

/*Construct the characteristic polynomial equation for a 3x3 eigen system*/
template <typename T>
inline void i_characteristic_poly_3x3(const T A[9], T p[4]) {
  /*determinant of [AI - x]
  [A[0]-x    A[1]     A[2] ]
  [A[3]      A[4]-x   A[5] ]
  [A[6]      A[7]     A[8]-x]*/
  T a4pa8 = A[4] + A[8];
  T a5a7 = A[5] * A[7];
  T a1a3 = A[1] * A[3];
  T a2a6 = A[2] * A[6];
  p[0] = (A[0] * A[4] * A[8]) + (A[1] * A[5] * A[6]) + (A[2] * A[3] * A[7]) -
         a2a6 * A[4] - a5a7 * A[0] - a1a3 * A[8];
  p[1] = a2a6 + a5a7 + a1a3 - A[0] * a4pa8 - A[4] * A[8];
  p[2] = A[0] + a4pa8; /*trace*/
  p[3] = (T)(-1.0);
}

/*Closed form solution for computing the real eigenvalues of a 3x3 matrix A
  if the number of real eigen values is less than 2, 0 will be filled in ev.
  Return
  the number of actual eigen values*/
template <typename T>
inline int i_eigenvalues_2x2_closed(const T A[4], T ev[2]) {
  T p[3];
  i_characteristic_poly_2x2(A, p);
  return i_quadratic_solve_closed(p, ev);
}

/*Closed form solution for computing the real eigenvalues of a 3x3 matrix A
  if force_three_real_eigen is true, always try for 3 eigen values. Return
  the number of actual eigen values*/
template <typename T>
inline int i_eigenvalues_3x3_closed(const T A[9], T ev[3],
                                    bool force_three_real_eigen = true) {
  T p[4];
  i_characteristic_poly_3x3(A, p);
  return i_cubic_solve_closed(p, ev, force_three_real_eigen);
}

/*Compute the n-dimensional eigen-vector corresponding to eigenvalue eval of nxn
matrix A.
The function needs n positions of scratch space in h, htA, ss, iv, and (n x n)
matrices Q and Ac.
The eigenvector is unitized and returned as the last row of Q*/
template <typename T>
inline void i_eigenvector_from_eigenvalue(const T *const *A, T ev, T **Q, T *h,
                                          T *htA, T *ss, T **Ac, int n,
                                          int *iv) {
  int i;
  i_copy(A, Ac, n, n);
  for (i = 0; i < n; i++) {
    Ac[i][i] -= ev;
  }
  i_lq_decompose_explicit(Q, Ac, h, htA, ss, n, n, iv);
}

/*Compute the n-dimensional eigen-vector corresponding to eigenvalue eval of nxn
matrix A.
The function needs n positions of scratch space in h, htA, ss, iv, and (n x n)
matrices Q and Ac.
The eigenvector is unitized and returned as the last row of Q*/
template <typename T>
inline void i_eigenvector_from_eigenvalue_3x3(const T *const *A, T ev, T **Q,
                                              T *h, T *htA, T *ss, T **Ac,
                                              int *iv) {
  Ac[0][0] = (A[0][0] - ev);
  Ac[0][1] = A[0][1];
  Ac[0][2] = A[0][2];

  Ac[1][0] = A[1][0];
  Ac[1][1] = (A[1][1] - ev);
  Ac[1][2] = A[1][2];

  Ac[2][0] = A[2][0];
  Ac[2][1] = A[2][1];
  Ac[2][2] = (A[2][2] - ev);

  i_lq_decompose_explicit_3x3(Q, Ac, h, htA, ss, iv);
}

/*Closed form solution for computing the eigen decomposition of a symmetric 2x2
 * matrix A*/
template <typename T>
inline void i_eig_symmetric_2x2_closed(const T A[4], T ev[2], T Q[4]) {
  T h[2], hta[2], ss[2];
  T ac[4], qtmp[4];
  int iv[2];
  const T *a_ref[2];
  T *ac_ref[2], *q_ref[2];
  i_make_const_reference_2x2(A, a_ref);
  i_make_reference_2x2(ac, ac_ref);
  i_make_reference_2x2(qtmp, q_ref);
  i_eigenvalues_2x2_closed(A, ev);
  if (i_abs(ev[0]) < i_abs(ev[1])) {
    i_swap(ev[0], ev[1]);
  }
  i_eigenvector_from_eigenvalue(a_ref, ev[0], q_ref, h, hta, ss, ac_ref, 2, iv);
  Q[0] = qtmp[2];
  Q[2] = qtmp[3];
  i_eigenvector_from_eigenvalue(a_ref, ev[1], q_ref, h, hta, ss, ac_ref, 2, iv);
  Q[1] = qtmp[2];
  Q[3] = qtmp[3];
}

/*Closed form solution for computing the eigen decomposition of a symmetric 3x3
 * matrix A*/
template <typename T>
inline void i_eig_symmetric_3x3_closed(const T A[9], T ev[3], T Q[9]) {
  T h[3], hta[3], ss[3];
  T ac[9], qtmp[9];
  int iv[3];
  const T *a_ref[3];
  T *ac_ref[3], *q_ref[3];
  i_make_const_reference_3x3(A, a_ref);
  i_make_reference_3x3(ac, ac_ref);
  i_make_reference_3x3(qtmp, q_ref);
  i_eigenvalues_3x3_closed(A, ev);
  if (i_abs(ev[0]) < i_abs(ev[1])) {
    i_swap(ev[0], ev[1]);
  }
  if (i_abs(ev[0]) < i_abs(ev[2])) {
    i_swap(ev[0], ev[2]);
  }
  i_eigenvector_from_eigenvalue_3x3(a_ref, ev[0], q_ref, h, hta, ss, ac_ref,
                                    iv);
  Q[0] = qtmp[6];
  Q[3] = qtmp[7];
  Q[6] = qtmp[8];
  if (i_abs(ev[1]) < i_abs(ev[2])) {
    i_swap(ev[1], ev[2]);
  }
  i_eigenvector_from_eigenvalue_3x3(a_ref, ev[1], q_ref, h, hta, ss, ac_ref,
                                    iv);
  Q[1] = qtmp[6];
  Q[4] = qtmp[7];
  Q[7] = qtmp[8];
  i_eigenvector_from_eigenvalue_3x3(a_ref, ev[2], q_ref, h, hta, ss, ac_ref,
                                    iv);
  Q[2] = qtmp[6];
  Q[5] = qtmp[7];
  Q[8] = qtmp[8];
}

} /* namespace idl */

#endif