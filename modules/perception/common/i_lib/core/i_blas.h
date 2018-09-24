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

#include "i_basic.h"

namespace idl {

/*Copy of 1D arrays*/
template <typename T>
inline void i_copy(const T *src, T *dst, int n) {
  memcpy(dst, src, n * sizeof(T));
}
template <typename T>
inline void i_copy1(const T src[1], T dst[1]) {
  dst[0] = src[0];
}
template <typename T>
inline void i_copy2(const T src[2], T dst[2]) {
  dst[0] = src[0];
  dst[1] = src[1];
}
template <typename T>
inline void i_copy3(const T src[3], T dst[3]) {
  dst[0] = src[0];
  dst[1] = src[1];
  dst[2] = src[2];
}
template <typename T>
inline void i_copy4(const T src[4], T dst[4]) {
  dst[0] = src[0];
  dst[1] = src[1];
  dst[2] = src[2];
  dst[3] = src[3];
}
template <typename T>
inline void i_copy5(const T src[5], T dst[5]) {
  dst[0] = src[0];
  dst[1] = src[1];
  dst[2] = src[2];
  dst[3] = src[3];
  dst[4] = src[4];
}
template <typename T>
inline void i_copy6(const T src[6], T dst[6]) {
  dst[0] = src[0];
  dst[1] = src[1];
  dst[2] = src[2];
  dst[3] = src[3];
  dst[4] = src[4];
  dst[5] = src[5];
}
template <typename T>
inline void i_copy7(const T src[7], T dst[7]) {
  dst[0] = src[0];
  dst[1] = src[1];
  dst[2] = src[2];
  dst[3] = src[3];
  dst[4] = src[4];
  dst[5] = src[5];
  dst[6] = src[6];
}
template <typename T>
inline void i_copy8(const T src[8], T dst[8]) {
  dst[0] = src[0];
  dst[1] = src[1];
  dst[2] = src[2];
  dst[3] = src[3];
  dst[4] = src[4];
  dst[5] = src[5];
  dst[6] = src[6];
  dst[7] = src[7];
}
template <typename T>
inline void i_copy9(const T src[9], T dst[9]) {
  dst[0] = src[0];
  dst[1] = src[1];
  dst[2] = src[2];
  dst[3] = src[3];
  dst[4] = src[4];
  dst[5] = src[5];
  dst[6] = src[6];
  dst[7] = src[7];
  dst[8] = src[8];
}
template <typename T>
inline void i_copy10(const T src[10], T dst[10]) {
  dst[0] = src[0];
  dst[1] = src[1];
  dst[2] = src[2];
  dst[3] = src[3];
  dst[4] = src[4];
  dst[5] = src[5];
  dst[6] = src[6];
  dst[7] = src[7];
  dst[8] = src[8];
  dst[9] = src[9];
}
template <typename T>
inline void i_copy11(const T src[11], T dst[11]) {
  dst[0] = src[0];
  dst[1] = src[1];
  dst[2] = src[2];
  dst[3] = src[3];
  dst[4] = src[4];
  dst[5] = src[5];
  dst[6] = src[6];
  dst[7] = src[7];
  dst[8] = src[8];
  dst[9] = src[9];
  dst[10] = src[10];
}
template <typename T>
inline void i_copy12(const T src[12], T dst[12]) {
  dst[0] = src[0];
  dst[1] = src[1];
  dst[2] = src[2];
  dst[3] = src[3];
  dst[4] = src[4];
  dst[5] = src[5];
  dst[6] = src[6];
  dst[7] = src[7];
  dst[8] = src[8];
  dst[9] = src[9];
  dst[10] = src[10];
  dst[11] = src[11];
}
template <typename T>
inline void i_copy13(const T src[13], T dst[13]) {
  dst[0] = src[0];
  dst[1] = src[1];
  dst[2] = src[2];
  dst[3] = src[3];
  dst[4] = src[4];
  dst[5] = src[5];
  dst[6] = src[6];
  dst[7] = src[7];
  dst[8] = src[8];
  dst[9] = src[9];
  dst[10] = src[10];
  dst[11] = src[11];
  dst[12] = src[12];
}
template <typename T>
inline void i_copy14(const T src[14], T dst[14]) {
  dst[0] = src[0];
  dst[1] = src[1];
  dst[2] = src[2];
  dst[3] = src[3];
  dst[4] = src[4];
  dst[5] = src[5];
  dst[6] = src[6];
  dst[7] = src[7];
  dst[8] = src[8];
  dst[9] = src[9];
  dst[10] = src[10];
  dst[11] = src[11];
  dst[12] = src[12];
  dst[13] = src[13];
}
template <typename T>
inline void i_copy15(const T src[15], T dst[15]) {
  dst[0] = src[0];
  dst[1] = src[1];
  dst[2] = src[2];
  dst[3] = src[3];
  dst[4] = src[4];
  dst[5] = src[5];
  dst[6] = src[6];
  dst[7] = src[7];
  dst[8] = src[8];
  dst[9] = src[9];
  dst[10] = src[10];
  dst[11] = src[11];
  dst[12] = src[12];
  dst[13] = src[13];
  dst[14] = src[14];
}
template <typename T>
inline void i_copy16(const T src[16], T dst[16]) {
  dst[0] = src[0];
  dst[1] = src[1];
  dst[2] = src[2];
  dst[3] = src[3];
  dst[4] = src[4];
  dst[5] = src[5];
  dst[6] = src[6];
  dst[7] = src[7];
  dst[8] = src[8];
  dst[9] = src[9];
  dst[10] = src[10];
  dst[11] = src[11];
  dst[12] = src[12];
  dst[13] = src[13];
  dst[14] = src[14];
  dst[15] = src[15];
}
/*Copy of 2D arrays*/
template <typename T>
inline void i_copy(const T *const *src, T **dst, int m, int n) {
  int i;
  for (i = 0; i < m; i++) i_copy<T>(src[i], dst[i], n);
}

/*Copy of 1D arrays with different types*/
template <typename T, typename S>
inline void i_copy(const T *src, S *dst, int n) {
  int i;
  for (i = 0; i < n; ++i) {
    dst[i] = (S)(src[i]);
  }
}
/*Copy of 2D arrays with different types*/
template <typename T, typename S>
inline void i_copy(const T *const *src, S **dst, int m, int n) {
  int i;
  for (i = 0; i < m; i++) i_copy<T, S>(src[i], dst[i], n);
}

/*Fill array elements with constant value*/
template <typename T>
inline void i_fill(T *a, int n, T val) {
  for (int i = 0; i < n; i++) a[i] = val;
}
template <typename T>
inline void i_fill1(T a[1], T val) {
  a[0] = val;
}
template <typename T>
inline void i_fill2(T a[2], T val) {
  a[0] = a[1] = val;
}
template <typename T>
inline void i_fill3(T a[3], T val) {
  a[0] = a[1] = a[2] = val;
}
template <typename T>
inline void i_fill4(T a[4], T val) {
  a[0] = a[1] = a[2] = a[3] = val;
}
template <typename T>
inline void i_fill5(T a[5], T val) {
  a[0] = a[1] = a[2] = a[3] = a[4] = val;
}
template <typename T>
inline void i_fill6(T a[6], T val) {
  a[0] = a[1] = a[2] = a[3] = a[4] = a[5] = val;
}
template <typename T>
inline void i_fill7(T a[7], T val) {
  a[0] = a[1] = a[2] = a[3] = a[4] = a[5] = a[6] = val;
}
template <typename T>
inline void i_fill8(T a[8], T val) {
  a[0] = a[1] = a[2] = a[3] = a[4] = a[5] = a[6] = a[7] = val;
}
template <typename T>
inline void i_fill9(T a[9], T val) {
  a[0] = a[1] = a[2] = a[3] = a[4] = a[5] = a[6] = a[7] = a[8] = val;
}
template <typename T>
inline void i_fill10(T a[10], T val) {
  a[0] = a[1] = a[2] = a[3] = a[4] = a[5] = a[6] = a[7] = a[8] = a[9] = val;
}
template <typename T>
inline void i_fill11(T a[11], T val) {
  a[0] = a[1] = a[2] = a[3] = a[4] = a[5] = a[6] = a[7] = a[8] = a[9] = a[10] =
      val;
}
template <typename T>
inline void i_fill12(T a[12], T val) {
  a[0] = a[1] = a[2] = a[3] = a[4] = a[5] = a[6] = a[7] = a[8] = a[9] = a[10] =
      a[11] = val;
}
template <typename T>
inline void i_fill13(T a[13], T val) {
  a[0] = a[1] = a[2] = a[3] = a[4] = a[5] = a[6] = a[7] = a[8] = a[9] = a[10] =
      a[11] = a[12] = val;
}
template <typename T>
inline void i_fill14(T a[14], T val) {
  a[0] = a[1] = a[2] = a[3] = a[4] = a[5] = a[6] = a[7] = a[8] = a[9] = a[10] =
      a[11] = a[12] = a[13] = val;
}
template <typename T>
inline void i_fill15(T a[15], T val) {
  a[0] = a[1] = a[2] = a[3] = a[4] = a[5] = a[6] = a[7] = a[8] = a[9] = a[10] =
      a[11] = a[12] = a[13] = a[14] = val;
}
template <typename T>
inline void i_fill16(T a[16], T val) {
  a[0] = a[1] = a[2] = a[3] = a[4] = a[5] = a[6] = a[7] = a[8] = a[9] = a[10] =
      a[11] = a[12] = a[13] = a[14] = a[15] = val;
}

/*Fill array elements with zeroes*/
template <typename T>
inline void i_zero(T *a, int n) {
  for (int i = 0; i < n; i++) a[i] = (T)0.0;
}
template <typename T>
inline void i_zero1(T a[1]) {
  a[0] = (T)0.0;
}
template <typename T>
inline void i_zero2(T a[2]) {
  a[0] = a[1] = (T)0.0;
}
template <typename T>
inline void i_zero3(T a[3]) {
  a[0] = a[1] = a[2] = (T)0.0;
}
template <typename T>
inline void i_zero4(T a[4]) {
  a[0] = a[1] = a[2] = a[3] = (T)0.0;
}
template <typename T>
inline void i_zero5(T a[5]) {
  a[0] = a[1] = a[2] = a[3] = a[4] = (T)0.0;
}
template <typename T>
inline void i_zero6(T a[6]) {
  a[0] = a[1] = a[2] = a[3] = a[4] = a[5] = (T)0.0;
}
template <typename T>
inline void i_zero7(T a[7]) {
  a[0] = a[1] = a[2] = a[3] = a[4] = a[5] = a[6] = (T)0.0;
}
template <typename T>
inline void i_zero8(T a[8]) {
  a[0] = a[1] = a[2] = a[3] = a[4] = a[5] = a[6] = a[7] = (T)0.0;
}
template <typename T>
inline void i_zero9(T a[9]) {
  a[0] = a[1] = a[2] = a[3] = a[4] = a[5] = a[6] = a[7] = a[8] = (T)0.0;
}
template <typename T>
inline void i_zero10(T a[10]) {
  a[0] = a[1] = a[2] = a[3] = a[4] = a[5] = a[6] = a[7] = a[8] = a[9] = (T)0.0;
}
template <typename T>
inline void i_zero11(T a[11]) {
  a[0] = a[1] = a[2] = a[3] = a[4] = a[5] = a[6] = a[7] = a[8] = a[9] = a[10] =
      (T)0.0;
}
template <typename T>
inline void i_zero12(T a[12]) {
  a[0] = a[1] = a[2] = a[3] = a[4] = a[5] = a[6] = a[7] = a[8] = a[9] = a[10] =
      a[11] = (T)0.0;
}
template <typename T>
inline void i_zero13(T a[13]) {
  a[0] = a[1] = a[2] = a[3] = a[4] = a[5] = a[6] = a[7] = a[8] = a[9] = a[10] =
      a[11] = a[12] = (T)0.0;
}
template <typename T>
inline void i_zero14(T a[14]) {
  a[0] = a[1] = a[2] = a[3] = a[4] = a[5] = a[6] = a[7] = a[8] = a[9] = a[10] =
      a[11] = a[12] = a[13] = (T)0.0;
}
template <typename T>
inline void i_zero15(T a[15]) {
  a[0] = a[1] = a[2] = a[3] = a[4] = a[5] = a[6] = a[7] = a[8] = a[9] = a[10] =
      a[11] = a[12] = a[13] = a[14] = (T)0.0;
}
template <typename T>
inline void i_zero16(T a[16]) {
  a[0] = a[1] = a[2] = a[3] = a[4] = a[5] = a[6] = a[7] = a[8] = a[9] = a[10] =
      a[11] = a[12] = a[13] = a[14] = a[15] = (T)0.0;
}

/*Check, for i=0 to n-1, if a[i] = val for a n-dimensional vector a, if all
 * elements are equal to val, return true, otherwise false*/
template <typename T>
inline bool i_equal(const T *a, int n, T val) {
  int i = 0;
  for (; i < n; ++i) {
    if (a[i] != val) {
      break;
    }
  }
  return (i < n) ? false : true;
}
template <typename T>
inline bool i_equal2(const T a[2], T val) {
  if (a[0] == val && a[1] == val) {
    return true;
  } else
    return false;
}
template <typename T>
inline bool i_equal3(const T a[3], T val) {
  if (a[0] == val && a[1] == val && a[2] == val) {
    return true;
  } else
    return false;
}
template <typename T>
inline bool i_equal4(const T a[4], T val) {
  if (a[0] == val && a[1] == val && a[2] == val && a[3] == val) {
    return true;
  } else
    return false;
}

template <typename T>
inline bool i_equal(const T *a, const T *b, int n) {
  int i = 0;
  for (; i < n; ++i) {
    if (a[i] != b[i]) {
      break;
    }
  }
  return (i < n) ? false : true;
}
template <typename T>
inline bool i_equal2(const T a[2], const T b[2]) {
  if (a[0] == b[0] && a[1] == b[1]) {
    return true;
  } else
    return false;
}
template <typename T>
inline bool i_equal3(const T a[3], const T b[3]) {
  if (a[0] == b[0] && a[1] == b[1] && a[2] == b[2]) {
    return true;
  } else
    return false;
}
template <typename T>
inline bool i_equal4(const T a[4], const T b[4]) {
  if (a[0] == b[0] && a[1] == b[1] && a[2] == b[2] && a[3] == b[3]) {
    return true;
  } else
    return false;
}

template <typename T>
inline bool i_equal_identity_2x2(const T a[4]) {
  if (a[0] == (T)1.0 && a[1] == (T)0.0 && a[2] == (T)0.0 && a[3] == (T)1.0) {
    return true;
  } else
    return false;
}

template <typename T>
inline bool i_equal_identity_3x3(const T a[9]) {
  if (a[0] == (T)1.0 && a[1] == (T)0.0 && a[2] == (T)0.0 && a[3] == (T)0.0 &&
      a[4] == (T)1.0 && a[5] == (T)0.0 && a[6] == (T)0.0 && a[7] == (T)0.0 &&
      a[8] == (T)1.0) {
    return true;
  } else
    return false;
}

/*Negate a vector x of length n inplace*/
template <typename T>
inline void i_neg(T *x, int n) {
  for (int i = 0; i < n; i++) {
    x[i] = -x[i];
  }
}
template <typename T>
inline void i_neg1(T x[1]) {
  x[0] = -x[0];
}
template <typename T>
inline void i_neg2(T x[2]) {
  x[0] = -x[0];
  x[1] = -x[1];
}
template <typename T>
inline void i_neg3(T x[3]) {
  x[0] = -x[0];
  x[1] = -x[1];
  x[2] = -x[2];
}
template <typename T>
inline void i_neg4(T x[4]) {
  x[0] = -x[0];
  x[1] = -x[1];
  x[2] = -x[2];
  x[3] = -x[3];
}
template <typename T>
inline void i_neg5(T x[5]) {
  x[0] = -x[0];
  x[1] = -x[1];
  x[2] = -x[2];
  x[3] = -x[3];
  x[4] = -x[4];
}
template <typename T>
inline void i_neg6(T x[6]) {
  x[0] = -x[0];
  x[1] = -x[1];
  x[2] = -x[2];
  x[3] = -x[3];
  x[4] = -x[4];
  x[5] = -x[5];
}
template <typename T>
inline void i_neg7(T x[7]) {
  x[0] = -x[0];
  x[1] = -x[1];
  x[2] = -x[2];
  x[3] = -x[3];
  x[4] = -x[4];
  x[5] = -x[5];
  x[6] = -x[6];
}
template <typename T>
inline void i_neg8(T x[8]) {
  x[0] = -x[0];
  x[1] = -x[1];
  x[2] = -x[2];
  x[3] = -x[3];
  x[4] = -x[4];
  x[5] = -x[5];
  x[6] = -x[6];
  x[7] = -x[7];
}
template <typename T>
inline void i_neg9(T x[9]) {
  x[0] = -x[0];
  x[1] = -x[1];
  x[2] = -x[2];
  x[3] = -x[3];
  x[4] = -x[4];
  x[5] = -x[5];
  x[6] = -x[6];
  x[7] = -x[7];
  x[8] = -x[8];
}
template <typename T>
inline void i_neg10(T x[10]) {
  x[0] = -x[0];
  x[1] = -x[1];
  x[2] = -x[2];
  x[3] = -x[3];
  x[4] = -x[4];
  x[5] = -x[5];
  x[6] = -x[6];
  x[7] = -x[7];
  x[8] = -x[8];
  x[9] = -x[9];
}
template <typename T>
inline void i_neg11(T x[11]) {
  x[0] = -x[0];
  x[1] = -x[1];
  x[2] = -x[2];
  x[3] = -x[3];
  x[4] = -x[4];
  x[5] = -x[5];
  x[6] = -x[6];
  x[7] = -x[7];
  x[8] = -x[8];
  x[9] = -x[9];
  x[10] = -x[10];
}
template <typename T>
inline void i_neg12(T x[12]) {
  x[0] = -x[0];
  x[1] = -x[1];
  x[2] = -x[2];
  x[3] = -x[3];
  x[4] = -x[4];
  x[5] = -x[5];
  x[6] = -x[6];
  x[7] = -x[7];
  x[8] = -x[8];
  x[9] = -x[9];
  x[10] = -x[10];
  x[11] = -x[11];
}
template <typename T>
inline void i_neg13(T x[13]) {
  x[0] = -x[0];
  x[1] = -x[1];
  x[2] = -x[2];
  x[3] = -x[3];
  x[4] = -x[4];
  x[5] = -x[5];
  x[6] = -x[6];
  x[7] = -x[7];
  x[8] = -x[8];
  x[9] = -x[9];
  x[10] = -x[10];
  x[11] = -x[11];
  x[12] = -x[12];
}
template <typename T>
inline void i_neg14(T x[14]) {
  x[0] = -x[0];
  x[1] = -x[1];
  x[2] = -x[2];
  x[3] = -x[3];
  x[4] = -x[4];
  x[5] = -x[5];
  x[6] = -x[6];
  x[7] = -x[7];
  x[8] = -x[8];
  x[9] = -x[9];
  x[10] = -x[10];
  x[11] = -x[11];
  x[12] = -x[12];
  x[13] = -x[13];
}
template <typename T>
inline void i_neg15(T x[15]) {
  x[0] = -x[0];
  x[1] = -x[1];
  x[2] = -x[2];
  x[3] = -x[3];
  x[4] = -x[4];
  x[5] = -x[5];
  x[6] = -x[6];
  x[7] = -x[7];
  x[8] = -x[8];
  x[9] = -x[9];
  x[10] = -x[10];
  x[11] = -x[11];
  x[12] = -x[12];
  x[13] = -x[13];
  x[14] = -x[14];
}
template <typename T>
inline void i_neg16(T x[16]) {
  x[0] = -x[0];
  x[1] = -x[1];
  x[2] = -x[2];
  x[3] = -x[3];
  x[4] = -x[4];
  x[5] = -x[5];
  x[6] = -x[6];
  x[7] = -x[7];
  x[8] = -x[8];
  x[9] = -x[9];
  x[10] = -x[10];
  x[11] = -x[11];
  x[12] = -x[12];
  x[13] = -x[13];
  x[14] = -x[14];
  x[15] = -x[15];
}
/*Negate a vector x of length n, save results in a vector y*/
template <typename T>
inline void i_neg1(const T x[1], T y[1]) {
  y[0] = -x[0];
}
template <typename T>
inline void i_neg2(const T x[2], T y[2]) {
  y[0] = -x[0];
  y[1] = -x[1];
}
template <typename T>
inline void i_neg3(const T x[3], T y[3]) {
  y[0] = -x[0];
  y[1] = -x[1];
  y[2] = -x[2];
}
template <typename T>
inline void i_neg4(const T x[4], T y[4]) {
  y[0] = -x[0];
  y[1] = -x[1];
  y[2] = -x[2];
  y[3] = -x[3];
}
template <typename T>
inline void i_neg5(const T x[5], T y[5]) {
  y[0] = -x[0];
  y[1] = -x[1];
  y[2] = -x[2];
  y[3] = -x[3];
  y[4] = -x[4];
}
template <typename T>
inline void i_neg6(const T x[6], T y[6]) {
  y[0] = -x[0];
  y[1] = -x[1];
  y[2] = -x[2];
  y[3] = -x[3];
  y[4] = -x[4];
  y[5] = -x[5];
}
template <typename T>
inline void i_neg7(const T x[7], T y[7]) {
  y[0] = -x[0];
  y[1] = -x[1];
  y[2] = -x[2];
  y[3] = -x[3];
  y[4] = -x[4];
  y[5] = -x[5];
  y[6] = -x[6];
}
template <typename T>
inline void i_neg8(const T x[8], T y[8]) {
  y[0] = -x[0];
  y[1] = -x[1];
  y[2] = -x[2];
  y[3] = -x[3];
  y[4] = -x[4];
  y[5] = -x[5];
  y[6] = -x[6];
  y[7] = -x[7];
}
template <typename T>
inline void i_neg9(const T x[9], T y[9]) {
  y[0] = -x[0];
  y[1] = -x[1];
  y[2] = -x[2];
  y[3] = -x[3];
  y[4] = -x[4];
  y[5] = -x[5];
  y[6] = -x[6];
  y[7] = -x[7];
  y[8] = -x[8];
}
template <typename T>
inline void i_neg10(const T x[10], T y[10]) {
  y[0] = -x[0];
  y[1] = -x[1];
  y[2] = -x[2];
  y[3] = -x[3];
  y[4] = -x[4];
  y[5] = -x[5];
  y[6] = -x[6];
  y[7] = -x[7];
  y[8] = -x[8];
  y[9] = -x[9];
}
template <typename T>
inline void i_neg11(const T x[11], T y[11]) {
  y[0] = -x[0];
  y[1] = -x[1];
  y[2] = -x[2];
  y[3] = -x[3];
  y[4] = -x[4];
  y[5] = -x[5];
  y[6] = -x[6];
  y[7] = -x[7];
  y[8] = -x[8];
  y[9] = -x[9];
  y[10] = -x[10];
}
template <typename T>
inline void i_neg12(const T x[12], T y[12]) {
  y[0] = -x[0];
  y[1] = -x[1];
  y[2] = -x[2];
  y[3] = -x[3];
  y[4] = -x[4];
  y[5] = -x[5];
  y[6] = -x[6];
  y[7] = -x[7];
  y[8] = -x[8];
  y[9] = -x[9];
  y[10] = -x[10];
  y[11] = -x[11];
}
template <typename T>
inline void i_neg13(const T x[13], T y[13]) {
  y[0] = -x[0];
  y[1] = -x[1];
  y[2] = -x[2];
  y[3] = -x[3];
  y[4] = -x[4];
  y[5] = -x[5];
  y[6] = -x[6];
  y[7] = -x[7];
  y[8] = -x[8];
  y[9] = -x[9];
  y[10] = -x[10];
  y[11] = -x[11];
  y[12] = -x[12];
}
template <typename T>
inline void i_neg14(const T x[14], T y[14]) {
  y[0] = -x[0];
  y[1] = -x[1];
  y[2] = -x[2];
  y[3] = -x[3];
  y[4] = -x[4];
  y[5] = -x[5];
  y[6] = -x[6];
  y[7] = -x[7];
  y[8] = -x[8];
  y[9] = -x[9];
  y[10] = -x[10];
  y[11] = -x[11];
  y[12] = -x[12];
  y[13] = -x[13];
}
template <typename T>
inline void i_neg15(const T x[15], T y[15]) {
  y[0] = -x[0];
  y[1] = -x[1];
  y[2] = -x[2];
  y[3] = -x[3];
  y[4] = -x[4];
  y[5] = -x[5];
  y[6] = -x[6];
  y[7] = -x[7];
  y[8] = -x[8];
  y[9] = -x[9];
  y[10] = -x[10];
  y[11] = -x[11];
  y[12] = -x[12];
  y[13] = -x[13];
  y[14] = -x[14];
}
template <typename T>
inline void i_neg16(const T x[16], T y[16]) {
  y[0] = -x[0];
  y[1] = -x[1];
  y[2] = -x[2];
  y[3] = -x[3];
  y[4] = -x[4];
  y[5] = -x[5];
  y[6] = -x[6];
  y[7] = -x[7];
  y[8] = -x[8];
  y[9] = -x[9];
  y[10] = -x[10];
  y[11] = -x[11];
  y[12] = -x[12];
  y[13] = -x[13];
  y[14] = -x[14];
  y[15] = -x[15];
}

/*Negate the cth column of mxn matrix A*/
template <typename T>
inline void i_neg_col(T *A, int c, int m, int n) {
  T *ref = A;
  for (int r = 0; r < m; ++r, ref += n) {
    ref[c] = -ref[c];
  }
}

/*Compute x=x+c where x is n-dimensional vectors and c is a constant*/
template <typename T>
inline void i_add(T *x, int n, T k) {
  for (int i = 0; i < n; i++) {
    x[i] += k;
  }
}
/*Compute z=x+y where x, y, and z are n-dimensional vectors*/
template <typename T>
inline void i_add(const T *x, const T *y, int n, T *z) {
  for (int i = 0; i < n; i++) {
    z[i] = x[i] + y[i];
  }
}
template <typename T>
inline void i_add1(const T x[1], const T y[1], T z[1]) {
  z[0] = x[0] + y[0];
}
template <typename T>
inline void i_add2(const T x[2], const T y[2], T z[2]) {
  z[0] = x[0] + y[0];
  z[1] = x[1] + y[1];
}
template <typename T>
inline void i_add3(const T x[3], const T y[3], T z[3]) {
  z[0] = x[0] + y[0];
  z[1] = x[1] + y[1];
  z[2] = x[2] + y[2];
}
template <typename T>
inline void i_add4(const T x[4], const T y[4], T z[4]) {
  z[0] = x[0] + y[0];
  z[1] = x[1] + y[1];
  z[2] = x[2] + y[2];
  z[3] = x[3] + y[3];
}
template <typename T>
inline void i_add5(const T x[5], const T y[5], T z[5]) {
  z[0] = x[0] + y[0];
  z[1] = x[1] + y[1];
  z[2] = x[2] + y[2];
  z[3] = x[3] + y[3];
  z[4] = x[4] + y[4];
}
template <typename T>
inline void i_add6(const T x[6], const T y[6], T z[6]) {
  z[0] = x[0] + y[0];
  z[1] = x[1] + y[1];
  z[2] = x[2] + y[2];
  z[3] = x[3] + y[3];
  z[4] = x[4] + y[4];
  z[5] = x[5] + y[5];
}
template <typename T>
inline void i_add7(const T x[7], const T y[7], T z[7]) {
  z[0] = x[0] + y[0];
  z[1] = x[1] + y[1];
  z[2] = x[2] + y[2];
  z[3] = x[3] + y[3];
  z[4] = x[4] + y[4];
  z[5] = x[5] + y[5];
  z[6] = x[6] + y[6];
}
template <typename T>
inline void i_add8(const T x[8], const T y[8], T z[8]) {
  z[0] = x[0] + y[0];
  z[1] = x[1] + y[1];
  z[2] = x[2] + y[2];
  z[3] = x[3] + y[3];
  z[4] = x[4] + y[4];
  z[5] = x[5] + y[5];
  z[6] = x[6] + y[6];
  z[7] = x[7] + y[7];
}
template <typename T>
inline void i_add9(const T x[9], const T y[9], T z[9]) {
  z[0] = x[0] + y[0];
  z[1] = x[1] + y[1];
  z[2] = x[2] + y[2];
  z[3] = x[3] + y[3];
  z[4] = x[4] + y[4];
  z[5] = x[5] + y[5];
  z[6] = x[6] + y[6];
  z[7] = x[7] + y[7];
  z[8] = x[8] + y[8];
}
template <typename T>
inline void i_add10(const T x[10], const T y[10], T z[10]) {
  z[0] = x[0] + y[0];
  z[1] = x[1] + y[1];
  z[2] = x[2] + y[2];
  z[3] = x[3] + y[3];
  z[4] = x[4] + y[4];
  z[5] = x[5] + y[5];
  z[6] = x[6] + y[6];
  z[7] = x[7] + y[7];
  z[8] = x[8] + y[8];
  z[9] = x[9] + y[9];
}
template <typename T>
inline void i_add11(const T x[11], const T y[11], T z[11]) {
  z[0] = x[0] + y[0];
  z[1] = x[1] + y[1];
  z[2] = x[2] + y[2];
  z[3] = x[3] + y[3];
  z[4] = x[4] + y[4];
  z[5] = x[5] + y[5];
  z[6] = x[6] + y[6];
  z[7] = x[7] + y[7];
  z[8] = x[8] + y[8];
  z[9] = x[9] + y[9];
  z[10] = x[10] + y[10];
}
template <typename T>
inline void i_add12(const T x[12], const T y[12], T z[12]) {
  z[0] = x[0] + y[0];
  z[1] = x[1] + y[1];
  z[2] = x[2] + y[2];
  z[3] = x[3] + y[3];
  z[4] = x[4] + y[4];
  z[5] = x[5] + y[5];
  z[6] = x[6] + y[6];
  z[7] = x[7] + y[7];
  z[8] = x[8] + y[8];
  z[9] = x[9] + y[9];
  z[10] = x[10] + y[10];
  z[11] = x[11] + y[11];
}
template <typename T>
inline void i_add13(const T x[13], const T y[13], T z[13]) {
  z[0] = x[0] + y[0];
  z[1] = x[1] + y[1];
  z[2] = x[2] + y[2];
  z[3] = x[3] + y[3];
  z[4] = x[4] + y[4];
  z[5] = x[5] + y[5];
  z[6] = x[6] + y[6];
  z[7] = x[7] + y[7];
  z[8] = x[8] + y[8];
  z[9] = x[9] + y[9];
  z[10] = x[10] + y[10];
  z[11] = x[11] + y[11];
  z[12] = x[12] + y[12];
}
template <typename T>
inline void i_add14(const T x[14], const T y[14], T z[14]) {
  z[0] = x[0] + y[0];
  z[1] = x[1] + y[1];
  z[2] = x[2] + y[2];
  z[3] = x[3] + y[3];
  z[4] = x[4] + y[4];
  z[5] = x[5] + y[5];
  z[6] = x[6] + y[6];
  z[7] = x[7] + y[7];
  z[8] = x[8] + y[8];
  z[9] = x[9] + y[9];
  z[10] = x[10] + y[10];
  z[11] = x[11] + y[11];
  z[12] = x[12] + y[12];
  z[13] = x[13] + y[13];
}
template <typename T>
inline void i_add15(const T x[15], const T y[15], T z[15]) {
  z[0] = x[0] + y[0];
  z[1] = x[1] + y[1];
  z[2] = x[2] + y[2];
  z[3] = x[3] + y[3];
  z[4] = x[4] + y[4];
  z[5] = x[5] + y[5];
  z[6] = x[6] + y[6];
  z[7] = x[7] + y[7];
  z[8] = x[8] + y[8];
  z[9] = x[9] + y[9];
  z[10] = x[10] + y[10];
  z[11] = x[11] + y[11];
  z[12] = x[12] + y[12];
  z[13] = x[13] + y[13];
  z[14] = x[14] + y[14];
}
template <typename T>
inline void i_add16(const T x[16], const T y[16], T z[16]) {
  z[0] = x[0] + y[0];
  z[1] = x[1] + y[1];
  z[2] = x[2] + y[2];
  z[3] = x[3] + y[3];
  z[4] = x[4] + y[4];
  z[5] = x[5] + y[5];
  z[6] = x[6] + y[6];
  z[7] = x[7] + y[7];
  z[8] = x[8] + y[8];
  z[9] = x[9] + y[9];
  z[10] = x[10] + y[10];
  z[11] = x[11] + y[11];
  z[12] = x[12] + y[12];
  z[13] = x[13] + y[13];
  z[14] = x[14] + y[14];
  z[15] = x[15] + y[15];
}
template <typename T>
inline void i_add20(const T x[16], const T y[16], T z[16]) {
  z[0] = x[0] + y[0];
  z[1] = x[1] + y[1];
  z[2] = x[2] + y[2];
  z[3] = x[3] + y[3];
  z[4] = x[4] + y[4];
  z[5] = x[5] + y[5];
  z[6] = x[6] + y[6];
  z[7] = x[7] + y[7];
  z[8] = x[8] + y[8];
  z[9] = x[9] + y[9];
  z[10] = x[10] + y[10];
  z[11] = x[11] + y[11];
  z[12] = x[12] + y[12];
  z[13] = x[13] + y[13];
  z[14] = x[14] + y[14];
  z[15] = x[15] + y[15];
  z[16] = x[16] + y[16];
  z[17] = x[17] + y[17];
  z[18] = x[18] + y[18];
  z[19] = x[19] + y[19];
}

/*Compute y=y+x where x and y are n-dimensional vectors*/
template <typename T>
inline void i_add(const T *x, T *y, int n) {
  for (int i = 0; i < n; i++) {
    y[i] += x[i];
  }
}
template <typename T>
inline void i_add1(const T x[1], T y[1]) {
  y[0] += x[0];
}
template <typename T>
inline void i_add2(const T x[2], T y[2]) {
  y[0] += x[0];
  y[1] += x[1];
}
template <typename T>
inline void i_add3(const T x[3], T y[3]) {
  y[0] += x[0];
  y[1] += x[1];
  y[2] += x[2];
}
template <typename T>
inline void i_add4(const T x[4], T y[4]) {
  y[0] += x[0];
  y[1] += x[1];
  y[2] += x[2];
  y[3] += x[3];
}
template <typename T>
inline void i_add5(const T x[5], T y[5]) {
  y[0] += x[0];
  y[1] += x[1];
  y[2] += x[2];
  y[3] += x[3];
  y[4] += x[4];
}
template <typename T>
inline void i_add6(const T x[6], T y[6]) {
  y[0] += x[0];
  y[1] += x[1];
  y[2] += x[2];
  y[3] += x[3];
  y[4] += x[4];
  y[5] += x[5];
}
template <typename T>
inline void i_add7(const T x[7], T y[7]) {
  y[0] += x[0];
  y[1] += x[1];
  y[2] += x[2];
  y[3] += x[3];
  y[4] += x[4];
  y[5] += x[5];
  y[6] += x[6];
}
template <typename T>
inline void i_add8(const T x[8], T y[8]) {
  y[0] += x[0];
  y[1] += x[1];
  y[2] += x[2];
  y[3] += x[3];
  y[4] += x[4];
  y[5] += x[5];
  y[6] += x[6];
  y[7] += x[7];
}
template <typename T>
inline void i_add9(const T x[9], T y[9]) {
  y[0] += x[0];
  y[1] += x[1];
  y[2] += x[2];
  y[3] += x[3];
  y[4] += x[4];
  y[5] += x[5];
  y[6] += x[6];
  y[7] += x[7];
  y[8] += x[8];
}
template <typename T>
inline void i_add10(const T x[10], T y[10]) {
  y[0] += x[0];
  y[1] += x[1];
  y[2] += x[2];
  y[3] += x[3];
  y[4] += x[4];
  y[5] += x[5];
  y[6] += x[6];
  y[7] += x[7];
  y[8] += x[8];
  y[9] += x[9];
}
template <typename T>
inline void i_add11(const T x[11], T y[11]) {
  y[0] += x[0];
  y[1] += x[1];
  y[2] += x[2];
  y[3] += x[3];
  y[4] += x[4];
  y[5] += x[5];
  y[6] += x[6];
  y[7] += x[7];
  y[8] += x[8];
  y[9] += x[9];
  y[10] += x[10];
}
template <typename T>
inline void i_add12(const T x[12], T y[12]) {
  y[0] += x[0];
  y[1] += x[1];
  y[2] += x[2];
  y[3] += x[3];
  y[4] += x[4];
  y[5] += x[5];
  y[6] += x[6];
  y[7] += x[7];
  y[8] += x[8];
  y[9] += x[9];
  y[10] += x[10];
  y[11] += x[11];
}
template <typename T>
inline void i_add13(const T x[13], T y[13]) {
  y[0] += x[0];
  y[1] += x[1];
  y[2] += x[2];
  y[3] += x[3];
  y[4] += x[4];
  y[5] += x[5];
  y[6] += x[6];
  y[7] += x[7];
  y[8] += x[8];
  y[9] += x[9];
  y[10] += x[10];
  y[11] += x[11];
  y[12] += x[12];
}
template <typename T>
inline void i_add14(const T x[14], T y[14]) {
  y[0] += x[0];
  y[1] += x[1];
  y[2] += x[2];
  y[3] += x[3];
  y[4] += x[4];
  y[5] += x[5];
  y[6] += x[6];
  y[7] += x[7];
  y[8] += x[8];
  y[9] += x[9];
  y[10] += x[10];
  y[11] += x[11];
  y[12] += x[12];
  y[13] += x[13];
}
template <typename T>
inline void i_add15(const T x[15], T y[15]) {
  y[0] += x[0];
  y[1] += x[1];
  y[2] += x[2];
  y[3] += x[3];
  y[4] += x[4];
  y[5] += x[5];
  y[6] += x[6];
  y[7] += x[7];
  y[8] += x[8];
  y[9] += x[9];
  y[10] += x[10];
  y[11] += x[11];
  y[12] += x[12];
  y[13] += x[13];
  y[14] += x[14];
}
template <typename T>
inline void i_add16(const T x[16], T y[16]) {
  y[0] += x[0];
  y[1] += x[1];
  y[2] += x[2];
  y[3] += x[3];
  y[4] += x[4];
  y[5] += x[5];
  y[6] += x[6];
  y[7] += x[7];
  y[8] += x[8];
  y[9] += x[9];
  y[10] += x[10];
  y[11] += x[11];
  y[12] += x[12];
  y[13] += x[13];
  y[14] += x[14];
  y[15] += x[15];
}
template <typename T>
inline void i_add20(const T x[20], T y[20]) {
  y[0] += x[0];
  y[1] += x[1];
  y[2] += x[2];
  y[3] += x[3];
  y[4] += x[4];
  y[5] += x[5];
  y[6] += x[6];
  y[7] += x[7];
  y[8] += x[8];
  y[9] += x[9];
  y[10] += x[10];
  y[11] += x[11];
  y[12] += x[12];
  y[13] += x[13];
  y[14] += x[14];
  y[15] += x[15];
  y[16] += x[16];
  y[17] += x[17];
  y[18] += x[18];
  y[19] += x[19];
}

/*Compute y=y+x*k where x and y are n-dimensional vectors, k is constant*/
template <typename T>
inline void i_add_scaled(const T *x, T *y, int n, T k) {
  for (int i = 0; i < n; i++) {
    y[i] += (x[i] * k);
  }
}
template <typename T>
inline void i_add_scaled1(const T x[1], T y[1], T k) {
  y[0] += x[0] * k;
}
template <typename T>
inline void i_add_scaled2(const T x[2], T y[2], T k) {
  y[0] += x[0] * k;
  y[1] += x[1] * k;
}
template <typename T>
inline void i_add_scaled3(const T x[3], T y[3], T k) {
  y[0] += x[0] * k;
  y[1] += x[1] * k;
  y[2] += x[2] * k;
}
template <typename T>
inline void i_add_scaled4(const T x[4], T y[4], T k) {
  y[0] += x[0] * k;
  y[1] += x[1] * k;
  y[2] += x[2] * k;
  y[3] += x[3] * k;
}
template <typename T>
inline void i_add_scaled5(const T x[5], T y[5], T k) {
  y[0] += x[0] * k;
  y[1] += x[1] * k;
  y[2] += x[2] * k;
  y[3] += x[3] * k;
  y[4] += x[4] * k;
}
template <typename T>
inline void i_add_scaled6(const T x[6], T y[6], T k) {
  y[0] += x[0] * k;
  y[1] += x[1] * k;
  y[2] += x[2] * k;
  y[3] += x[3] * k;
  y[4] += x[4] * k;
  y[5] += x[5] * k;
}
template <typename T>
inline void i_add_scaled7(const T x[7], T y[7], T k) {
  y[0] += x[0] * k;
  y[1] += x[1] * k;
  y[2] += x[2] * k;
  y[3] += x[3] * k;
  y[4] += x[4] * k;
  y[5] += x[5] * k;
  y[6] += x[6] * k;
}
template <typename T>
inline void i_add_scaled8(const T x[8], T y[8], T k) {
  y[0] += x[0] * k;
  y[1] += x[1] * k;
  y[2] += x[2] * k;
  y[3] += x[3] * k;
  y[4] += x[4] * k;
  y[5] += x[5] * k;
  y[6] += x[6] * k;
  y[7] += x[7] * k;
}
template <typename T>
inline void i_add_scaled9(const T x[9], T y[9], T k) {
  y[0] += x[0] * k;
  y[1] += x[1] * k;
  y[2] += x[2] * k;
  y[3] += x[3] * k;
  y[4] += x[4] * k;
  y[5] += x[5] * k;
  y[6] += x[6] * k;
  y[7] += x[7] * k;
  y[8] += x[8] * k;
}

/*Compute z=x+y*k where x, y and z are n-dimensional vectors, k is constant*/
template <typename T>
inline void i_add_scaled(const T *x, const T *y, T *z, int n, T k) {
  for (int i = 0; i < n; i++) {
    z[i] = x[i] + y[i] * k;
  }
}
template <typename T>
inline void i_add_scaled1(const T x[1], const T y[1], T z[1], T k) {
  z[0] = x[0] + y[0] * k;
}
template <typename T>
inline void i_add_scaled2(const T x[2], const T y[2], T z[2], T k) {
  z[0] = x[0] + y[0] * k;
  z[1] = x[1] + y[1] * k;
}
template <typename T>
inline void i_add_scaled3(const T x[3], const T y[3], T z[3], T k) {
  z[0] = x[0] + y[0] * k;
  z[1] = x[1] + y[1] * k;
  z[2] = x[2] + y[2] * k;
}
template <typename T>
inline void i_add_scaled4(const T x[4], const T y[4], T z[4], T k) {
  z[0] = x[0] + y[0] * k;
  z[1] = x[1] + y[1] * k;
  z[2] = x[2] + y[2] * k;
  z[3] = x[3] + y[3] * k;
}
template <typename T>
inline void i_add_scaled5(const T x[5], const T y[5], T z[5], T k) {
  z[0] = x[0] + y[0] * k;
  z[1] = x[1] + y[1] * k;
  z[2] = x[2] + y[2] * k;
  z[3] = x[3] + y[3] * k;
  z[4] = x[4] + y[4] * k;
}
template <typename T>
inline void i_add_scaled6(const T x[6], const T y[6], T z[6], T k) {
  z[0] = x[0] + y[0] * k;
  z[1] = x[1] + y[1] * k;
  z[2] = x[2] + y[2] * k;
  z[3] = x[3] + y[3] * k;
  z[4] = x[4] + y[4] * k;
  z[5] = x[5] + y[5] * k;
}
template <typename T>
inline void i_add_scaled7(const T x[7], const T y[7], T z[7], T k) {
  z[0] = x[0] + y[0] * k;
  z[1] = x[1] + y[1] * k;
  z[2] = x[2] + y[2] * k;
  z[3] = x[3] + y[3] * k;
  z[4] = x[4] + y[4] * k;
  z[5] = x[5] + y[5] * k;
  z[6] = x[6] + y[6] * k;
}
template <typename T>
inline void i_add_scaled8(const T x[7], const T y[7], T z[7], T k) {
  z[0] = x[0] + y[0] * k;
  z[1] = x[1] + y[1] * k;
  z[2] = x[2] + y[2] * k;
  z[3] = x[3] + y[3] * k;
  z[4] = x[4] + y[4] * k;
  z[5] = x[5] + y[5] * k;
  z[6] = x[6] + y[6] * k;
  z[7] = x[7] + y[7] * k;
}
template <typename T>
inline void i_add_scaled9(const T x[9], const T y[9], T z[9], T k) {
  z[0] = x[0] + y[0] * k;
  z[1] = x[1] + y[1] * k;
  z[2] = x[2] + y[2] * k;
  z[3] = x[3] + y[3] * k;
  z[4] = x[4] + y[4] * k;
  z[5] = x[5] + y[5] * k;
  z[6] = x[6] + y[6] * k;
  z[7] = x[7] + y[7] * k;
  z[8] = x[8] + y[8] * k;
}

/*Compute x=x-c where x is n-dimensional vectors and c is a constant*/
template <typename T>
inline void i_sub(T *x, int n, T k) {
  for (int i = 0; i < n; i++) {
    x[i] -= k;
  }
}
/*Compute z=x-y where x, y, and z are n-dimensional vectors*/
template <typename T>
inline void i_sub(const T *x, const T *y, int n, T *z) {
  for (int i = 0; i < n; i++) {
    z[i] = x[i] - y[i];
  }
}
template <typename T>
inline void i_sub1(const T x[1], const T y[1], T z[1]) {
  z[0] = x[0] - y[0];
}
template <typename T>
inline void i_sub2(const T x[2], const T y[2], T z[2]) {
  z[0] = x[0] - y[0];
  z[1] = x[1] - y[1];
}
template <typename T>
inline void i_sub3(const T x[3], const T y[3], T z[3]) {
  z[0] = x[0] - y[0];
  z[1] = x[1] - y[1];
  z[2] = x[2] - y[2];
}
template <typename T>
inline void i_sub4(const T x[4], const T y[4], T z[4]) {
  z[0] = x[0] - y[0];
  z[1] = x[1] - y[1];
  z[2] = x[2] - y[2];
  z[3] = x[3] - y[3];
}
template <typename T>
inline void i_sub5(const T x[5], const T y[5], T z[5]) {
  z[0] = x[0] - y[0];
  z[1] = x[1] - y[1];
  z[2] = x[2] - y[2];
  z[3] = x[3] - y[3];
  z[4] = x[4] - y[4];
}
template <typename T>
inline void i_sub6(const T x[6], const T y[6], T z[6]) {
  z[0] = x[0] - y[0];
  z[1] = x[1] - y[1];
  z[2] = x[2] - y[2];
  z[3] = x[3] - y[3];
  z[4] = x[4] - y[4];
  z[5] = x[5] - y[5];
}
template <typename T>
inline void i_sub7(const T x[7], const T y[7], T z[7]) {
  z[0] = x[0] - y[0];
  z[1] = x[1] - y[1];
  z[2] = x[2] - y[2];
  z[3] = x[3] - y[3];
  z[4] = x[4] - y[4];
  z[5] = x[5] - y[5];
  z[6] = x[6] - y[6];
}
template <typename T>
inline void i_sub8(const T x[8], const T y[8], T z[8]) {
  z[0] = x[0] - y[0];
  z[1] = x[1] - y[1];
  z[2] = x[2] - y[2];
  z[3] = x[3] - y[3];
  z[4] = x[4] - y[4];
  z[5] = x[5] - y[5];
  z[6] = x[6] - y[6];
  z[7] = x[7] - y[7];
}
template <typename T>
inline void i_sub9(const T x[9], const T y[9], T z[9]) {
  z[0] = x[0] - y[0];
  z[1] = x[1] - y[1];
  z[2] = x[2] - y[2];
  z[3] = x[3] - y[3];
  z[4] = x[4] - y[4];
  z[5] = x[5] - y[5];
  z[6] = x[6] - y[6];
  z[7] = x[7] - y[7];
  z[8] = x[8] - y[8];
}
template <typename T>
inline void i_sub10(const T x[10], const T y[10], T z[10]) {
  z[0] = x[0] - y[0];
  z[1] = x[1] - y[1];
  z[2] = x[2] - y[2];
  z[3] = x[3] - y[3];
  z[4] = x[4] - y[4];
  z[5] = x[5] - y[5];
  z[6] = x[6] - y[6];
  z[7] = x[7] - y[7];
  z[8] = x[8] - y[8];
  z[9] = x[9] - y[9];
}
template <typename T>
inline void i_sub11(const T x[11], const T y[11], T z[11]) {
  z[0] = x[0] - y[0];
  z[1] = x[1] - y[1];
  z[2] = x[2] - y[2];
  z[3] = x[3] - y[3];
  z[4] = x[4] - y[4];
  z[5] = x[5] - y[5];
  z[6] = x[6] - y[6];
  z[7] = x[7] - y[7];
  z[8] = x[8] - y[8];
  z[9] = x[9] - y[9];
  z[10] = x[10] - y[10];
}
template <typename T>
inline void i_sub12(const T x[12], const T y[12], T z[12]) {
  z[0] = x[0] - y[0];
  z[1] = x[1] - y[1];
  z[2] = x[2] - y[2];
  z[3] = x[3] - y[3];
  z[4] = x[4] - y[4];
  z[5] = x[5] - y[5];
  z[6] = x[6] - y[6];
  z[7] = x[7] - y[7];
  z[8] = x[8] - y[8];
  z[9] = x[9] - y[9];
  z[10] = x[10] - y[10];
  z[11] = x[11] - y[11];
}
template <typename T>
inline void i_sub13(const T x[13], const T y[13], T z[13]) {
  z[0] = x[0] - y[0];
  z[1] = x[1] - y[1];
  z[2] = x[2] - y[2];
  z[3] = x[3] - y[3];
  z[4] = x[4] - y[4];
  z[5] = x[5] - y[5];
  z[6] = x[6] - y[6];
  z[7] = x[7] - y[7];
  z[8] = x[8] - y[8];
  z[9] = x[9] - y[9];
  z[10] = x[10] - y[10];
  z[11] = x[11] - y[11];
  z[12] = x[12] - y[12];
}
template <typename T>
inline void i_sub14(const T x[14], const T y[14], T z[14]) {
  z[0] = x[0] - y[0];
  z[1] = x[1] - y[1];
  z[2] = x[2] - y[2];
  z[3] = x[3] - y[3];
  z[4] = x[4] - y[4];
  z[5] = x[5] - y[5];
  z[6] = x[6] - y[6];
  z[7] = x[7] - y[7];
  z[8] = x[8] - y[8];
  z[9] = x[9] - y[9];
  z[10] = x[10] - y[10];
  z[11] = x[11] - y[11];
  z[12] = x[12] - y[12];
  z[13] = x[13] - y[13];
}
template <typename T>
inline void i_sub15(const T x[15], const T y[15], T z[15]) {
  z[0] = x[0] - y[0];
  z[1] = x[1] - y[1];
  z[2] = x[2] - y[2];
  z[3] = x[3] - y[3];
  z[4] = x[4] - y[4];
  z[5] = x[5] - y[5];
  z[6] = x[6] - y[6];
  z[7] = x[7] - y[7];
  z[8] = x[8] - y[8];
  z[9] = x[9] - y[9];
  z[10] = x[10] - y[10];
  z[11] = x[11] - y[11];
  z[12] = x[12] - y[12];
  z[13] = x[13] - y[13];
  z[14] = x[14] - y[14];
}
template <typename T>
inline void i_sub16(const T x[16], const T y[16], T z[16]) {
  z[0] = x[0] - y[0];
  z[1] = x[1] - y[1];
  z[2] = x[2] - y[2];
  z[3] = x[3] - y[3];
  z[4] = x[4] - y[4];
  z[5] = x[5] - y[5];
  z[6] = x[6] - y[6];
  z[7] = x[7] - y[7];
  z[8] = x[8] - y[8];
  z[9] = x[9] - y[9];
  z[10] = x[10] - y[10];
  z[11] = x[11] - y[11];
  z[12] = x[12] - y[12];
  z[13] = x[13] - y[13];
  z[14] = x[14] - y[14];
  z[15] = x[15] - y[15];
}

/*Compute y=y-x where x and y are n-dimensional vectors*/
template <typename T>
inline void i_sub(const T *x, T *y, int n) {
  for (int i = 0; i < n; i++) {
    y[i] -= x[i];
  }
}
template <typename T>
inline void i_sub1(const T x[1], T y[1]) {
  y[0] -= x[0];
}
template <typename T>
inline void i_sub2(const T x[2], T y[2]) {
  y[0] -= x[0];
  y[1] -= x[1];
}
template <typename T>
inline void i_sub3(const T x[3], T y[3]) {
  y[0] -= x[0];
  y[1] -= x[1];
  y[2] -= x[2];
}
template <typename T>
inline void i_sub4(const T x[4], T y[4]) {
  y[0] -= x[0];
  y[1] -= x[1];
  y[2] -= x[2];
  y[3] -= x[3];
}
template <typename T>
inline void i_sub5(const T x[5], T y[5]) {
  y[0] -= x[0];
  y[1] -= x[1];
  y[2] -= x[2];
  y[3] -= x[3];
  y[4] -= x[4];
}
template <typename T>
inline void i_sub6(const T x[6], T y[6]) {
  y[0] -= x[0];
  y[1] -= x[1];
  y[2] -= x[2];
  y[3] -= x[3];
  y[4] -= x[4];
  y[5] -= x[5];
}
template <typename T>
inline void i_sub7(const T x[7], T y[7]) {
  y[0] -= x[0];
  y[1] -= x[1];
  y[2] -= x[2];
  y[3] -= x[3];
  y[4] -= x[4];
  y[5] -= x[5];
  y[6] -= x[6];
}
template <typename T>
inline void i_sub8(const T x[8], T y[8]) {
  y[0] -= x[0];
  y[1] -= x[1];
  y[2] -= x[2];
  y[3] -= x[3];
  y[4] -= x[4];
  y[5] -= x[5];
  y[6] -= x[6];
  y[7] -= x[7];
}
template <typename T>
inline void i_sub9(const T x[9], T y[9]) {
  y[0] -= x[0];
  y[1] -= x[1];
  y[2] -= x[2];
  y[3] -= x[3];
  y[4] -= x[4];
  y[5] -= x[5];
  y[6] -= x[6];
  y[7] -= x[7];
  y[8] -= x[8];
}
template <typename T>
inline void i_sub10(const T x[10], T y[10]) {
  y[0] -= x[0];
  y[1] -= x[1];
  y[2] -= x[2];
  y[3] -= x[3];
  y[4] -= x[4];
  y[5] -= x[5];
  y[6] -= x[6];
  y[7] -= x[7];
  y[8] -= x[8];
  y[9] -= x[9];
}
template <typename T>
inline void i_sub11(const T x[11], T y[11]) {
  y[0] -= x[0];
  y[1] -= x[1];
  y[2] -= x[2];
  y[3] -= x[3];
  y[4] -= x[4];
  y[5] -= x[5];
  y[6] -= x[6];
  y[7] -= x[7];
  y[8] -= x[8];
  y[9] -= x[9];
  y[10] -= x[10];
}
template <typename T>
inline void i_sub12(const T x[12], T y[12]) {
  y[0] -= x[0];
  y[1] -= x[1];
  y[2] -= x[2];
  y[3] -= x[3];
  y[4] -= x[4];
  y[5] -= x[5];
  y[6] -= x[6];
  y[7] -= x[7];
  y[8] -= x[8];
  y[9] -= x[9];
  y[10] -= x[10];
  y[11] -= x[11];
}
template <typename T>
inline void i_sub13(const T x[13], T y[13]) {
  y[0] -= x[0];
  y[1] -= x[1];
  y[2] -= x[2];
  y[3] -= x[3];
  y[4] -= x[4];
  y[5] -= x[5];
  y[6] -= x[6];
  y[7] -= x[7];
  y[8] -= x[8];
  y[9] -= x[9];
  y[10] -= x[10];
  y[11] -= x[11];
  y[12] -= x[12];
}
template <typename T>
inline void i_sub14(const T x[14], T y[14]) {
  y[0] -= x[0];
  y[1] -= x[1];
  y[2] -= x[2];
  y[3] -= x[3];
  y[4] -= x[4];
  y[5] -= x[5];
  y[6] -= x[6];
  y[7] -= x[7];
  y[8] -= x[8];
  y[9] -= x[9];
  y[10] -= x[10];
  y[11] -= x[11];
  y[12] -= x[12];
  y[13] -= x[13];
}
template <typename T>
inline void i_sub15(const T x[15], T y[15]) {
  y[0] -= x[0];
  y[1] -= x[1];
  y[2] -= x[2];
  y[3] -= x[3];
  y[4] -= x[4];
  y[5] -= x[5];
  y[6] -= x[6];
  y[7] -= x[7];
  y[8] -= x[8];
  y[9] -= x[9];
  y[10] -= x[10];
  y[11] -= x[11];
  y[12] -= x[12];
  y[13] -= x[13];
  y[14] -= x[14];
}
template <typename T>
inline void i_sub16(const T x[16], T y[16]) {
  y[0] -= x[0];
  y[1] -= x[1];
  y[2] -= x[2];
  y[3] -= x[3];
  y[4] -= x[4];
  y[5] -= x[5];
  y[6] -= x[6];
  y[7] -= x[7];
  y[8] -= x[8];
  y[9] -= x[9];
  y[10] -= x[10];
  y[11] -= x[11];
  y[12] -= x[12];
  y[13] -= x[13];
  y[14] -= x[14];
  y[15] -= x[15];
}
/*Compute y=y-x*k where x and y are n-dimensional vectors, k is constant*/
template <typename T>
inline void i_sub_scaled(const T *x, T *y, int n, T k) {
  for (int i = 0; i < n; i++) {
    y[i] -= (x[i] * k);
  }
}
template <typename T>
inline void i_sub_scaled1(const T x[1], T y[1], T k) {
  y[0] -= x[0] * k;
}
template <typename T>
inline void i_sub_scaled2(const T x[2], T y[2], T k) {
  y[0] -= x[0] * k;
  y[1] -= x[1] * k;
}
template <typename T>
inline void i_sub_scaled3(const T x[3], T y[3], T k) {
  y[0] -= x[0] * k;
  y[1] -= x[1] * k;
  y[2] -= x[2] * k;
}
template <typename T>
inline void i_sub_scaled4(const T x[4], T y[4], T k) {
  y[0] -= x[0] * k;
  y[1] -= x[1] * k;
  y[2] -= x[2] * k;
  y[3] -= x[3] * k;
}
template <typename T>
inline void i_sub_scaled5(const T x[5], T y[5], T k) {
  y[0] -= x[0] * k;
  y[1] -= x[1] * k;
  y[2] -= x[2] * k;
  y[3] -= x[3] * k;
  y[4] -= x[4] * k;
}
template <typename T>
inline void i_sub_scaled6(const T x[6], T y[6], T k) {
  y[0] -= x[0] * k;
  y[1] -= x[1] * k;
  y[2] -= x[2] * k;
  y[3] -= x[3] * k;
  y[4] -= x[4] * k;
  y[5] -= x[5] * k;
}
template <typename T>
inline void i_sub_scaled7(const T x[7], T y[7], T k) {
  y[0] -= x[0] * k;
  y[1] -= x[1] * k;
  y[2] -= x[2] * k;
  y[3] -= x[3] * k;
  y[4] -= x[4] * k;
  y[5] -= x[5] * k;
  y[6] -= x[6] * k;
}
template <typename T>
inline void i_sub_scaled8(const T x[8], T y[8], T k) {
  y[0] -= x[0] * k;
  y[1] -= x[1] * k;
  y[2] -= x[2] * k;
  y[3] -= x[3] * k;
  y[4] -= x[4] * k;
  y[5] -= x[5] * k;
  y[6] -= x[6] * k;
  y[7] -= x[7] * k;
}
template <typename T>
inline void i_sub_scaled9(const T x[9], T y[9], T k) {
  y[0] -= x[0] * k;
  y[1] -= x[1] * k;
  y[2] -= x[2] * k;
  y[3] -= x[3] * k;
  y[4] -= x[4] * k;
  y[5] -= x[5] * k;
  y[6] -= x[6] * k;
  y[7] -= x[7] * k;
  y[8] -= x[8] * k;
}

/*Rescale n-dimensional vector x with a scale factor sf (inplace)*/
template <typename T>
inline void i_scale(T *x, int n, T sf) {
  for (int i = 0; i < n; i++) x[i] *= sf;
}
template <typename T>
inline void i_scale1(T x[1], T sf) {
  x[0] *= sf;
}
template <typename T>
inline void i_scale2(T x[2], T sf) {
  x[0] *= sf;
  x[1] *= sf;
}
template <typename T>
inline void i_scale3(T x[3], T sf) {
  x[0] *= sf;
  x[1] *= sf;
  x[2] *= sf;
}
template <typename T>
inline void i_scale4(T x[4], T sf) {
  x[0] *= sf;
  x[1] *= sf;
  x[2] *= sf;
  x[3] *= sf;
}
template <typename T>
inline void i_scale5(T x[5], T sf) {
  x[0] *= sf;
  x[1] *= sf;
  x[2] *= sf;
  x[3] *= sf;
  x[4] *= sf;
}
template <typename T>
inline void i_scale6(T x[6], T sf) {
  x[0] *= sf;
  x[1] *= sf;
  x[2] *= sf;
  x[3] *= sf;
  x[4] *= sf;
  x[5] *= sf;
}
template <typename T>
inline void i_scale7(T x[7], T sf) {
  x[0] *= sf;
  x[1] *= sf;
  x[2] *= sf;
  x[3] *= sf;
  x[4] *= sf;
  x[5] *= sf;
  x[6] *= sf;
}
template <typename T>
inline void i_scale8(T x[8], T sf) {
  x[0] *= sf;
  x[1] *= sf;
  x[2] *= sf;
  x[3] *= sf;
  x[4] *= sf;
  x[5] *= sf;
  x[6] *= sf;
  x[7] *= sf;
}
template <typename T>
inline void i_scale9(T x[9], T sf) {
  x[0] *= sf;
  x[1] *= sf;
  x[2] *= sf;
  x[3] *= sf;
  x[4] *= sf;
  x[5] *= sf;
  x[6] *= sf;
  x[7] *= sf;
  x[8] *= sf;
}
template <typename T>
inline void i_scale10(T x[10], T sf) {
  x[0] *= sf;
  x[1] *= sf;
  x[2] *= sf;
  x[3] *= sf;
  x[4] *= sf;
  x[5] *= sf;
  x[6] *= sf;
  x[7] *= sf;
  x[8] *= sf;
  x[9] *= sf;
}
template <typename T>
inline void i_scale11(T x[11], T sf) {
  x[0] *= sf;
  x[1] *= sf;
  x[2] *= sf;
  x[3] *= sf;
  x[4] *= sf;
  x[5] *= sf;
  x[6] *= sf;
  x[7] *= sf;
  x[8] *= sf;
  x[9] *= sf;
  x[10] *= sf;
}
template <typename T>
inline void i_scale12(T x[12], T sf) {
  x[0] *= sf;
  x[1] *= sf;
  x[2] *= sf;
  x[3] *= sf;
  x[4] *= sf;
  x[5] *= sf;
  x[6] *= sf;
  x[7] *= sf;
  x[8] *= sf;
  x[9] *= sf;
  x[10] *= sf;
  x[11] *= sf;
}
template <typename T>
inline void i_scale13(T x[13], T sf) {
  x[0] *= sf;
  x[1] *= sf;
  x[2] *= sf;
  x[3] *= sf;
  x[4] *= sf;
  x[5] *= sf;
  x[6] *= sf;
  x[7] *= sf;
  x[8] *= sf;
  x[9] *= sf;
  x[10] *= sf;
  x[11] *= sf;
  x[12] *= sf;
}
template <typename T>
inline void i_scale14(T x[14], T sf) {
  x[0] *= sf;
  x[1] *= sf;
  x[2] *= sf;
  x[3] *= sf;
  x[4] *= sf;
  x[5] *= sf;
  x[6] *= sf;
  x[7] *= sf;
  x[8] *= sf;
  x[9] *= sf;
  x[10] *= sf;
  x[11] *= sf;
  x[12] *= sf;
  x[13] *= sf;
}
template <typename T>
inline void i_scale15(T x[15], T sf) {
  x[0] *= sf;
  x[1] *= sf;
  x[2] *= sf;
  x[3] *= sf;
  x[4] *= sf;
  x[5] *= sf;
  x[6] *= sf;
  x[7] *= sf;
  x[8] *= sf;
  x[9] *= sf;
  x[10] *= sf;
  x[11] *= sf;
  x[12] *= sf;
  x[13] *= sf;
  x[14] *= sf;
}
template <typename T>
inline void i_scale16(T x[16], T sf) {
  x[0] *= sf;
  x[1] *= sf;
  x[2] *= sf;
  x[3] *= sf;
  x[4] *= sf;
  x[5] *= sf;
  x[6] *= sf;
  x[7] *= sf;
  x[8] *= sf;
  x[9] *= sf;
  x[10] *= sf;
  x[11] *= sf;
  x[12] *= sf;
  x[13] *= sf;
  x[14] *= sf;
  x[15] *= sf;
}
/*Rescale n-dimensional vector x with a scale factor sf, save the result to
 * n-dimensional vector y*/
template <typename T>
inline void i_scale(const T *x, T *y, int n, T sf) {
  for (int i = 0; i < n; i++) {
    y[i] = x[i] * sf;
  }
}
template <typename T>
inline void i_scale1(const T x[1], T y[1], T sf) {
  y[0] = x[0] * sf;
}
template <typename T>
inline void i_scale2(const T x[2], T y[2], T sf) {
  y[0] = x[0] * sf;
  y[1] = x[1] * sf;
}
template <typename T>
inline void i_scale3(const T x[3], T y[3], T sf) {
  y[0] = x[0] * sf;
  y[1] = x[1] * sf;
  y[2] = x[2] * sf;
}
template <typename T>
inline void i_scale4(const T x[4], T y[4], T sf) {
  y[0] = x[0] * sf;
  y[1] = x[1] * sf;
  y[2] = x[2] * sf;
  y[3] = x[3] * sf;
}
template <typename T>
inline void i_scale5(const T x[5], T y[5], T sf) {
  y[0] = x[0] * sf;
  y[1] = x[1] * sf;
  y[2] = x[2] * sf;
  y[3] = x[3] * sf;
  y[4] = x[4] * sf;
}
template <typename T>
inline void i_scale6(const T x[6], T y[6], T sf) {
  y[0] = x[0] * sf;
  y[1] = x[1] * sf;
  y[2] = x[2] * sf;
  y[3] = x[3] * sf;
  y[4] = x[4] * sf;
  y[5] = x[5] * sf;
}
template <typename T>
inline void i_scale7(const T x[7], T y[7], T sf) {
  y[0] = x[0] * sf;
  y[1] = x[1] * sf;
  y[2] = x[2] * sf;
  y[3] = x[3] * sf;
  y[4] = x[4] * sf;
  y[5] = x[5] * sf;
  y[6] = x[6] * sf;
}
template <typename T>
inline void i_scale8(const T x[8], T y[8], T sf) {
  y[0] = x[0] * sf;
  y[1] = x[1] * sf;
  y[2] = x[2] * sf;
  y[3] = x[3] * sf;
  y[4] = x[4] * sf;
  y[5] = x[5] * sf;
  y[6] = x[6] * sf;
  y[7] = x[7] * sf;
}
template <typename T>
inline void i_scale9(const T x[9], T y[9], T sf) {
  y[0] = x[0] * sf;
  y[1] = x[1] * sf;
  y[2] = x[2] * sf;
  y[3] = x[3] * sf;
  y[4] = x[4] * sf;
  y[5] = x[5] * sf;
  y[6] = x[6] * sf;
  y[7] = x[7] * sf;
  y[8] = x[8] * sf;
}
template <typename T>
inline void i_scale10(const T x[10], T y[10], T sf) {
  y[0] = x[0] * sf;
  y[1] = x[1] * sf;
  y[2] = x[2] * sf;
  y[3] = x[3] * sf;
  y[4] = x[4] * sf;
  y[5] = x[5] * sf;
  y[6] = x[6] * sf;
  y[7] = x[7] * sf;
  y[8] = x[8] * sf;
  y[9] = x[9] * sf;
}
template <typename T>
inline void i_scale11(const T x[11], T y[11], T sf) {
  y[0] = x[0] * sf;
  y[1] = x[1] * sf;
  y[2] = x[2] * sf;
  y[3] = x[3] * sf;
  y[4] = x[4] * sf;
  y[5] = x[5] * sf;
  y[6] = x[6] * sf;
  y[7] = x[7] * sf;
  y[8] = x[8] * sf;
  y[9] = x[9] * sf;
  y[10] = x[10] * sf;
}
template <typename T>
inline void i_scale12(const T x[12], T y[12], T sf) {
  y[0] = x[0] * sf;
  y[1] = x[1] * sf;
  y[2] = x[2] * sf;
  y[3] = x[3] * sf;
  y[4] = x[4] * sf;
  y[5] = x[5] * sf;
  y[6] = x[6] * sf;
  y[7] = x[7] * sf;
  y[8] = x[8] * sf;
  y[9] = x[9] * sf;
  y[10] = x[10] * sf;
  y[11] = x[11] * sf;
}
template <typename T>
inline void i_scale13(const T x[13], T y[13], T sf) {
  y[0] = x[0] * sf;
  y[1] = x[1] * sf;
  y[2] = x[2] * sf;
  y[3] = x[3] * sf;
  y[4] = x[4] * sf;
  y[5] = x[5] * sf;
  y[6] = x[6] * sf;
  y[7] = x[7] * sf;
  y[8] = x[8] * sf;
  y[9] = x[9] * sf;
  y[10] = x[10] * sf;
  y[11] = x[11] * sf;
  y[12] = x[12] * sf;
}
template <typename T>
inline void i_scale14(const T x[14], T y[14], T sf) {
  y[0] = x[0] * sf;
  y[1] = x[1] * sf;
  y[2] = x[2] * sf;
  y[3] = x[3] * sf;
  y[4] = x[4] * sf;
  y[5] = x[5] * sf;
  y[6] = x[6] * sf;
  y[7] = x[7] * sf;
  y[8] = x[8] * sf;
  y[9] = x[9] * sf;
  y[10] = x[10] * sf;
  y[11] = x[11] * sf;
  y[12] = x[12] * sf;
  y[13] = x[13] * sf;
}
template <typename T>
inline void i_scale15(const T x[15], T y[15], T sf) {
  y[0] = x[0] * sf;
  y[1] = x[1] * sf;
  y[2] = x[2] * sf;
  y[3] = x[3] * sf;
  y[4] = x[4] * sf;
  y[5] = x[5] * sf;
  y[6] = x[6] * sf;
  y[7] = x[7] * sf;
  y[8] = x[8] * sf;
  y[9] = x[9] * sf;
  y[10] = x[10] * sf;
  y[11] = x[11] * sf;
  y[12] = x[12] * sf;
  y[13] = x[13] * sf;
  y[14] = x[14] * sf;
}
template <typename T>
inline void i_scale16(const T x[16], T y[16], T sf) {
  y[0] = x[0] * sf;
  y[1] = x[1] * sf;
  y[2] = x[2] * sf;
  y[3] = x[3] * sf;
  y[4] = x[4] * sf;
  y[5] = x[5] * sf;
  y[6] = x[6] * sf;
  y[7] = x[7] * sf;
  y[8] = x[8] * sf;
  y[9] = x[9] * sf;
  y[10] = x[10] * sf;
  y[11] = x[11] * sf;
  y[12] = x[12] * sf;
  y[13] = x[13] * sf;
  y[14] = x[14] * sf;
  y[15] = x[15] * sf;
}

/*Compute dot product of x and y*/
inline int i_dot_u(const unsigned char *x, const unsigned char *y, int n) {
  int acc = 0;
  for (int i = 0; i < n; ++i) {
    acc += (int)x[i] * (int)y[i];
  }
  return acc;
}
template <typename T>
inline T i_dot(const T *x, const T *y, int n) {
  T acc = (T)0.0;
  for (int i = 0; i < n; ++i) {
    acc += x[i] * y[i];
  }
  return acc;
}
template <typename T>
inline T i_dot1(const T x[1], const T y[1]) {
  return (x[0] * y[0]);
}
template <typename T>
inline T i_dot2(const T x[2], const T y[2]) {
  return (x[0] * y[0] + x[1] * y[1]);
}
template <typename T>
inline T i_dot3(const T x[3], const T y[3]) {
  return (x[0] * y[0] + x[1] * y[1] + x[2] * y[2]);
}
template <typename T>
inline T i_dot4(const T x[4], const T y[4]) {
  return (x[0] * y[0] + x[1] * y[1] + x[2] * y[2] + x[3] * y[3]);
}
template <typename T>
inline T i_dot5(const T x[5], const T y[5]) {
  return (x[0] * y[0] + x[1] * y[1] + x[2] * y[2] + x[3] * y[3] + x[4] * y[4]);
}
template <typename T>
inline T i_dot6(const T x[6], const T y[6]) {
  return (x[0] * y[0] + x[1] * y[1] + x[2] * y[2] + x[3] * y[3] + x[4] * y[4] +
          x[5] * y[5]);
}
template <typename T>
inline T i_dot7(const T x[7], const T y[7]) {
  return (x[0] * y[0] + x[1] * y[1] + x[2] * y[2] + x[3] * y[3] + x[4] * y[4] +
          x[5] * y[5] + x[6] * y[6]);
}
template <typename T>
inline T i_dot8(const T x[8], const T y[8]) {
  return (x[0] * y[0] + x[1] * y[1] + x[2] * y[2] + x[3] * y[3] + x[4] * y[4] +
          x[5] * y[5] + x[6] * y[6] + x[7] * y[7]);
}
template <typename T>
inline T i_dot9(const T x[9], const T y[9]) {
  return (x[0] * y[0] + x[1] * y[1] + x[2] * y[2] + x[3] * y[3] + x[4] * y[4] +
          x[5] * y[5] + x[6] * y[6] + x[7] * y[7] + x[8] * y[8]);
}
template <typename T>
inline T i_dot10(const T x[10], const T y[10]) {
  return (x[0] * y[0] + x[1] * y[1] + x[2] * y[2] + x[3] * y[3] + x[4] * y[4] +
          x[5] * y[5] + x[6] * y[6] + x[7] * y[7] + x[8] * y[8] + x[9] * y[9]);
}
template <typename T>
inline T i_dot11(const T x[11], const T y[11]) {
  return (x[0] * y[0] + x[1] * y[1] + x[2] * y[2] + x[3] * y[3] + x[4] * y[4] +
          x[5] * y[5] + x[6] * y[6] + x[7] * y[7] + x[8] * y[8] + x[9] * y[9] +
          x[10] * y[10]);
}
template <typename T>
inline T i_dot12(const T x[12], const T y[12]) {
  return (x[0] * y[0] + x[1] * y[1] + x[2] * y[2] + x[3] * y[3] + x[4] * y[4] +
          x[5] * y[5] + x[6] * y[6] + x[7] * y[7] + x[8] * y[8] + x[9] * y[9] +
          x[10] * y[10] + x[11] * y[11]);
}
template <typename T>
inline T i_dot13(const T x[13], const T y[13]) {
  return (x[0] * y[0] + x[1] * y[1] + x[2] * y[2] + x[3] * y[3] + x[4] * y[4] +
          x[5] * y[5] + x[6] * y[6] + x[7] * y[7] + x[8] * y[8] + x[9] * y[9] +
          x[10] * y[10] + x[11] * y[11] + x[12] * y[12]);
}
template <typename T>
inline T i_dot14(const T x[14], const T y[14]) {
  return (x[0] * y[0] + x[1] * y[1] + x[2] * y[2] + x[3] * y[3] + x[4] * y[4] +
          x[5] * y[5] + x[6] * y[6] + x[7] * y[7] + x[8] * y[8] + x[9] * y[9] +
          x[10] * y[10] + x[11] * y[11] + x[12] * y[12] + x[13] * y[13]);
}
template <typename T>
inline T i_dot15(const T x[15], const T y[15]) {
  return (x[0] * y[0] + x[1] * y[1] + x[2] * y[2] + x[3] * y[3] + x[4] * y[4] +
          x[5] * y[5] + x[6] * y[6] + x[7] * y[7] + x[8] * y[8] + x[9] * y[9] +
          x[10] * y[10] + x[11] * y[11] + x[12] * y[12] + x[13] * y[13] +
          x[14] * y[14]);
}
template <typename T>
inline T i_dot16(const T x[16], const T y[16]) {
  return (x[0] * y[0] + x[1] * y[1] + x[2] * y[2] + x[3] * y[3] + x[4] * y[4] +
          x[5] * y[5] + x[6] * y[6] + x[7] * y[7] + x[8] * y[8] + x[9] * y[9] +
          x[10] * y[10] + x[11] * y[11] + x[12] * y[12] + x[13] * y[13] +
          x[14] * y[14] + x[15] * y[15]);
}

/*Compute sum of n-dimensional vector x*/
inline int i_sum_u(const unsigned char *x, int n) {
  int acc = 0;
  for (int i = 0; i < n; ++i) {
    acc += (int)x[i];
  }
  return acc;
}
template <typename T>
inline T i_sum(const T *x, int n) {
  T acc = (T)(0.0);
  for (int i = 0; i < n; ++i) {
    acc += x[i];
  }
  return acc;
}
template <typename T>
inline T i_sum1(const T x[1]) {
  return (x[0]);
}
template <typename T>
inline T i_sum2(const T x[2]) {
  return (x[0] + x[1]);
}
template <typename T>
inline T i_sum3(const T x[3]) {
  return (x[0] + x[1] + x[2]);
}
template <typename T>
inline T i_sum4(const T x[4]) {
  return (x[0] + x[1] + x[2] + x[3]);
}
template <typename T>
inline T i_sum5(const T x[5]) {
  return (x[0] + x[1] + x[2] + x[3] + x[4]);
}
template <typename T>
inline T i_sum6(const T x[6]) {
  return (x[0] + x[1] + x[2] + x[3] + x[4] + x[5]);
}
template <typename T>
inline T i_sum7(const T x[7]) {
  return (x[0] + x[1] + x[2] + x[3] + x[4] + x[5] + x[6]);
}
template <typename T>
inline T i_sum8(const T x[8]) {
  return (x[0] + x[1] + x[2] + x[3] + x[4] + x[5] + x[6] + x[7]);
}
template <typename T>
inline T i_sum9(const T x[9]) {
  return (x[0] + x[1] + x[2] + x[3] + x[4] + x[5] + x[6] + x[7] + x[8]);
}
template <typename T>
inline T i_sum10(const T x[10]) {
  return (x[0] + x[1] + x[2] + x[3] + x[4] + x[5] + x[6] + x[7] + x[8] + x[9]);
}
template <typename T>
inline T i_sum11(const T x[11]) {
  return (x[0] + x[1] + x[2] + x[3] + x[4] + x[5] + x[6] + x[7] + x[8] + x[9] +
          x[10]);
}
template <typename T>
inline T i_sum12(const T x[12]) {
  return (x[0] + x[1] + x[2] + x[3] + x[4] + x[5] + x[6] + x[7] + x[8] + x[9] +
          x[10] + x[11]);
}
template <typename T>
inline T i_sum13(const T x[13]) {
  return (x[0] + x[1] + x[2] + x[3] + x[4] + x[5] + x[6] + x[7] + x[8] + x[9] +
          x[10] + x[11] + x[12]);
}
template <typename T>
inline T i_sum14(const T x[14]) {
  return (x[0] + x[1] + x[2] + x[3] + x[4] + x[5] + x[6] + x[7] + x[8] + x[9] +
          x[10] + x[11] + x[12] + x[13]);
}
template <typename T>
inline T i_sum15(const T x[15]) {
  return (x[0] + x[1] + x[2] + x[3] + x[4] + x[5] + x[6] + x[7] + x[8] + x[9] +
          x[10] + x[11] + x[12] + x[13] + x[14]);
}
template <typename T>
inline T i_sum16(const T x[16]) {
  return (x[0] + x[1] + x[2] + x[3] + x[4] + x[5] + x[6] + x[7] + x[8] + x[9] +
          x[10] + x[11] + x[12] + x[13] + x[14] + x[15]);
}

template <typename T>
inline T i_abs_sum(const T *x, int n) {
  T acc = (T)(0.0);
  for (int i = 0; i < n; ++i) {
    acc += i_abs(x[i]);
  }
  return acc;
}

/*Compute mean of n-dimensional vector x*/
inline int i_mean_u(const unsigned char *x, int n) { return i_sum_u(x, n) / n; }
template <typename T>
inline T i_mean(const T *x, int n) {
  return i_sum(x, n) / n;
}
template <typename T>
inline T i_mean2(const T x[2]) {
  return i_sum2(x) / 2;
}
template <typename T>
inline T i_mean3(const T x[3]) {
  return i_sum3(x) / 3;
}
template <typename T>
inline T i_mean4(const T x[4]) {
  return i_sum4(x) / 4;
}
template <typename T>
inline T i_mean5(const T x[5]) {
  return i_sum5(x) / 5;
}
template <typename T>
inline T i_mean6(const T x[6]) {
  return i_sum6(x) / 6;
}
template <typename T>
inline T i_mean7(const T x[7]) {
  return i_sum7(x) / 7;
}
template <typename T>
inline T i_mean8(const T x[8]) {
  return i_sum8(x) / 8;
}
template <typename T>
inline T i_mean9(const T x[9]) {
  return i_sum9(x) / 9;
}
template <typename T>
inline T i_mean10(const T x[10]) {
  return i_sum10(x) / 10;
}
template <typename T>
inline T i_mean11(const T x[11]) {
  return i_sum11(x) / 11;
}
template <typename T>
inline T i_mean12(const T x[12]) {
  return i_sum12(x) / 12;
}
template <typename T>
inline T i_mean13(const T x[13]) {
  return i_sum13(x) / 13;
}
template <typename T>
inline T i_mean14(const T x[14]) {
  return i_sum14(x) / 14;
}
template <typename T>
inline T i_mean15(const T x[15]) {
  return i_sum15(x) / 15;
}
template <typename T>
inline T i_mean16(const T x[16]) {
  return i_sum16(x) / 16;
}

/*Compute the sample standard deviation of sample data x*/
template <typename T>
inline T i_sdv(const T *x, T mean, int n) {
  if (n < 2) return (T)0.0;
  T sdv = (T)0.0;
  for (int i = 0; i < n; ++i) {
    sdv += i_sqr(x[i] - mean);
  }
  return i_sqrt(i_div(sdv, n - 1));
}

/*Compute square sum of n-dimensional vector x*/
inline int i_squaresum_u(const unsigned char *x, int n) {
  int acc = 0;
  for (int i = 0; i < n; i++) {
    acc += i_sqr(x[i]);
  }
  return (acc);
}
template <typename T>
inline T i_squaresum(const T *x, int n) {
  T acc = (T)(0.0);
  for (int i = 0; i < n; ++i) {
    acc += i_sqr(x[i]);
  }
  return acc;
}
template <typename T>
inline T i_squaresum1(const T x[1]) {
  return (i_sqr(x[0]));
}
template <typename T>
inline T i_squaresum2(const T x[2]) {
  return (i_sqr(x[0]) + i_sqr(x[1]));
}
template <typename T>
inline T i_squaresum3(const T x[3]) {
  return (i_sqr(x[0]) + i_sqr(x[1]) + i_sqr(x[2]));
}
template <typename T>
inline T i_squaresum4(const T x[4]) {
  return (i_sqr(x[0]) + i_sqr(x[1]) + i_sqr(x[2]) + i_sqr(x[3]));
}
template <typename T>
inline T i_squaresum5(const T x[5]) {
  return (i_sqr(x[0]) + i_sqr(x[1]) + i_sqr(x[2]) + i_sqr(x[3]) + i_sqr(x[4]));
}
template <typename T>
inline T i_squaresum6(const T x[6]) {
  return (i_sqr(x[0]) + i_sqr(x[1]) + i_sqr(x[2]) + i_sqr(x[3]) + i_sqr(x[4]) +
          i_sqr(x[5]));
}
template <typename T>
inline T i_squaresum7(const T x[7]) {
  return (i_sqr(x[0]) + i_sqr(x[1]) + i_sqr(x[2]) + i_sqr(x[3]) + i_sqr(x[4]) +
          i_sqr(x[5]) + i_sqr(x[6]));
}
template <typename T>
inline T i_squaresum8(const T x[8]) {
  return (i_sqr(x[0]) + i_sqr(x[1]) + i_sqr(x[2]) + i_sqr(x[3]) + i_sqr(x[4]) +
          i_sqr(x[5]) + i_sqr(x[6]) + i_sqr(x[7]));
}
template <typename T>
inline T i_squaresum9(const T x[9]) {
  return (i_sqr(x[0]) + i_sqr(x[1]) + i_sqr(x[2]) + i_sqr(x[3]) + i_sqr(x[4]) +
          i_sqr(x[5]) + i_sqr(x[6]) + i_sqr(x[7]) + i_sqr(x[8]));
}
template <typename T>
inline T i_squaresum10(const T x[10]) {
  return (i_sqr(x[0]) + i_sqr(x[1]) + i_sqr(x[2]) + i_sqr(x[3]) + i_sqr(x[4]) +
          i_sqr(x[5]) + i_sqr(x[6]) + i_sqr(x[7]) + i_sqr(x[8]) + i_sqr(x[9]));
}
template <typename T>
inline T i_squaresum11(const T x[11]) {
  return (i_sqr(x[0]) + i_sqr(x[1]) + i_sqr(x[2]) + i_sqr(x[3]) + i_sqr(x[4]) +
          i_sqr(x[5]) + i_sqr(x[6]) + i_sqr(x[7]) + i_sqr(x[8]) + i_sqr(x[9]) +
          i_sqr(x[10]));
}
template <typename T>
inline T i_squaresum12(const T x[12]) {
  return (i_sqr(x[0]) + i_sqr(x[1]) + i_sqr(x[2]) + i_sqr(x[3]) + i_sqr(x[4]) +
          i_sqr(x[5]) + i_sqr(x[6]) + i_sqr(x[7]) + i_sqr(x[8]) + i_sqr(x[9]) +
          i_sqr(x[10]) + i_sqr(x[11]));
}
template <typename T>
inline T i_squaresum13(const T x[13]) {
  return (i_sqr(x[0]) + i_sqr(x[1]) + i_sqr(x[2]) + i_sqr(x[3]) + i_sqr(x[4]) +
          i_sqr(x[5]) + i_sqr(x[6]) + i_sqr(x[7]) + i_sqr(x[8]) + i_sqr(x[9]) +
          i_sqr(x[10]) + i_sqr(x[11]) + i_sqr(x[12]));
}
template <typename T>
inline T i_squaresum14(const T x[14]) {
  return (i_sqr(x[0]) + i_sqr(x[1]) + i_sqr(x[2]) + i_sqr(x[3]) + i_sqr(x[4]) +
          i_sqr(x[5]) + i_sqr(x[6]) + i_sqr(x[7]) + i_sqr(x[8]) + i_sqr(x[9]) +
          i_sqr(x[10]) + i_sqr(x[11]) + i_sqr(x[12]) + i_sqr(x[13]));
}
template <typename T>
inline T i_squaresum15(const T x[15]) {
  return (i_sqr(x[0]) + i_sqr(x[1]) + i_sqr(x[2]) + i_sqr(x[3]) + i_sqr(x[4]) +
          i_sqr(x[5]) + i_sqr(x[6]) + i_sqr(x[7]) + i_sqr(x[8]) + i_sqr(x[9]) +
          i_sqr(x[10]) + i_sqr(x[11]) + i_sqr(x[12]) + i_sqr(x[13]) +
          i_sqr(x[14]));
}
template <typename T>
inline T i_squaresum16(const T x[16]) {
  return (i_sqr(x[0]) + i_sqr(x[1]) + i_sqr(x[2]) + i_sqr(x[3]) + i_sqr(x[4]) +
          i_sqr(x[5]) + i_sqr(x[6]) + i_sqr(x[7]) + i_sqr(x[8]) + i_sqr(x[9]) +
          i_sqr(x[10]) + i_sqr(x[11]) + i_sqr(x[12]) + i_sqr(x[13]) +
          i_sqr(x[14]) + i_sqr(x[15]));
}

/*Compute square sum of the diff (x-y) between two n-dimensional vector x and
 * y*/
inline int i_squaresum_diff_u(const unsigned char *x, const unsigned char *y,
                              int n) {
  int acc = 0;
  for (int i = 0; i < n; i++) {
    acc += i_sqr((int)x[i] - (int)y[i]);
  }
  return acc;
}
template <typename T>
inline T i_squaresum_diff(const T *x, const T *y, int n) {
  T acc = (T)(0.0);
  for (int i = 0; i < n; i++) {
    acc += i_sqr(x[i] - y[i]);
  }
  return acc;
}
template <typename T>
inline T i_squaresum_diff1(const T x[1], const T y[1]) {
  return (i_sqr(x[0] - y[0]));
}
template <typename T>
inline T i_squaresum_diff2(const T x[2], const T y[2]) {
  return (i_sqr(x[0] - y[0]) + i_sqr(x[1] - y[1]));
}
template <typename T>
inline T i_squaresum_diff3(const T x[3], const T y[3]) {
  return (i_sqr(x[0] - y[0]) + i_sqr(x[1] - y[1]) + i_sqr(x[2] - y[2]));
}
template <typename T>
inline T i_squaresum_diff4(const T x[4], const T y[4]) {
  return (i_sqr(x[0] - y[0]) + i_sqr(x[1] - y[1]) + i_sqr(x[2] - y[2]) +
          i_sqr(x[3] - y[3]));
}
template <typename T>
inline T i_squaresum_diff5(const T x[5], const T y[5]) {
  return (i_sqr(x[0] - y[0]) + i_sqr(x[1] - y[1]) + i_sqr(x[2] - y[2]) +
          i_sqr(x[3] - y[3]) + i_sqr(x[4] - y[4]));
}
template <typename T>
inline T i_squaresum_diff6(const T x[6], const T y[6]) {
  return (i_sqr(x[0] - y[0]) + i_sqr(x[1] - y[1]) + i_sqr(x[2] - y[2]) +
          i_sqr(x[3] - y[3]) + i_sqr(x[4] - y[4]) + i_sqr(x[5] - y[5]));
}
template <typename T>
inline T i_squaresum_diff7(const T x[7], const T y[7]) {
  return (i_sqr(x[0] - y[0]) + i_sqr(x[1] - y[1]) + i_sqr(x[2] - y[2]) +
          i_sqr(x[3] - y[3]) + i_sqr(x[4] - y[4]) + i_sqr(x[5] - y[5]) +
          i_sqr(x[6] - y[6]));
}
template <typename T>
inline T i_squaresum_diff8(const T x[8], const T y[8]) {
  return (i_sqr(x[0] - y[0]) + i_sqr(x[1] - y[1]) + i_sqr(x[2] - y[2]) +
          i_sqr(x[3] - y[3]) + i_sqr(x[4] - y[4]) + i_sqr(x[5] - y[5]) +
          i_sqr(x[6] - y[6]) + i_sqr(x[7] - y[7]));
}
template <typename T>
inline T i_squaresum_diff9(const T x[9], const T y[9]) {
  return (i_sqr(x[0] - y[0]) + i_sqr(x[1] - y[1]) + i_sqr(x[2] - y[2]) +
          i_sqr(x[3] - y[3]) + i_sqr(x[4] - y[4]) + i_sqr(x[5] - y[5]) +
          i_sqr(x[6] - y[6]) + i_sqr(x[7] - y[7]) + i_sqr(x[8] - y[8]));
}
template <typename T>
inline T i_squaresum_diff10(const T x[10], const T y[10]) {
  return (i_sqr(x[0] - y[0]) + i_sqr(x[1] - y[1]) + i_sqr(x[2] - y[2]) +
          i_sqr(x[3] - y[3]) + i_sqr(x[4] - y[4]) + i_sqr(x[5] - y[5]) +
          i_sqr(x[6] - y[6]) + i_sqr(x[7] - y[7]) + i_sqr(x[8] - y[8]) +
          i_sqr(x[9] - y[9]));
}
template <typename T>
inline T i_squaresum_diff11(const T x[11], const T y[11]) {
  return (i_sqr(x[0] - y[0]) + i_sqr(x[1] - y[1]) + i_sqr(x[2] - y[2]) +
          i_sqr(x[3] - y[3]) + i_sqr(x[4] - y[4]) + i_sqr(x[5] - y[5]) +
          i_sqr(x[6] - y[6]) + i_sqr(x[7] - y[7]) + i_sqr(x[8] - y[8]) +
          i_sqr(x[9] - y[9]) + i_sqr(x[10] - y[10]));
}
template <typename T>
inline T i_squaresum_diff12(const T x[12], const T y[12]) {
  return (i_sqr(x[0] - y[0]) + i_sqr(x[1] - y[1]) + i_sqr(x[2] - y[2]) +
          i_sqr(x[3] - y[3]) + i_sqr(x[4] - y[4]) + i_sqr(x[5] - y[5]) +
          i_sqr(x[6] - y[6]) + i_sqr(x[7] - y[7]) + i_sqr(x[8] - y[8]) +
          i_sqr(x[9] - y[9]) + i_sqr(x[10] - y[10]) + i_sqr(x[11] - y[11]));
}
template <typename T>
inline T i_squaresum_diff13(const T x[13], const T y[13]) {
  return (i_sqr(x[0] - y[0]) + i_sqr(x[1] - y[1]) + i_sqr(x[2] - y[2]) +
          i_sqr(x[3] - y[3]) + i_sqr(x[4] - y[4]) + i_sqr(x[5] - y[5]) +
          i_sqr(x[6] - y[6]) + i_sqr(x[7] - y[7]) + i_sqr(x[8] - y[8]) +
          i_sqr(x[9] - y[9]) + i_sqr(x[10] - y[10]) + i_sqr(x[11] - y[11]) +
          i_sqr(x[12] - y[12]));
}
template <typename T>
inline T i_squaresum_diff14(const T x[14], const T y[14]) {
  return (i_sqr(x[0] - y[0]) + i_sqr(x[1] - y[1]) + i_sqr(x[2] - y[2]) +
          i_sqr(x[3] - y[3]) + i_sqr(x[4] - y[4]) + i_sqr(x[5] - y[5]) +
          i_sqr(x[6] - y[6]) + i_sqr(x[7] - y[7]) + i_sqr(x[8] - y[8]) +
          i_sqr(x[9] - y[9]) + i_sqr(x[10] - y[10]) + i_sqr(x[11] - y[11]) +
          i_sqr(x[12] - y[12]) + i_sqr(x[13] - y[13]));
}
template <typename T>
inline T i_squaresum_diff15(const T x[15], const T y[15]) {
  return (i_sqr(x[0] - y[0]) + i_sqr(x[1] - y[1]) + i_sqr(x[2] - y[2]) +
          i_sqr(x[3] - y[3]) + i_sqr(x[4] - y[4]) + i_sqr(x[5] - y[5]) +
          i_sqr(x[6] - y[6]) + i_sqr(x[7] - y[7]) + i_sqr(x[8] - y[8]) +
          i_sqr(x[9] - y[9]) + i_sqr(x[10] - y[10]) + i_sqr(x[11] - y[11]) +
          i_sqr(x[12] - y[12]) + i_sqr(x[13] - y[13]) + i_sqr(x[14] - y[14]));
}
template <typename T>
inline T i_squaresum_diff16(const T x[16], const T y[16]) {
  return (i_sqr(x[0] - y[0]) + i_sqr(x[1] - y[1]) + i_sqr(x[2] - y[2]) +
          i_sqr(x[3] - y[3]) + i_sqr(x[4] - y[4]) + i_sqr(x[5] - y[5]) +
          i_sqr(x[6] - y[6]) + i_sqr(x[7] - y[7]) + i_sqr(x[8] - y[8]) +
          i_sqr(x[9] - y[9]) + i_sqr(x[10] - y[10]) + i_sqr(x[11] - y[11]) +
          i_sqr(x[12] - y[12]) + i_sqr(x[13] - y[13]) + i_sqr(x[14] - y[14]) +
          i_sqr(x[15] - y[15]));
}

/*Compute the Hamming distance between two (unsigned) integer arrays (considered
 * as binary values, that is, as sequences of bits)*/
inline unsigned int i_hamming_diff(const unsigned int *x, const unsigned int *y,
                                   int n) {
  unsigned int distance = 0;
  for (int i = 0; i < n; ++i) {
    distance += i_hamming_lut(x[i], y[i]);
  }
  return distance;  // Return the number of differing bits
}
inline unsigned int i_hamming_diff2(const unsigned int x[2],
                                    const unsigned int y[2]) {
  unsigned int distance = 0;
  distance += i_hamming_lut(x[0], y[0]);
  distance += i_hamming_lut(x[1], y[1]);
  return distance;
}
inline unsigned int i_hamming_diff4(const unsigned int x[4],
                                    const unsigned int y[4]) {
  unsigned int distance = 0;
  distance += i_hamming_lut(x[0], y[0]);
  distance += i_hamming_lut(x[1], y[1]);
  distance += i_hamming_lut(x[2], y[2]);
  distance += i_hamming_lut(x[3], y[3]);
  return distance;
}
inline unsigned int i_hamming_diff8(const unsigned int x[8],
                                    const unsigned int y[8]) {
  unsigned int distance = 0;
  distance += i_hamming_lut(x[0], y[0]);
  distance += i_hamming_lut(x[1], y[1]);
  distance += i_hamming_lut(x[2], y[2]);
  distance += i_hamming_lut(x[3], y[3]);
  distance += i_hamming_lut(x[4], y[4]);
  distance += i_hamming_lut(x[5], y[5]);
  distance += i_hamming_lut(x[6], y[6]);
  distance += i_hamming_lut(x[7], y[7]);
  return distance;
}
inline unsigned int i_hamming_diff16(const unsigned int x[16],
                                     const unsigned int y[16]) {
  unsigned int distance = 0;
  distance += i_hamming_lut(x[0], y[0]);
  distance += i_hamming_lut(x[1], y[1]);
  distance += i_hamming_lut(x[2], y[2]);
  distance += i_hamming_lut(x[3], y[3]);
  distance += i_hamming_lut(x[4], y[4]);
  distance += i_hamming_lut(x[5], y[5]);
  distance += i_hamming_lut(x[6], y[6]);
  distance += i_hamming_lut(x[7], y[7]);
  distance += i_hamming_lut(x[8], y[8]);
  distance += i_hamming_lut(x[9], y[9]);
  distance += i_hamming_lut(x[10], y[10]);
  distance += i_hamming_lut(x[11], y[11]);
  distance += i_hamming_lut(x[12], y[12]);
  distance += i_hamming_lut(x[13], y[13]);
  distance += i_hamming_lut(x[14], y[14]);
  distance += i_hamming_lut(x[15], y[15]);
  return distance;
}

/*Compute the L2 norm of n-dimensional vector x*/
inline double i_l2_norm(const unsigned char *x, int n) {
  return (i_sqrt(i_squaresum_u(x, n)));
}
inline double i_l2_norm(const int *x, int n) {
  return (i_sqrt(i_squaresum(x, n)));
}
inline float i_l2_norm(const float *x, int n) {
  return (i_sqrt(i_squaresum(x, n)));
}
inline double i_l2_norm(const double *x, int n) {
  return (i_sqrt(i_squaresum(x, n)));
}

/*For type float and double only*/
template <typename T>
inline T i_l2_norm1(const T x[1]) {
  return (x[0]);
}
template <typename T>
inline T i_l2_norm2(const T x[2]) {
  return (i_sqrt(i_squaresum2(x)));
}
template <typename T>
inline T i_l2_norm3(const T x[3]) {
  return (i_sqrt(i_squaresum3(x)));
}
template <typename T>
inline T i_l2_norm4(const T x[4]) {
  return (i_sqrt(i_squaresum4(x)));
}
template <typename T>
inline T i_l2_norm5(const T x[5]) {
  return (i_sqrt(i_squaresum5(x)));
}
template <typename T>
inline T i_l2_norm6(const T x[6]) {
  return (i_sqrt(i_squaresum6(x)));
}
template <typename T>
inline T i_l2_norm7(const T x[7]) {
  return (i_sqrt(i_squaresum7(x)));
}
template <typename T>
inline T i_l2_norm8(const T x[8]) {
  return (i_sqrt(i_squaresum8(x)));
}
template <typename T>
inline T i_l2_norm9(const T x[9]) {
  return (i_sqrt(i_squaresum9(x)));
}
template <typename T>
inline T i_l2_norm10(const T x[10]) {
  return (i_sqrt(i_squaresum10(x)));
}
template <typename T>
inline T i_l2_norm11(const T x[11]) {
  return (i_sqrt(i_squaresum11(x)));
}
template <typename T>
inline T i_l2_norm12(const T x[12]) {
  return (i_sqrt(i_squaresum12(x)));
}
template <typename T>
inline T i_l2_norm13(const T x[13]) {
  return (i_sqrt(i_squaresum13(x)));
}
template <typename T>
inline T i_l2_norm14(const T x[14]) {
  return (i_sqrt(i_squaresum14(x)));
}
template <typename T>
inline T i_l2_norm15(const T x[15]) {
  return (i_sqrt(i_squaresum15(x)));
}
template <typename T>
inline T i_l2_norm16(const T x[16]) {
  return (i_sqrt(i_squaresum16(x)));
}

// Compute sqrt( a^2 + b^2 ) with decent precision, for type float and double
// only
template <typename T>
inline T i_l2_norm2_adv(T a, T b) {
  T absa = i_abs(a);
  T absb = i_abs(b);
  if (absa > absb) {
    return (absa * i_sqrt((T)1.0 + i_sqr(i_div(absb, absa))));
  } else {
    return (absb == (T)0.0 ? (T)0.0
                           : absb * i_sqrt((T)1.0 + i_sqr(i_div(absa, absb))));
  }
}

// Compute sqrt( x[0]^2 + x[1]^2 ) with decent precision, for type float and
// double only
template <typename T>
inline T i_l2_norm2_adv(const T x[2]) {
  return i_l2_norm2_adv<T>(x[0], x[1]);
}

// Compute the infinity-norm of a m x n matrix A
template <typename T>
inline T i_infinity_norm(const T *A, int m, int n) {
  T infinity_norm = i_abs_sum(A, n);
  T tmp;
  int i, ni = n;
  for (i = 1; i < m; ++i, ni += n) {
    tmp = i_abs_sum(A + ni, n);
    if (infinity_norm < tmp) {
      infinity_norm = tmp;
    }
  }
  return infinity_norm;
}

/*Unitize n-dimensional vector x by dividing its L2-norm and save the result
 * inplace, for type float and double only*/
inline void i_unitize(double *x, int n) {
  i_scale(x, n, i_rec(i_l2_norm(x, n)));
}
inline void i_unitize(float *x, int n) {
  i_scale(x, n, i_rec(i_l2_norm(x, n)));
}
inline void i_safe_unitize(double *x, int n) {
  double norm = i_l2_norm(x, n);
  if (norm < Constant<double>::MIN_ABS_SAFE_DIVIDEND()) {
    i_fill(x, n, i_rec(i_sqrt(n)));
  } else {
    i_scale(x, n, 1.0 / norm);
  }
}
inline void i_safe_unitize(float *x, int n) {
  float norm = i_l2_norm(x, n);
  if (norm < Constant<float>::MIN_ABS_SAFE_DIVIDEND()) {
    i_fill(x, n, (float)(i_rec(i_sqrt(n))));
  } else {
    i_scale(x, n, (float)(1.0) / norm);
  }
}
/*Unitize n-dimensional vector x by dividing its L2-norm and save the result in
 * vector y, for type float and double only*/
inline void i_unitize(const double *x, double *y, int n) {
  i_scale(x, y, n, i_rec(i_l2_norm(x, n)));
}
inline void i_unitize(const float *x, float *y, int n) {
  i_scale(x, y, n, i_rec(i_l2_norm(x, n)));
}
inline void i_safe_unitize(const double *x, double *y, int n) {
  double norm = i_l2_norm(x, n);
  if (norm < Constant<double>::MIN_ABS_SAFE_DIVIDEND()) {
    i_fill(y, n, i_rec(i_sqrt(n)));
  } else {
    i_scale(x, y, n, 1.0 / norm);
  }
}
inline void i_safe_unitize(float *x, float *y, int n) {
  float norm = i_l2_norm(x, n);
  if (norm < Constant<float>::MIN_ABS_SAFE_DIVIDEND()) {
    i_fill(y, n, (float)(i_rec(i_sqrt(n))));
  } else {
    i_scale(x, y, n, (float)(1.0) / norm);
  }
}
/*For type float and double only!*/
template <typename T>
inline void i_unitize2(T x[2]) {
  i_scale2(x, i_rec(i_l2_norm2(x)));
}
template <typename T>
inline void i_unitize3(T x[3]) {
  i_scale3(x, i_rec(i_l2_norm3(x)));
}
template <typename T>
inline void i_unitize4(T x[4]) {
  i_scale4(x, i_rec(i_l2_norm4(x)));
}
template <typename T>
inline void i_unitize5(T x[5]) {
  i_scale5(x, i_rec(i_l2_norm5(x)));
}
template <typename T>
inline void i_unitize6(T x[6]) {
  i_scale6(x, i_rec(i_l2_norm6(x)));
}
template <typename T>
inline void i_unitize7(T x[7]) {
  i_scale7(x, i_rec(i_l2_norm7(x)));
}
template <typename T>
inline void i_unitize8(T x[8]) {
  i_scale8(x, i_rec(i_l2_norm8(x)));
}
template <typename T>
inline void i_unitize9(T x[9]) {
  i_scale9(x, i_rec(i_l2_norm9(x)));
}
template <typename T>
inline void i_unitize10(T x[10]) {
  i_scale10(x, i_rec(i_l2_norm10(x)));
}
template <typename T>
inline void i_unitize11(T x[11]) {
  i_scale11(x, i_rec(i_l2_norm11(x)));
}
template <typename T>
inline void i_unitize12(T x[12]) {
  i_scale12(x, i_rec(i_l2_norm12(x)));
}
template <typename T>
inline void i_unitize13(T x[13]) {
  i_scale13(x, i_rec(i_l2_norm13(x)));
}
template <typename T>
inline void i_unitize14(T x[14]) {
  i_scale14(x, i_rec(i_l2_norm14(x)));
}
template <typename T>
inline void i_unitize15(T x[15]) {
  i_scale15(x, i_rec(i_l2_norm15(x)));
}
template <typename T>
inline void i_unitize16(T x[16]) {
  i_scale16(x, i_rec(i_l2_norm16(x)));
}
template <typename T>
inline void i_unitize2(const T x[2], T y[2]) {
  i_scale2(x, y, i_rec(i_l2_norm2(x)));
}
template <typename T>
inline void i_unitize3(const T x[3], T y[3]) {
  i_scale3(x, y, i_rec(i_l2_norm3(x)));
}
template <typename T>
inline void i_unitize4(const T x[4], T y[4]) {
  i_scale4(x, y, i_rec(i_l2_norm4(x)));
}
template <typename T>
inline void i_unitize5(const T x[5], T y[5]) {
  i_scale5(x, y, i_rec(i_l2_norm5(x)));
}
template <typename T>
inline void i_unitize6(const T x[6], T y[6]) {
  i_scale6(x, y, i_rec(i_l2_norm6(x)));
}
template <typename T>
inline void i_unitize7(const T x[7], T y[7]) {
  i_scale7(x, y, i_rec(i_l2_norm7(x)));
}
template <typename T>
inline void i_unitize8(const T x[8], T y[8]) {
  i_scale8(x, y, i_rec(i_l2_norm8(x)));
}
template <typename T>
inline void i_unitize9(const T x[9], T y[9]) {
  i_scale9(x, y, i_rec(i_l2_norm9(x)));
}
template <typename T>
inline void i_unitize10(const T x[10], T y[10]) {
  i_scale10(x, y, i_rec(i_l2_norm10(x)));
}
template <typename T>
inline void i_unitize11(const T x[11], T y[11]) {
  i_scale11(x, y, i_rec(i_l2_norm11(x)));
}
template <typename T>
inline void i_unitize12(const T x[12], T y[12]) {
  i_scale12(x, y, i_rec(i_l2_norm12(x)));
}
template <typename T>
inline void i_unitize13(const T x[13], T y[13]) {
  i_scale13(x, y, i_rec(i_l2_norm13(x)));
}
template <typename T>
inline void i_unitize14(const T x[14], T y[14]) {
  i_scale14(x, y, i_rec(i_l2_norm14(x)));
}
template <typename T>
inline void i_unitize15(const T x[15], T y[15]) {
  i_scale15(x, y, i_rec(i_l2_norm15(x)));
}
template <typename T>
inline void i_unitize16(const T x[16], T y[16]) {
  i_scale16(x, y, i_rec(i_l2_norm16(x)));
}

template <typename T>
inline void i_signed_unitize2(T x[2]) {
  i_scale2(x, i_sign_never_zero(x[1]) * i_rec(i_l2_norm2(x)));
}
template <typename T>
inline void i_signed_unitize3(T x[3]) {
  i_scale3(x, i_sign_never_zero(x[2]) * i_rec(i_l2_norm3(x)));
}
template <typename T>
inline void i_signed_unitize4(T x[4]) {
  i_scale4(x, i_sign_never_zero(x[3]) * i_rec(i_l2_norm4(x)));
}

template <typename T>
inline void i_homogeneous_unitize(T *x, int n) {
  i_scale(x, n, i_rec(x[n - 1]));
}
template <typename T>
inline void i_homogeneous_unitize2(T x[2]) {
  i_scale2(x, i_rec(x[1]));
}
template <typename T>
inline void i_homogeneous_unitize3(T x[3]) {
  i_scale3(x, i_rec(x[2]));
}
template <typename T>
inline void i_homogeneous_unitize4(T x[4]) {
  i_scale4(x, i_rec(x[3]));
}
template <typename T>
inline void i_homogeneous_unitize9(T x[9]) {
  i_scale9(x, i_rec(x[8]));
}

template <typename T>
inline void i_homogeneous_unitize(const T *x, T *y, int n) {
  i_scale(x, y, n, i_rec(x[n - 1]));
}
template <typename T>
inline void i_homogeneous_unitize2(const T x[2], T y[2]) {
  i_scale2(x, y, i_rec(x[1]));
}
template <typename T>
inline void i_homogeneous_unitize3(const T x[3], T y[3]) {
  i_scale3(x, y, i_rec(x[2]));
}
template <typename T>
inline void i_homogeneous_unitize4(const T x[4], T y[4]) {
  i_scale4(x, y, i_rec(x[3]));
}
template <typename T>
inline void i_homogeneous_unitize9(const T x[9], T y[9]) {
  i_scale9(x, y, i_rec(x[8]));
}

/*Compute the centroid of n 3-dimensional vectors*/
inline void i_centroid3(const double *a, int n, double centroid[3]) {
  int length = 3 * n;
  i_fill3(centroid, 0.0);
  for (int i = 0; i < length; i += 3) {
    i_add3(a + i, centroid);
  }
  i_scale3(centroid, i_rec(n));
}
inline void i_centroid3(const float *a, int n, float centroid[3]) {
  int length = 3 * n;
  i_fill3(centroid, 0.f);
  for (int i = 0; i < length; i += 3) {
    i_add3(a + i, centroid);
  }
  i_scale3(centroid, (float)(i_rec(n)));
}
/*Compute the centroid of n 2-dimensional vectors*/
inline void i_centroid2(const double *a, int n, double centroid[2]) {
  int length = 2 * n;
  i_fill2(centroid, 0.0);
  for (int i = 0; i < length; i += 2) {
    i_add2(a + i, centroid);
  }
  i_scale2(centroid, i_rec(n));
}
inline void i_centroid2(const float *a, int n, float centroid[2]) {
  int length = 2 * n;
  i_fill2(centroid, 0.f);
  for (int i = 0; i < length; i += 2) {
    i_add2(a + i, centroid);
  }
  i_scale2(centroid, (float)(i_rec(n)));
}

/*Compute the centroid of n 3-dimensional vectors and their Euclidean distances
 * to the centroid*/
inline void i_centroid3(const double *a, int n, double centroid[3],
                        double *distances) {
  int length = 3 * n;
  int i, j;
  i_fill3(centroid, 0.0);
  for (i = 0; i < length; i += 3) {
    i_add3(a + i, centroid);
  }
  i_scale3(centroid, i_rec(n));
  for (i = 0, j = 0; i < n; ++i, j += 3) {
    distances[i] = i_sqrt(i_squaresum_diff3(a + j, centroid));
  }
}
inline void i_centroid3(const float *a, int n, float centroid[3],
                        float *distances) {
  int length = 3 * n;
  int i, j;
  i_fill3(centroid, 0.f);
  for (i = 0; i < length; i += 3) {
    i_add3(a + i, centroid);
  }
  i_scale3(centroid, (float)(i_rec(n)));
  for (i = 0, j = 0; i < n; ++i, j += 3) {
    distances[i] = i_sqrt(i_squaresum_diff3(a + j, centroid));
  }
}
/*Compute the centroid of n 2-dimensional vectors and their Euclidean distances
 * to the centroid*/
inline void i_centroid2(const double *a, int n, double centroid[2],
                        double *distances) {
  int length = 2 * n;
  int i, j;
  i_fill2(centroid, 0.0);
  for (i = 0; i < length; i += 2) {
    i_add2(a + i, centroid);
  }
  i_scale2(centroid, i_rec(n));
  for (i = 0, j = 0; i < n; ++i, j += 2) {
    distances[i] = i_sqrt(i_squaresum_diff2(a + j, centroid));
  }
}
inline void i_centroid2(const float *a, int n, float centroid[2],
                        float *distances) {
  int length = 2 * n;
  int i, j;
  i_fill2(centroid, 0.f);
  for (i = 0; i < length; i += 2) {
    i_add2(a + i, centroid);
  }
  i_scale2(centroid, (float)(i_rec(n)));
  for (i = 0, j = 0; i < n; ++i, j += 2) {
    distances[i] = i_sqrt(i_squaresum_diff2(a + j, centroid));
  }
}

/*Compute the minimum element in array*/
template <typename T>
inline T i_min_element(const T *a, int n) {
  T val, temp;
  if (n <= 0) return ((T)0.0);
  val = a[0];
  for (int i = 1; i < n; i++) {
    temp = a[i];
    if (temp < val) {
      val = temp;
    }
  }
  return (val);
}
/*Compute the maximum element in array*/
template <typename T>
inline T i_max_element(const T *a, int n) {
  T val, temp;
  if (n <= 0) return ((T)0.0);
  val = a[0];
  for (int i = 1; i < n; i++) {
    temp = a[i];
    if (temp > val) {
      val = temp;
    }
  }
  return (val);
}
/*Compute the minimum element and its index in array*/
template <typename T>
inline void i_min_element(const T *a, int n, T &min_val, int &index) {
  T temp;
  index = 0;
  if (n <= 0) {
    min_val = (T)0.0;
    return;
  }
  min_val = a[0];
  for (int i = 1; i < n; i++) {
    temp = a[i];
    if (temp < min_val) {
      min_val = temp;
      index = i;
    }
  }
}
/*Compute the maximum element and its index in array*/
template <typename T>
inline void i_max_element(const T *a, int n, T &max_val, int &index) {
  T temp;
  index = 0;
  if (n <= 0) {
    max_val = (T)0.0;
    return;
  }
  max_val = a[0];
  for (int i = 1; i < n; i++) {
    temp = a[i];
    if (temp > max_val) {
      max_val = temp;
      index = i;
    }
  }
}

/*Compute the maximum diagonal element of a n x n square matrix*/
template <typename T>
inline T i_max_diagonal_element(const T *a, int n) {
  T val, temp;
  if (n <= 0) return ((T)0.0);
  val = a[0];
  int i, ni = n;
  for (i = 1; i < n; i++, ni += n) {
    temp = a[ni + i];
    if (temp > val) {
      val = temp;
    }
  }
  return (val);
}

/*Compute the the index of element with largest element in array*/
inline int i_max_index(const double *a, int n) {
  int bi;
  double b, t;
  if (n <= 0) return (0);
  b = a[0];
  bi = 0;
  for (int i = 1; i < n; i++)
    if ((t = a[i]) > b) {
      b = t;
      bi = i;
    }
  return (bi);
}
inline int i_max_index(const float *a, int n) {
  int bi;
  float b, t;
  if (n <= 0) return (0);
  b = a[0];
  bi = 0;
  for (int i = 1; i < n; i++)
    if ((t = a[i]) > b) {
      b = t;
      bi = i;
    }
  return (bi);
}
inline int i_max_index(const int *a, int n) {
  int bi;
  int b, t;
  if (n <= 0) return (0);
  b = a[0];
  bi = 0;
  for (int i = 1; i < n; i++)
    if ((t = a[i]) > b) {
      b = t;
      bi = i;
    }
  return (bi);
}

/*Compute the the index of element with largest  magnitude element in array*/
inline int i_max_abs_index(const double *a, int n) {
  int bi;
  double b, t;
  if (n <= 0) return (0);
  b = i_abs(a[0]);
  bi = 0;
  for (int i = 1; i < n; i++)
    if ((t = i_abs(a[i])) > b) {
      b = t;
      bi = i;
    }
  return (bi);
}
inline int i_max_abs_index(const float *a, int n) {
  int bi;
  float b, t;
  if (n <= 0) return (0);
  b = i_abs(a[0]);
  bi = 0;
  for (int i = 1; i < n; i++)
    if ((t = i_abs(a[i])) > b) {
      b = t;
      bi = i;
    }
  return (bi);
}
inline int i_max_abs_index(const int *a, int n) {
  int bi;
  int b, t;
  if (n <= 0) return (0);
  b = i_abs(a[0]);
  bi = 0;
  for (int i = 1; i < n; i++)
    if ((t = i_abs(a[i])) > b) {
      b = t;
      bi = i;
    }
  return (bi);
}
/*Compute the the index of element with smallest magnitude element in array*/
inline int i_min_abs_index(const double *a, int n) {
  int bi;
  double b, t;
  if (n <= 0) return (0);
  b = i_abs(a[0]);
  bi = 0;
  for (int i = 1; i < n; i++)
    if ((t = i_abs(a[i])) < b) {
      b = t;
      bi = i;
    }
  return (bi);
}
inline int i_min_abs_index(const float *a, int n) {
  int bi;
  float b, t;
  if (n <= 0) return (0);
  b = i_abs(a[0]);
  bi = 0;
  for (int i = 1; i < n; i++)
    if ((t = i_abs(a[i])) < b) {
      b = t;
      bi = i;
    }
  return (bi);
}
inline int i_min_abs_index(const int *a, int n) {
  int bi;
  int b, t;
  if (n <= 0) return (0);
  b = i_abs(a[0]);
  bi = 0;
  for (int i = 1; i < n; i++)
    if ((t = i_abs(a[i])) < b) {
      b = t;
      bi = i;
    }
  return (bi);
}

/*Compute the index of element in interval [i1,i2) with largest magnitude
 * element in array*/
inline int i_max_abs_index_interval(const double *a, int i1, int i2) {
  int bi;
  double b, t;
  b = i_abs(a[i1]);
  bi = i1;
  for (int i = i1 + 1; i < i2; i++) {
    if ((t = i_abs(a[i])) > b) {
      b = t;
      bi = i;
    }
  }
  return (bi);
}
inline int i_max_abs_index_interval(const float *a, int i1, int i2) {
  int bi;
  float b, t;
  b = i_abs(a[i1]);
  bi = i1;
  for (int i = i1 + 1; i < i2; i++) {
    if ((t = i_abs(a[i])) > b) {
      b = t;
      bi = i;
    }
  }
  return (bi);
}
inline int i_max_abs_index_interval(const int *a, int i1, int i2) {
  int bi;
  int b, t;
  b = i_abs(a[i1]);
  bi = i1;
  for (int i = i1 + 1; i < i2; i++) {
    if ((t = i_abs(a[i])) > b) {
      b = t;
      bi = i;
    }
  }
  return (bi);
}
/*Compute the index of element in interval [i1,i2) with smallest magnitude
 * element in array*/
inline int i_min_abs_index_interval(const double *a, int i1, int i2) {
  int bi;
  double b, t;
  b = i_abs(a[i1]);
  bi = i1;
  for (int i = i1 + 1; i < i2; i++) {
    if ((t = i_abs(a[i])) < b) {
      b = t;
      bi = i;
    }
  }
  return (bi);
}
inline int i_min_abs_index_interval(const float *a, int i1, int i2) {
  int bi;
  float b, t;
  b = i_abs(a[i1]);
  bi = i1;
  for (int i = i1 + 1; i < i2; i++) {
    if ((t = i_abs(a[i])) < b) {
      b = t;
      bi = i;
    }
  }
  return (bi);
}
inline int i_min_abs_index_interval(const int *a, int i1, int i2) {
  int bi;
  int b, t;
  b = i_abs(a[i1]);
  bi = i1;
  for (int i = i1 + 1; i < i2; i++) {
    if ((t = i_abs(a[i])) < b) {
      b = t;
      bi = i;
    }
  }
  return (bi);
}

/*Compute the index of element in interval [i1,i2) with largest magnitude
 * element in a column of a mxn matrix*/
inline int i_max_abs_index_interval_column(
    const double *a, int i1, int i2, int n /*# of columns of the matrix*/) {
  int bi;
  double b, t;
  const double *ref = a + n * i1;
  b = i_abs(*ref);
  bi = i1;
  ref += n;
  for (int i = i1 + 1; i < i2; ++i, ref += n) {
    if ((t = i_abs(*ref)) > b) {
      b = t;
      bi = i;
    }
  }
  return (bi);
}
inline int i_max_abs_index_interval_column(
    const float *a, int i1, int i2, int n /*# of columns of the matrix*/) {
  int bi;
  float b, t;
  const float *ref = a + i1 * n;
  b = i_abs(*ref);
  bi = i1;
  for (int i = i1 + 1; i < i2; ++i, ref += n) {
    if ((t = i_abs(*ref)) > b) {
      b = t;
      bi = i;
    }
  }
  return (bi);
}
inline int i_max_abs_index_interval_column(
    const int *a, int i1, int i2, int n /*# of columns of the matrix*/) {
  int b, bi, t;
  const int *ref = a + i1 * n;
  b = i_abs(*ref);
  bi = i1;
  for (int i = i1; i < i2; ++i, ref += n) {
    if ((t = i_abs(*ref)) > b) {
      b = t;
      bi = i;
    }
  }
  return (bi);
}

/*Compute the index of element in interval [i1,i2) with smallest magnitude
 * element in a column of a mxn matrix*/
inline int i_min_abs_index_interval_column(
    const double *a, int i1, int i2, int n /*# of columns of the matrix*/) {
  int bi;
  double b, t;
  const double *ref = a + n * i1;
  b = i_abs(*ref);
  bi = i1;
  for (int i = i1 + 1; i < i2; ++i, ref += n) {
    if ((t = i_abs(*ref)) < b) {
      b = t;
      bi = i;
    }
  }
  return (bi);
}
inline int i_min_abs_index_interval_column(
    const float *a, int i1, int i2, int n /*# of columns of the matrix*/) {
  int bi;
  float b, t;
  const float *ref = a + i1 * n;
  b = i_abs(*ref);
  bi = i1;
  for (int i = i1 + 1; i < i2; ++i, ref += n) {
    if ((t = i_abs(*ref)) < b) {
      b = t;
      bi = i;
    }
  }
  return (bi);
}
inline int i_min_abs_index_interval_column(
    const int *a, int i1, int i2, int n /*# of columns of the matrix*/) {
  int b, bi, t;
  const int *ref = a + i1 * n;
  b = i_abs(*ref);
  bi = i1;
  for (int i = i1; i < i2; ++i, ref += n) {
    if ((t = i_abs(*ref)) < b) {
      b = t;
      bi = i;
    }
  }
  return (bi);
}

/*Find row-index of element on or below the diagonal of column i of (n x n)
matrix A
with the largest absolute value. */
template <typename T>
inline int i_max_abs_index_subdiagonal_column(const T *A, int i, int n) {
  int j, largest_j;
  T largest_val, temp;
  largest_val = i_abs(A[i * n + i]);
  largest_j = i;
  for (j = i + 1; j < n; j++) {
    temp = i_abs(A[j * n + i]);
    if (temp > largest_val) {
      largest_val = temp;
      largest_j = j;
    }
  }
  return largest_j;
}

/*Compute the minimum and maximum elements in an array*/
template <typename T>
inline void i_min_max_elements(const T *a, int n, T &min_val, T &max_val) {
  T temp;
  if (n <= 0) {
    min_val = max_val = (T)0.0;
    return;
  }
  min_val = max_val = a[0];
  for (int i = 1; i < n; i++) {
    temp = a[i];
    if (temp > max_val) {
      max_val = temp;
    }
    if (temp < min_val) {
      min_val = temp;
    }
  }
  return;
}

/*Compute the minimum and maximum elements in an array, ingoring the
 * "ignored_val" in a*/
template <typename T>
inline void i_min_max_elements(const T *a, int n, T &min_val, T &max_val,
                               const T ignored_val) {
  T temp;
  int i;
  if (n <= 0) {
    min_val = max_val = (T)0.0;
    return;
  }
  for (i = 0; i < n; ++i) {
    if (a[i] != ignored_val) {
      break;
    }
  }
  min_val = max_val = a[i];
  for (; i < n; i++) {
    temp = a[i];
    if (temp == ignored_val) {
      continue;
    }
    if (temp > max_val) {
      max_val = temp;
    }
    if (temp < min_val) {
      min_val = temp;
    }
  }
  return;
}

/*Given a n-dimensional vector x, construct its homogeneous representation
 * (n+1-dimensional vector) y by adding 1 to the last entry*/
template <typename T>
inline void i_homogenize(const T *x, T *y, int n) {
  for (int i = 0; i < n; ++i) {
    y[i] = x[i];
  }
  y[n] = (T)(1.0);
}
template <typename T>
inline void i_homogenize1(const T x[1], T y[2]) {
  y[0] = x[0];
  y[1] = (T)(1.0);
}
template <typename T>
inline void i_homogenize2(const T x[2], T y[3]) {
  y[0] = x[0];
  y[1] = x[1];
  y[2] = (T)(1.0);
}
template <typename T>
inline void i_homogenize3(const T x[3], T y[4]) {
  y[0] = x[0];
  y[1] = x[1];
  y[2] = x[2];
  y[3] = (T)(1.0);
}

/*Compute the cross product between two 3-dimensional vectors x and y*/
template <typename T>
inline void i_cross(const T x[3], const T y[3], T xxy[3]) {
  xxy[0] = x[1] * y[2] - x[2] * y[1];
  xxy[1] = x[2] * y[0] - x[0] * y[2];
  xxy[2] = x[0] * y[1] - x[1] * y[0];
}

/*Compute the 3x3 skew-symmetric matrix [e]_x such that [e]_x * y is the cross
 * product of 3-dimensional vectors x and y for all y*/
template <typename T>
inline void i_axiator(const T x[3], T e_x[9]) {
  e_x[0] = (T)(0.0);
  e_x[1] = -x[2];
  e_x[2] = x[1];
  e_x[3] = x[2];
  e_x[4] = (T)(0.0);
  e_x[5] = -x[0];
  e_x[6] = -x[1];
  e_x[7] = x[0];
  e_x[8] = (T)(0.0);
}

/*Compute the square of a 3x3 skew-symmetric matrix [e]_x, e_x2 = [e_x]^2 =
 * [e]_x*[e]_x*/
template <typename T>
inline void i_sqr_skew_symmetric_3x3(const T x[3], T e_x2[9]) {
  T x0_sqr = i_sqr(x[0]);
  T x1_sqr = i_sqr(x[1]);
  T x2_sqr = i_sqr(x[2]);
  e_x2[0] = -(x1_sqr + x2_sqr);
  e_x2[1] = e_x2[3] = x[0] * x[1];
  e_x2[4] = -(x2_sqr + x0_sqr);
  e_x2[2] = e_x2[6] = x[2] * x[0];
  e_x2[8] = -(x0_sqr + x1_sqr);
  e_x2[5] = e_x2[7] = x[1] * x[2];
}

/*Set the nxn matrix A to be an identity matrix*/
template <typename T>
inline void i_eye(T *A, int n) {
  int in = 0;
  i_zero(A, n * n);
  for (int i = 0; i < n; ++i, in += n) {
    A[in + i] = (T)1.0;
  }
}
template <typename T>
inline void i_eye_2x2(T A[4]) {
  A[0] = A[3] = (T)1.0;
  A[1] = A[2] = (T)0.0;
}
template <typename T>
inline void i_eye_3x3(T A[9]) {
  A[0] = A[4] = A[8] = (T)1.0;
  A[1] = A[2] = A[3] = A[5] = A[6] = A[7] = (T)0.0;
}
template <typename T>
inline void i_eye_4x4(T A[16]) {
  A[0] = A[5] = A[10] = A[15] = (T)1.0;
  A[1] = A[2] = A[3] = A[4] = A[6] = A[7] = A[8] = A[9] = A[11] = A[12] =
      A[13] = A[14] = (T)0.0;
}

/*Construct the 2x2 upper triangular matrix A*/
template <typename T>
inline void i_upper_triangular_2x2(T a0, T a1, T a3, T A[4]) {
  A[0] = a0;
  A[1] = a1;
  A[2] = (T)0.0;
  A[3] = a3;
}

/*Construct the 3x3 upper triangular matrix A*/
template <typename T>
inline void i_upper_triangular_3x3(T a0, T a1, T a2, T a4, T a5, T a8, T A[9]) {
  A[0] = a0;
  A[1] = a1;
  A[2] = a2;
  A[3] = (T)0.0;
  A[4] = a4;
  A[5] = a5;
  A[6] = (T)0.0;
  A[7] = (T)0.0;
  A[8] = a8;
}

/*Compute the trace of a 2x2 matrix A*/
inline double i_trace_2x2(const double A[4]) { return (A[0] + A[3]); }
inline float i_trace_2x2(const float A[4]) { return (A[0] + A[3]); }
/*Compute the trace of a 3x3 matrix A*/
inline double i_trace_3x3(const double A[9]) { return (A[0] + A[4] + A[8]); }
inline float i_trace_3x3(const float A[9]) { return (A[0] + A[4] + A[8]); }

/*Compute the determinant of a 2x2 matrix A*/
inline double i_determinant_2x2(const double A[4]) {
  return (A[0] * A[3] - A[1] * A[2]);
}
inline float i_determinant_2x2(const float A[4]) {
  return (A[0] * A[3] - A[1] * A[2]);
}
inline int i_determinant_2x2(const int A[4]) {
  return (A[0] * A[3] - A[1] * A[2]);
}

/*Compute the determinant of a 3x3 matrix A*/
inline double i_determinant_3x3(const double A[9]) {
  double r0_x_r1[3];
  i_cross(A, A + 3, r0_x_r1);
  return (i_dot3(r0_x_r1, A + 6));
}
inline float i_determinant_3x3(const float A[9]) {
  float r0_x_r1[3];
  i_cross(A, A + 3, r0_x_r1);
  return (i_dot3(r0_x_r1, A + 6));
}
inline int i_determinant_3x3(const int A[9]) {
  int r0_x_r1[3];
  i_cross(A, A + 3, r0_x_r1);
  return (i_dot3(r0_x_r1, A + 6));
}

/*Compute the 6 subdeterminants {sd[0],...,sd[5]} of the 2x4 matrix formed by x
 * as row 0 and y as row 1*/
inline void i_subdeterminants_2x4(const double x[4], const double y[4],
                                  double sd[6]) {
  sd[0] = x[0] * y[1] - x[1] * y[0];
  sd[1] = x[0] * y[2] - x[2] * y[0];
  sd[2] = x[0] * y[3] - x[3] * y[0];
  sd[3] = x[1] * y[2] - x[2] * y[1];
  sd[4] = x[1] * y[3] - x[3] * y[1];
  sd[5] = x[2] * y[3] - x[3] * y[2];
}
inline void i_subdeterminants_2x4(const float x[4], const float y[4],
                                  float sd[6]) {
  sd[0] = x[0] * y[1] - x[1] * y[0];
  sd[1] = x[0] * y[2] - x[2] * y[0];
  sd[2] = x[0] * y[3] - x[3] * y[0];
  sd[3] = x[1] * y[2] - x[2] * y[1];
  sd[4] = x[1] * y[3] - x[3] * y[1];
  sd[5] = x[2] * y[3] - x[3] * y[2];
}
inline void i_subdeterminants_2x4(const int x[4], const int y[4], int sd[6]) {
  sd[0] = x[0] * y[1] - x[1] * y[0];
  sd[1] = x[0] * y[2] - x[2] * y[0];
  sd[2] = x[0] * y[3] - x[3] * y[0];
  sd[3] = x[1] * y[2] - x[2] * y[1];
  sd[4] = x[1] * y[3] - x[3] * y[1];
  sd[5] = x[2] * y[3] - x[3] * y[2];
}

/*Compute the 4 subdeterminants {sd[0],...,sd[3]} of the 3x4 matrix formed by x
 * as row 0, y as row 1, and z as row 2*/
inline void i_subdeterminants_3x4(const double x[4], const double y[4],
                                  const double z[4], double sd[4]) {
  double ssd[6];
  i_subdeterminants_2x4(x, y, ssd);
  sd[0] = z[1] * ssd[5] - z[2] * ssd[4] + z[3] * ssd[3];
  sd[1] = z[0] * ssd[5] - z[2] * ssd[2] + z[3] * ssd[1];
  sd[2] = z[0] * ssd[4] - z[1] * ssd[2] + z[3] * ssd[0];
  sd[3] = z[0] * ssd[3] - z[1] * ssd[1] + z[2] * ssd[0];
}
inline void i_subdeterminants_3x4(const float x[4], const float y[4],
                                  const float z[4], float sd[4]) {
  float ssd[6];
  i_subdeterminants_2x4(x, y, ssd);
  sd[0] = z[1] * ssd[5] - z[2] * ssd[4] + z[3] * ssd[3];
  sd[1] = z[0] * ssd[5] - z[2] * ssd[2] + z[3] * ssd[1];
  sd[2] = z[0] * ssd[4] - z[1] * ssd[2] + z[3] * ssd[0];
  sd[3] = z[0] * ssd[3] - z[1] * ssd[1] + z[2] * ssd[0];
}
inline void i_subdeterminants_3x4(const int x[4], const int y[4],
                                  const int z[4], int sd[4]) {
  int ssd[6];
  i_subdeterminants_2x4(x, y, ssd);
  sd[0] = z[1] * ssd[5] - z[2] * ssd[4] + z[3] * ssd[3];
  sd[1] = z[0] * ssd[5] - z[2] * ssd[2] + z[3] * ssd[1];
  sd[2] = z[0] * ssd[4] - z[1] * ssd[2] + z[3] * ssd[0];
  sd[3] = z[0] * ssd[3] - z[1] * ssd[1] + z[2] * ssd[0];
}

/*Compute the determinant of a 4x4 matrix A*/
inline double i_determinant_4x4(const double A[16]) {
  double sd[4];
  i_subdeterminants_3x4(A, A + 4, A + 8, sd);
  return -(A[12] * sd[0]) + (A[13] * sd[1]) - (A[14] * sd[2]) + (A[15] * sd[3]);
}
inline float i_determinant_4x4(const float A[16]) {
  float sd[4];
  i_subdeterminants_3x4(A, A + 4, A + 8, sd);
  return -(A[12] * sd[0]) + (A[13] * sd[1]) - (A[14] * sd[2]) + (A[15] * sd[3]);
}
inline int i_determinant_4x4(const int A[16]) {
  int sd[4];
  i_subdeterminants_3x4(A, A + 4, A + 8, sd);
  return -(A[12] * sd[0]) + (A[13] * sd[1]) - (A[14] * sd[2]) + (A[15] * sd[3]);
}

/*Compute the nullvector to x,y and z => intersection of three planes x, y, z is
 * a point*/
template <typename T>
inline void i_cross(const T x[4], const T y[4], const T z[4], T xxyxz[4]) {
  i_subdeterminants_3x4(x, y, z, xxyxz);
  xxyxz[0] = -xxyxz[0];
  xxyxz[2] = -xxyxz[2];
}

/*Compute the inverse of a 2x2 matrix A using the Cramer's rule*/
inline void i_invert_2x2(const double A[4], double Ai[4]) {
  double d = i_determinant_2x2(A);
  double sf = i_rec(d);
  Ai[0] = sf * (A[3]);
  Ai[1] = sf * (-A[1]);
  Ai[2] = sf * (-A[2]);
  Ai[3] = sf * (A[0]);
}

inline void i_invert_2x2(const float A[4], float Ai[4]) {
  float d = i_determinant_2x2(A);
  float sf = i_rec(d);
  Ai[0] = sf * (A[3]);
  Ai[1] = sf * (-A[1]);
  Ai[2] = sf * (-A[2]);
  Ai[3] = sf * (A[0]);
}

inline void i_invert_2x2(const int A[4], double Ai[4]) {
  int d = i_determinant_2x2(A);
  double sf = i_rec(d);
  Ai[0] = sf * (A[3]);
  Ai[1] = sf * (-A[1]);
  Ai[2] = sf * (-A[2]);
  Ai[3] = sf * (A[0]);
}

inline void i_safe_invert_2x2(const double A[4], double Ai[4],
                              bool &is_invertible) {
  double d = i_determinant_2x2(A);
  double sf = i_rec(d);
  Ai[0] = sf * (A[3]);
  Ai[1] = sf * (-A[1]);
  Ai[2] = sf * (-A[2]);
  Ai[3] = sf * (A[0]);
  is_invertible =
      (i_abs(d) > Constant<double>::MIN_ABS_SAFE_DIVIDEND()) ? true : false;
}

inline void i_safe_invert_2x2(const float A[4], float Ai[4],
                              bool &is_invertible) {
  float d = i_determinant_2x2(A);
  float sf = i_rec(d);
  Ai[0] = sf * (A[3]);
  Ai[1] = sf * (-A[1]);
  Ai[2] = sf * (-A[2]);
  Ai[3] = sf * (A[0]);
  is_invertible =
      (i_abs(d) > Constant<float>::MIN_ABS_SAFE_DIVIDEND()) ? true : false;
}

inline void i_safe_invert_2x2(const int A[4], double Ai[4],
                              bool &is_invertible) {
  int d = i_determinant_2x2(A);
  double sf = i_rec(d);
  Ai[0] = sf * (A[3]);
  Ai[1] = sf * (-A[1]);
  Ai[2] = sf * (-A[2]);
  Ai[3] = sf * (A[0]);
  is_invertible = (d != 0) ? true : false;
}

/*Compute the inverse of a 3x3 matrix A using the Cramer's rule*/
inline void i_invert_3x3(const double A[9], double Ai[9]) {
  double sd0 = A[4] * A[8] - A[5] * A[7];
  double sd1 = A[5] * A[6] - A[3] * A[8];
  double sd2 = A[3] * A[7] - A[4] * A[6];
  double sd3 = A[2] * A[7] - A[1] * A[8];
  double sd4 = A[0] * A[8] - A[2] * A[6];
  double sd5 = A[1] * A[6] - A[0] * A[7];
  double sd6 = A[1] * A[5] - A[2] * A[4];
  double sd7 = A[2] * A[3] - A[0] * A[5];
  double sd8 = A[0] * A[4] - A[1] * A[3];
  double d = A[0] * sd0 + A[1] * sd1 + A[2] * sd2;
  double d_rec = i_rec(d);
  Ai[0] = d_rec * sd0;
  Ai[1] = d_rec * sd3;
  Ai[2] = d_rec * sd6;
  Ai[3] = d_rec * sd1;
  Ai[4] = d_rec * sd4;
  Ai[5] = d_rec * sd7;
  Ai[6] = d_rec * sd2;
  Ai[7] = d_rec * sd5;
  Ai[8] = d_rec * sd8;
}
inline void i_invert_3x3(const float A[9], float Ai[9]) {
  float sd0 = A[4] * A[8] - A[5] * A[7];
  float sd1 = A[5] * A[6] - A[3] * A[8];
  float sd2 = A[3] * A[7] - A[4] * A[6];
  float sd3 = A[2] * A[7] - A[1] * A[8];
  float sd4 = A[0] * A[8] - A[2] * A[6];
  float sd5 = A[1] * A[6] - A[0] * A[7];
  float sd6 = A[1] * A[5] - A[2] * A[4];
  float sd7 = A[2] * A[3] - A[0] * A[5];
  float sd8 = A[0] * A[4] - A[1] * A[3];
  float d = A[0] * sd0 + A[1] * sd1 + A[2] * sd2;
  float d_rec = i_rec(d);
  Ai[0] = d_rec * sd0;
  Ai[1] = d_rec * sd3;
  Ai[2] = d_rec * sd6;
  Ai[3] = d_rec * sd1;
  Ai[4] = d_rec * sd4;
  Ai[5] = d_rec * sd7;
  Ai[6] = d_rec * sd2;
  Ai[7] = d_rec * sd5;
  Ai[8] = d_rec * sd8;
}
inline void i_invert_3x3(const int A[9], double Ai[9]) {
  /*subdeterminants:*/
  int sd0 = A[4] * A[8] - A[5] * A[7];
  int sd1 = A[5] * A[6] - A[3] * A[8];
  int sd2 = A[3] * A[7] - A[4] * A[6];
  int sd3 = A[2] * A[7] - A[1] * A[8];
  int sd4 = A[0] * A[8] - A[2] * A[6];
  int sd5 = A[1] * A[6] - A[0] * A[7];
  int sd6 = A[1] * A[5] - A[2] * A[4];
  int sd7 = A[2] * A[3] - A[0] * A[5];
  int sd8 = A[0] * A[4] - A[1] * A[3];
  int d = A[0] * sd0 + A[1] * sd1 + A[2] * sd2;
  double d_rec = i_rec(d);
  Ai[0] = d_rec * sd0;
  Ai[1] = d_rec * sd3;
  Ai[2] = d_rec * sd6;
  Ai[3] = d_rec * sd1;
  Ai[4] = d_rec * sd4;
  Ai[5] = d_rec * sd7;
  Ai[6] = d_rec * sd2;
  Ai[7] = d_rec * sd5;
  Ai[8] = d_rec * sd8;
}
inline void i_safe_invert_3x3(const double A[9], double Ai[9],
                              bool &is_invertible) {
  /*subdeterminants:*/
  double sd0 = A[4] * A[8] - A[5] * A[7];
  double sd1 = A[5] * A[6] - A[3] * A[8];
  double sd2 = A[3] * A[7] - A[4] * A[6];
  double sd3 = A[2] * A[7] - A[1] * A[8];
  double sd4 = A[0] * A[8] - A[2] * A[6];
  double sd5 = A[1] * A[6] - A[0] * A[7];
  double sd6 = A[1] * A[5] - A[2] * A[4];
  double sd7 = A[2] * A[3] - A[0] * A[5];
  double sd8 = A[0] * A[4] - A[1] * A[3];
  double d = A[0] * sd0 + A[1] * sd1 + A[2] * sd2;
  is_invertible =
      (i_abs(d) > Constant<double>::MIN_ABS_SAFE_DIVIDEND()) ? true : false;
  double d_rec = is_invertible ? i_rec(d) : 1.0;
  Ai[0] = d_rec * sd0;
  Ai[1] = d_rec * sd3;
  Ai[2] = d_rec * sd6;
  Ai[3] = d_rec * sd1;
  Ai[4] = d_rec * sd4;
  Ai[5] = d_rec * sd7;
  Ai[6] = d_rec * sd2;
  Ai[7] = d_rec * sd5;
  Ai[8] = d_rec * sd8;
}
inline void i_safe_invert_3x3(const float A[9], float Ai[9],
                              bool &is_invertible) {
  /*subdeterminants:*/
  float sd0 = A[4] * A[8] - A[5] * A[7];
  float sd1 = A[5] * A[6] - A[3] * A[8];
  float sd2 = A[3] * A[7] - A[4] * A[6];
  float sd3 = A[2] * A[7] - A[1] * A[8];
  float sd4 = A[0] * A[8] - A[2] * A[6];
  float sd5 = A[1] * A[6] - A[0] * A[7];
  float sd6 = A[1] * A[5] - A[2] * A[4];
  float sd7 = A[2] * A[3] - A[0] * A[5];
  float sd8 = A[0] * A[4] - A[1] * A[3];
  float d = A[0] * sd0 + A[1] * sd1 + A[2] * sd2;
  is_invertible =
      (i_abs(d) > Constant<float>::MIN_ABS_SAFE_DIVIDEND()) ? true : false;
  float d_rec = is_invertible ? i_rec(d) : 1.f;
  Ai[0] = d_rec * sd0;
  Ai[1] = d_rec * sd3;
  Ai[2] = d_rec * sd6;
  Ai[3] = d_rec * sd1;
  Ai[4] = d_rec * sd4;
  Ai[5] = d_rec * sd7;
  Ai[6] = d_rec * sd2;
  Ai[7] = d_rec * sd5;
  Ai[8] = d_rec * sd8;
}
inline void i_safe_invert_3x3(const int A[9], double Ai[9],
                              bool &is_invertible) {
  /*subdeterminants:*/
  int sd0 = A[4] * A[8] - A[5] * A[7];
  int sd1 = A[5] * A[6] - A[3] * A[8];
  int sd2 = A[3] * A[7] - A[4] * A[6];
  int sd3 = A[2] * A[7] - A[1] * A[8];
  int sd4 = A[0] * A[8] - A[2] * A[6];
  int sd5 = A[1] * A[6] - A[0] * A[7];
  int sd6 = A[1] * A[5] - A[2] * A[4];
  int sd7 = A[2] * A[3] - A[0] * A[5];
  int sd8 = A[0] * A[4] - A[1] * A[3];
  int d = A[0] * sd0 + A[1] * sd1 + A[2] * sd2;
  is_invertible = (d != 0) ? true : false;
  double d_rec = is_invertible ? i_rec(d) : 1.0;
  Ai[0] = d_rec * sd0;
  Ai[1] = d_rec * sd3;
  Ai[2] = d_rec * sd6;
  Ai[3] = d_rec * sd1;
  Ai[4] = d_rec * sd4;
  Ai[5] = d_rec * sd7;
  Ai[6] = d_rec * sd2;
  Ai[7] = d_rec * sd5;
  Ai[8] = d_rec * sd8;
}

inline void i_invert_3x3_upper_triangular(const double A[9], double Ai[9]) {
  double A4A8 = A[4] * A[8];
  double A0rec = i_rec(A[0]);
  Ai[0] = A0rec;
  Ai[1] = -i_div(A[1], A[0] * A[4]);
  Ai[2] = (i_div(A[1] * A[5], A4A8) - i_div(A[2], A[8])) * A0rec;
  Ai[3] = 0;
  Ai[4] = i_rec(A[4]);
  Ai[5] = -i_div(A[5], A4A8);
  Ai[6] = 0;
  Ai[7] = 0;
  Ai[8] = i_rec(A[8]);
}
inline void i_invert_3x3_upper_triangular(const float A[9], float Ai[9]) {
  float A4A8 = A[4] * A[8];
  float A0rec = i_rec(A[0]);
  Ai[0] = A0rec;
  Ai[1] = -i_div(A[1], A[0] * A[4]);
  Ai[2] = (i_div(A[1] * A[5], A4A8) - i_div(A[2], A[8])) * A0rec;
  Ai[3] = 0;
  Ai[4] = i_rec(A[4]);
  Ai[5] = -i_div(A[5], A4A8);
  Ai[6] = 0;
  Ai[7] = 0;
  Ai[8] = i_rec(A[8]);
}
inline void i_invert_3x3_upper_triangular(const int A[9], double Ai[9]) {
  double A4A8 = (double)(A[4] * A[8]);
  double A0rec = i_rec(A[0]);
  Ai[0] = A0rec;
  Ai[1] = -i_div((double)A[1], (double)(A[0] * A[4]));
  Ai[2] = (i_div((double)(A[1] * A[5]), A4A8) - i_div(A[2], A[8])) * A0rec;
  Ai[3] = 0;
  Ai[4] = i_rec(A[4]);
  Ai[5] = -i_div((double)A[5], A4A8);
  Ai[6] = 0;
  Ai[7] = 0;
  Ai[8] = i_rec(A[8]);
}

/*Solve 2x2 linear equation system Ax=b using the Cramer's rule*/
inline void i_solve_2x2(const double A[4], const double b[2], double x[2]) {
  double d, rec;
  d = i_determinant_2x2(A);
  rec = i_rec(d);
  x[0] = rec * (A[3] * b[0] - A[1] * b[1]);
  x[1] = rec * (A[0] * b[1] - A[2] * b[0]);
}
inline void i_solve_2x2(const float A[4], const float b[2], float x[2]) {
  float d, rec;
  d = i_determinant_2x2(A);
  rec = i_rec(d);
  x[0] = rec * (A[3] * b[0] - A[1] * b[1]);
  x[1] = rec * (A[0] * b[1] - A[2] * b[0]);
}
/*Solve 3x3 linear equation system Ax=b using the Cramer's rule*/
inline void i_solve_3x3(const double A[9], const double b[3], double x[3]) {
  double d, rec, da0, da1, da2;
  d = i_determinant_3x3(A);
  rec = i_rec(d);
  da0 = b[0] * (A[4] * A[8] - A[5] * A[7]) +
        b[1] * (A[2] * A[7] - A[1] * A[8]) + b[2] * (A[1] * A[5] - A[2] * A[4]);
  da1 = b[0] * (A[5] * A[6] - A[3] * A[8]) +
        b[1] * (A[0] * A[8] - A[2] * A[6]) + b[2] * (A[2] * A[3] - A[0] * A[5]);
  da2 = b[0] * (A[3] * A[7] - A[4] * A[6]) +
        b[1] * (A[1] * A[6] - A[0] * A[7]) + b[2] * (A[0] * A[4] - A[1] * A[3]);
  x[0] = da0 * rec;
  x[1] = da1 * rec;
  x[2] = da2 * rec;
}
inline void i_solve_3x3(const float A[9], const float b[3], float x[3]) {
  float d, rec, da0, da1, da2;
  d = i_determinant_3x3(A);
  rec = i_rec(d);
  da0 = b[0] * (A[4] * A[8] - A[5] * A[7]) +
        b[1] * (A[2] * A[7] - A[1] * A[8]) + b[2] * (A[1] * A[5] - A[2] * A[4]);
  da1 = b[0] * (A[5] * A[6] - A[3] * A[8]) +
        b[1] * (A[0] * A[8] - A[2] * A[6]) + b[2] * (A[2] * A[3] - A[0] * A[5]);
  da2 = b[0] * (A[3] * A[7] - A[4] * A[6]) +
        b[1] * (A[1] * A[6] - A[0] * A[7]) + b[2] * (A[0] * A[4] - A[1] * A[3]);
  x[0] = da0 * rec;
  x[1] = da1 * rec;
  x[2] = da2 * rec;
}

/*Compute the transpose of a suqare nxn matrix inplace*/
template <typename T>
inline void i_transpose(T *A, int n) {
  for (int r = 0; r < n; ++r) {
    for (int c = r; c < n; ++c) {
      i_swap(A[r * n + c], A[c * n + r]);
    }
  }
}
/*Compute the transpose of a mxn matrix A, At is nxm*/
template <typename T>
inline void i_transpose(const T *A, T *At, int m, int n) {
  for (int r = 0; r < m; ++r) {
    for (int c = 0; c < n; ++c) {
      At[c * m + r] = A[r * n + c];
    }
  }
}
template <typename T>
inline void i_transpose_2x2(T A[4]) {
  i_swap(A[1], A[2]);
}
template <typename T>
inline void i_transpose_2x2(const T A[4], T At[4]) {
  At[0] = A[0];
  At[1] = A[2];
  At[2] = A[1];
  At[3] = A[3];
}
template <typename T>
inline void i_transpose_3x3(T A[9]) {
  i_swap(A[1], A[3]);
  i_swap(A[2], A[6]);
  i_swap(A[5], A[7]);
}
template <typename T>
inline void i_transpose_3x3(const T A[9], T At[9]) {
  At[0] = A[0];
  At[1] = A[3];
  At[2] = A[6];
  At[3] = A[1];
  At[4] = A[4];
  At[5] = A[7];
  At[6] = A[2];
  At[7] = A[5];
  At[8] = A[8];
}
template <typename T>
inline void i_transpose_4x4(T A[16]) {
  i_swap(A[1], A[4]);
  i_swap(A[2], A[8]);
  i_swap(A[6], A[9]);
  i_swap(A[3], A[12]);
  i_swap(A[7], A[13]);
  i_swap(A[11], A[14]);
}
template <typename T>
inline void i_transpose_4x4(const T A[16], T At[16]) {
  At[0] = A[0];
  At[1] = A[4];
  At[2] = A[8];
  At[3] = A[12];
  At[4] = A[1];
  At[5] = A[5];
  At[6] = A[9];
  At[7] = A[13];
  At[8] = A[2];
  At[9] = A[6];
  At[10] = A[10];
  At[11] = A[14];
  At[12] = A[3];
  At[13] = A[7];
  At[14] = A[11];
  At[15] = A[15];
}

/*Add a constant lambda (inplace) to the diagonal elements of a nxn square
 * matrix A*/
template <typename T>
inline void i_augment_diagonal(T *A, int n, const T lambda) {
  T *Ai;
  int i, ni = 0;
  for (i = 0; i < n; ++i, ni += n) {
    Ai = A + ni;
    Ai[i] += lambda;
  }
}

/*Multiply m x n matrix A with n-dimensional vector x*/
template <typename T>
inline void i_mult_Ax(const T *A, const T *x, T *Ax, int m, int n) {
  for (int i = 0; i < m; i++) {
    Ax[i] = i_dot(A + i * n, x, n);
  }
}

/*Multiply m x n matrix A's transpose (n x m matrix At) with m-dimensional
 * vector x*/
template <typename T>
inline void i_mult_Atx(const T *A, const T *x, T *Atx, int m, int n) {
  T acc;
  const T *Ai = NULL;
  for (int i = 0; i < n; i++) {
    Ai = A + i;
    acc = T(0.0);
    for (int j = 0; j < m; j++, Ai += n) {
      acc += (*Ai) * x[j];
    }
    Atx[i] = acc;
  }
}

/*Multiply 1 x 3 matrix A with 3-dimensional vector x*/
template <typename T>
inline void i_mult_Ax_1x3(const T A[3], const T x[3], T Ax[1]) {
  Ax[0] = A[0] * x[0] + A[1] * x[1] + A[2] * x[2];
}
/*Multiply 2 x 2 matrix A with 2-dimensional vector x*/
template <typename T>
inline void i_mult_Ax_2x2(const T A[4], const T x[2], T Ax[2]) {
  T x0, x1;
  x0 = x[0];
  x1 = x[1];
  Ax[0] = A[0] * x0 + A[1] * x1;
  Ax[1] = A[2] * x0 + A[3] * x1;
}
/*Multiply 2 x 3 matrix A with 3-dimensional vector x*/
template <typename T>
inline void i_mult_Ax_2x3(const T A[6], const T x[3], T Ax[2]) {
  T x0, x1, x2;
  x0 = x[0];
  x1 = x[1];
  x2 = x[2];
  Ax[0] = A[0] * x0 + A[1] * x1 + A[2] * x2;
  Ax[1] = A[3] * x0 + A[4] * x1 + A[5] * x2;
}
/*Multiply 3 x 3 matrix A with 3-dimensional vector x*/
template <typename T>
inline void i_mult_Ax_3x3(const T A[9], const T x[3], T Ax[3]) {
  T x0, x1, x2;
  x0 = x[0];
  x1 = x[1];
  x2 = x[2];
  Ax[0] = A[0] * x0 + A[1] * x1 + A[2] * x2;
  Ax[1] = A[3] * x0 + A[4] * x1 + A[5] * x2;
  Ax[2] = A[6] * x0 + A[7] * x1 + A[8] * x2;
}
/*Multiply 3 x 3 matrix A^t with 3-dimensional vector x*/
template <typename T>
inline void i_mult_Atx_3x3(const T A[9], const T x[3], T Atx[3]) {
  T x0, x1, x2;
  x0 = x[0];
  x1 = x[1];
  x2 = x[2];
  Atx[0] = A[0] * x0 + A[3] * x1 + A[6] * x2;
  Atx[1] = A[1] * x0 + A[4] * x1 + A[7] * x2;
  Atx[2] = A[2] * x0 + A[5] * x1 + A[8] * x2;
}
/*Multiply 3 x 4 matrix A with 4-dimensional vector x*/
template <typename T>
inline void i_mult_Ax_3x4(const T A[12], const T x[4], T Ax[3]) {
  T x0, x1, x2, x3;
  x0 = x[0];
  x1 = x[1];
  x2 = x[2];
  x3 = x[3];
  Ax[0] = A[0] * x0 + A[1] * x1 + A[2] * x2 + A[3] * x3;
  Ax[1] = A[4] * x0 + A[5] * x1 + A[6] * x2 + A[7] * x3;
  Ax[2] = A[8] * x0 + A[9] * x1 + A[10] * x2 + A[11] * x3;
}
/*Multiply 4 x 3 matrix A with 3-dimensional vector x*/
template <typename T>
inline void i_mult_Ax_4x3(const T A[12], const T x[3], T Ax[4]) {
  T x0, x1, x2;
  x0 = x[0];
  x1 = x[1];
  x2 = x[2];
  Ax[0] = A[0] * x0 + A[1] * x1 + A[2] * x2;
  Ax[1] = A[3] * x0 + A[4] * x1 + A[5] * x2;
  Ax[2] = A[6] * x0 + A[7] * x1 + A[8] * x2;
  Ax[3] = A[9] * x0 + A[10] * x1 + A[11] * x2;
}
/*Multiply 4 x 3 matrix A^t with 3-dimensional vector x*/
template <typename T>
inline void i_mult_Atx_4x3(const T A[12], const T x[3], T Atx[4]) {
  Atx[0] = A[0] * x[0] + A[4] * x[1] + A[8] * x[2];
  Atx[1] = A[1] * x[0] + A[5] * x[1] + A[9] * x[2];
  Atx[2] = A[2] * x[0] + A[6] * x[1] + A[10] * x[2];
  Atx[3] = A[3] * x[0] + A[7] * x[1] + A[11] * x[2];
}
/*Multiply 4 x 4 matrix A with 4-dimensional vector x*/
template <typename T>
inline void i_mult_Ax_4x4(const T A[16], const T x[4], T Ax[4]) {
  T x0, x1, x2, x3;
  x0 = x[0];
  x1 = x[1];
  x2 = x[2];
  x3 = x[3];
  Ax[0] = A[0] * x0 + A[1] * x1 + A[2] * x2 + A[3] * x3;
  Ax[1] = A[4] * x0 + A[5] * x1 + A[6] * x2 + A[7] * x3;
  Ax[2] = A[8] * x0 + A[9] * x1 + A[10] * x2 + A[11] * x3;
  Ax[3] = A[12] * x0 + A[13] * x1 + A[14] * x2 + A[15] * x3;
}
/*Multiply m x n matrix A with n x o matrix B*/
template <typename T>
inline void i_mult_AB(const T *A, const T *B, T *AB, int m, int n, int o) {
  int in, io;
  T acc;
  for (int i = 0; i < m; i++) {
    in = i * n;
    io = i * o;
    for (int j = 0; j < o; j++) {
      acc = (T)(0.0);
      for (int k = 0; k < n; k++) {
        acc += A[in + k] * B[k * o + j];
      }
      AB[io + j] = acc;
    }
  }
}

/*Multiply 3 x 1 matrix A with 1 x 3 matrix B*/
template <typename T>
inline void i_mult_AB_3x1_1x3(const T A[3], const T B[3], T AB[9]) {
  AB[0] = A[0] * B[0];
  AB[1] = A[0] * B[1];
  AB[2] = A[0] * B[2];
  AB[3] = A[1] * B[0];
  AB[4] = A[1] * B[1];
  AB[5] = A[1] * B[2];
  AB[6] = A[2] * B[0];
  AB[7] = A[2] * B[1];
  AB[8] = A[2] * B[2];
}

/*Multiply 2 x 2 matrix A with 2 x 2 matrix B*/
template <typename T>
inline void i_mult_AB_2x2_2x2(const T A[4], const T B[4], T AB[4]) {
  T a, b;
  a = A[0];
  b = A[1];
  AB[0] = a * B[0] + b * B[2];
  AB[1] = a * B[1] + b * B[3];
  a = A[2];
  b = A[3];
  AB[2] = a * B[0] + b * B[2];
  AB[3] = a * B[1] + b * B[3];
}

/*Multiply 2 x 3 matrix A with 3 x 2 matrix B*/
template <typename T>
inline void i_mult_AB_2x3_3x2(const T A[6], const T B[6], T AB[4]) {
  T a, b, c;
  a = A[0];
  b = A[1];
  c = A[2];
  AB[0] = a * B[0] + b * B[2] + c * B[4];
  AB[1] = a * B[1] + b * B[3] + c * B[5];
  a = A[3];
  b = A[4];
  c = A[5];
  AB[2] = a * B[0] + b * B[2] + c * B[4];
  AB[3] = a * B[1] + b * B[3] + c * B[5];
}

/*Multiply 2 x 3 matrix A with 3 x 3 matrix B*/
template <typename T>
inline void i_mult_AB_2x3_3x3(const T A[6], const T B[9], T AB[6]) {
  T a, b, c;
  a = A[0];
  b = A[1];
  c = A[2];
  AB[0] = a * B[0] + b * B[3] + c * B[6];
  AB[1] = a * B[1] + b * B[4] + c * B[7];
  AB[2] = a * B[2] + b * B[5] + c * B[8];

  a = A[3];
  b = A[4];
  c = A[5];
  AB[3] = a * B[0] + b * B[3] + c * B[6];
  AB[4] = a * B[1] + b * B[4] + c * B[7];
  AB[5] = a * B[2] + b * B[5] + c * B[8];
}

/*Multiply 2 x 3 matrix A with 3 x 4 matrix B*/
template <typename T>
inline void i_mult_AB_2x3_3x4(const T A[6], const T B[12], T AB[8]) {
  T a, b, c;
  a = A[0];
  b = A[1];
  c = A[2];
  AB[0] = a * B[0] + b * B[4] + c * B[8];
  AB[1] = a * B[1] + b * B[5] + c * B[9];
  AB[2] = a * B[2] + b * B[6] + c * B[10];
  AB[3] = a * B[3] + b * B[7] + c * B[11];
  a = A[3];
  b = A[4];
  c = A[5];
  AB[4] = a * B[0] + b * B[4] + c * B[8];
  AB[5] = a * B[1] + b * B[5] + c * B[9];
  AB[6] = a * B[2] + b * B[6] + c * B[10];
  AB[7] = a * B[3] + b * B[7] + c * B[11];
}

/*Multiply 3 x 3 matrix A with 3 x 3 matrix B*/
template <typename T>
inline void i_mult_AB_3x3_3x3(const T A[9], const T B[9], T AB[9]) {
  T a, b, c;
  a = A[0];
  b = A[1];
  c = A[2];
  AB[0] = a * B[0] + b * B[3] + c * B[6];
  AB[1] = a * B[1] + b * B[4] + c * B[7];
  AB[2] = a * B[2] + b * B[5] + c * B[8];
  a = A[3];
  b = A[4];
  c = A[5];
  AB[3] = a * B[0] + b * B[3] + c * B[6];
  AB[4] = a * B[1] + b * B[4] + c * B[7];
  AB[5] = a * B[2] + b * B[5] + c * B[8];
  a = A[6];
  b = A[7];
  c = A[8];
  AB[6] = a * B[0] + b * B[3] + c * B[6];
  AB[7] = a * B[1] + b * B[4] + c * B[7];
  AB[8] = a * B[2] + b * B[5] + c * B[8];
}

/*Multiply 4 x 4 matrix A with 4 x 4 matrix B*/
template <typename T>
inline void i_mult_AB_4x4_4x4(const T A[16], const T B[16], T AB[16]) {
  T a, b, c, d;
  a = A[0];
  b = A[1];
  c = A[2];
  d = A[3];
  AB[0] = a * B[0] + b * B[4] + c * B[8] + d * B[12];
  AB[1] = a * B[1] + b * B[5] + c * B[9] + d * B[13];
  AB[2] = a * B[2] + b * B[6] + c * B[10] + d * B[14];
  AB[3] = a * B[3] + b * B[7] + c * B[11] + d * B[15];

  a = A[4];
  b = A[5];
  c = A[6];
  d = A[7];
  AB[4] = a * B[0] + b * B[4] + c * B[8] + d * B[12];
  AB[5] = a * B[1] + b * B[5] + c * B[9] + d * B[13];
  AB[6] = a * B[2] + b * B[6] + c * B[10] + d * B[14];
  AB[7] = a * B[3] + b * B[7] + c * B[11] + d * B[15];

  a = A[8];
  b = A[9];
  c = A[10];
  d = A[11];
  AB[8] = a * B[0] + b * B[4] + c * B[8] + d * B[12];
  AB[9] = a * B[1] + b * B[5] + c * B[9] + d * B[13];
  AB[10] = a * B[2] + b * B[6] + c * B[10] + d * B[14];
  AB[11] = a * B[3] + b * B[7] + c * B[11] + d * B[15];

  a = A[12];
  b = A[13];
  c = A[14];
  d = A[15];
  AB[12] = a * B[0] + b * B[4] + c * B[8] + d * B[12];
  AB[13] = a * B[1] + b * B[5] + c * B[9] + d * B[13];
  AB[14] = a * B[2] + b * B[6] + c * B[10] + d * B[14];
  AB[15] = a * B[3] + b * B[7] + c * B[11] + d * B[15];
}

/*Multiply 4 x 1 matrix A with 1 x 4 matrix B*/
template <typename T>
inline void i_mult_AB_4x1_1x4(const T A[4], const T B[4], T AB[16]) {
  T sf = A[0];
  AB[0] = sf * B[0];
  AB[1] = sf * B[1];
  AB[2] = sf * B[2];
  AB[3] = sf * B[3];

  sf = A[1];
  AB[4] = sf * B[0];
  AB[5] = sf * B[1];
  AB[6] = sf * B[2];
  AB[7] = sf * B[3];

  sf = A[2];
  AB[8] = sf * B[0];
  AB[9] = sf * B[1];
  AB[10] = sf * B[2];
  AB[11] = sf * B[3];

  sf = A[3];
  AB[12] = sf * B[0];
  AB[13] = sf * B[1];
  AB[14] = sf * B[2];
  AB[15] = sf * B[3];
}

/*Multiply upper-triangular 3 x 3 matrix A with 3 x 3 matrix B*/
template <typename T>
inline void i_mult_AB_3x3_3x3_w_A_upper_triangular(const T A[9], const T B[9],
                                                   T AB[9]) {
  T a, b, c;
  a = A[0];
  b = A[1];
  c = A[2];
  AB[0] = a * B[0] + b * B[3] + c * B[6];
  AB[1] = a * B[1] + b * B[4] + c * B[7];
  AB[2] = a * B[2] + b * B[5] + c * B[8];
  b = A[4];
  c = A[5];
  AB[3] = b * B[3] + c * B[6];
  AB[4] = b * B[4] + c * B[7];
  AB[5] = b * B[5] + c * B[8];
  c = A[8];
  AB[6] = c * B[6];
  AB[7] = c * B[7];
  AB[8] = c * B[8];
}

/*Multiply 3 x 3 matrix A with upper-triangular 3 x 3 matrix B*/
template <typename T>
inline void i_mult_AB_3x3_3x3_w_B_upper_triangular(const T A[9], const T B[9],
                                                   T AB[9]) {
  T a, b, c;
  a = A[0];
  b = A[1];
  c = A[2];
  AB[0] = a * B[0];
  AB[1] = a * B[1] + b * B[4];
  AB[2] = a * B[2] + b * B[5] + c * B[8];
  a = A[3];
  b = A[4];
  c = A[5];
  AB[3] = a * B[0];
  AB[4] = a * B[1] + b * B[4];
  AB[5] = a * B[2] + b * B[5] + c * B[8];
  a = A[6];
  b = A[7];
  c = A[8];
  AB[6] = a * B[0];
  AB[7] = a * B[1] + b * B[4];
  AB[8] = a * B[2] + b * B[5] + c * B[8];
}

template <typename T>
inline void i_mult_AB_3x3_3x4(const T A[9], const T B[12], T AB[12]) {
  AB[0] = A[0] * B[0] + A[1] * B[4] + A[2] * B[8];
  AB[1] = A[0] * B[1] + A[1] * B[5] + A[2] * B[9];
  AB[2] = A[0] * B[2] + A[1] * B[6] + A[2] * B[10];
  AB[3] = A[0] * B[3] + A[1] * B[7] + A[2] * B[11];

  AB[4] = A[3] * B[0] + A[4] * B[4] + A[5] * B[8];
  AB[5] = A[3] * B[1] + A[4] * B[5] + A[5] * B[9];
  AB[6] = A[3] * B[2] + A[4] * B[6] + A[5] * B[10];
  AB[7] = A[3] * B[3] + A[4] * B[7] + A[5] * B[11];

  AB[8] = A[6] * B[0] + A[7] * B[4] + A[8] * B[8];
  AB[9] = A[6] * B[1] + A[7] * B[5] + A[8] * B[9];
  AB[10] = A[6] * B[2] + A[7] * B[6] + A[8] * B[10];
  AB[11] = A[6] * B[3] + A[7] * B[7] + A[8] * B[11];
}

template <typename T>
inline void i_mult_AB_3x4_4x3(const T A[12], const T B[12], T AB[9]) {
  T a, b, c, d;
  a = A[0];
  b = A[1];
  c = A[2];
  d = A[3];
  AB[0] = a * B[0] + b * B[3] + c * B[6] + d * B[9];
  AB[1] = a * B[1] + b * B[4] + c * B[7] + d * B[10];
  AB[2] = a * B[2] + b * B[5] + c * B[8] + d * B[11];

  a = A[4];
  b = A[5];
  c = A[6];
  d = A[7];
  AB[3] = a * B[0] + b * B[3] + c * B[6] + d * B[9];
  AB[4] = a * B[1] + b * B[4] + c * B[7] + d * B[10];
  AB[5] = a * B[2] + b * B[5] + c * B[8] + d * B[11];

  a = A[8];
  b = A[9];
  c = A[10];
  d = A[11];
  AB[6] = a * B[0] + b * B[3] + c * B[6] + d * B[9];
  AB[7] = a * B[1] + b * B[4] + c * B[7] + d * B[10];
  AB[8] = a * B[2] + b * B[5] + c * B[8] + d * B[11];
}

/*Multiply 3 x 3 matrix A with 3 x 3 matrix B^t*/
template <typename T>
inline void i_mult_ABt_3x3_3x3(const T A[9], const T B[9], T ABt[9]) {
  T a, b, c;
  a = A[0];
  b = A[1];
  c = A[2];
  ABt[0] = a * B[0] + b * B[1] + c * B[2];
  ABt[1] = a * B[3] + b * B[4] + c * B[5];
  ABt[2] = a * B[6] + b * B[7] + c * B[8];
  a = A[3];
  b = A[4];
  c = A[5];
  ABt[3] = a * B[0] + b * B[1] + c * B[2];
  ABt[4] = a * B[3] + b * B[4] + c * B[5];
  ABt[5] = a * B[6] + b * B[7] + c * B[8];
  a = A[6];
  b = A[7];
  c = A[8];
  ABt[6] = a * B[0] + b * B[1] + c * B[2];
  ABt[7] = a * B[3] + b * B[4] + c * B[5];
  ABt[8] = a * B[6] + b * B[7] + c * B[8];
}

/*Multiply 2 x 3 matrix A with 3 x 2 matrix B^t*/
template <typename T>
inline void i_mult_ABt_2x3_2x3(const T A[6], const T B[6], T ABt[4]) {
  ABt[0] = A[0] * B[0] + A[1] * B[1] + A[2] * B[2];
  ABt[1] = A[0] * B[3] + A[1] * B[4] + A[2] * B[5];
  ABt[2] = A[3] * B[0] + A[4] * B[1] + A[5] * B[2];
  ABt[3] = A[3] * B[3] + A[4] * B[4] + A[5] * B[5];
}

/*Multiply 4 x 4 matrix A with 4 x 3 matrix B^t*/
template <typename T>
inline void i_mult_ABt_4x4_3x4(const T A[16], const T B[12], T ABt[12]) {
  ABt[0] = i_dot4(A, B);
  ABt[1] = i_dot4(A, B + 4);
  ABt[2] = i_dot4(A, B + 8);

  ABt[3] = i_dot4(A + 4, B);
  ABt[4] = i_dot4(A + 4, B + 4);
  ABt[5] = i_dot4(A + 4, B + 8);

  ABt[6] = i_dot4(A + 8, B);
  ABt[7] = i_dot4(A + 8, B + 4);
  ABt[8] = i_dot4(A + 8, B + 8);

  ABt[9] = i_dot4(A + 12, B);
  ABt[10] = i_dot4(A + 12, B + 4);
  ABt[11] = i_dot4(A + 12, B + 8);
}

/*Multiply m x n matrix A's transpose At (n x m) with A and get AtA (n x n)*/
template <typename T>
inline void i_mult_AtA(const T *A, T *AtA, int m, int n) {
  int i, j, k, ni;
  T acc;
  const T *Ai, *Aj;
  for (i = 0; i < n; ++i) {
    ni = n * i;
    for (j = 0; j < i; ++j) {
      AtA[ni + j] = AtA[j * n + i];
    }
    for (j = i; j < n; ++j) {
      acc = (T)0.0;
      Ai = A + i;
      Aj = A + j;
      for (k = 0; k < m; ++k, Ai += n, Aj += n) {
        acc += (*Ai) * (*Aj);
      }
      AtA[ni + j] = acc;
    }
  }
}

template <typename T>
inline void i_mult_AtA_2x2(const T A[4], T AtA[4]) {
  AtA[0] = A[0] * A[0] + A[2] * A[2];
  AtA[1] = AtA[2] = A[0] * A[1] + A[2] * A[3];
  AtA[3] = A[1] * A[1] + A[3] * A[3];
}

template <typename T>
inline void i_mult_AtA_nx2(const T *A, T AtA[4], int n) {
  T xx = (T)0.0;
  T xy = (T)0.0;
  T yy = (T)0.0;
  T x, y;
  for (int i = 0; i < 2 * n; i += 2) {
    x = A[i];
    y = A[i + 1];
    xx += x * x;
    xy += x * y;
    yy += y * y;
  }
  AtA[0] = xx;
  AtA[1] = AtA[2] = xy;
  AtA[3] = yy;
}

template <typename T>
inline void i_mult_AtA_3x3(const T A[9], T AtA[9]) {
  AtA[0] = A[0] * A[0] + A[3] * A[3] + A[6] * A[6];
  AtA[1] = AtA[3] = A[0] * A[1] + A[3] * A[4] + A[6] * A[7];
  AtA[2] = AtA[6] = A[0] * A[2] + A[3] * A[5] + A[6] * A[8];
  AtA[4] = A[1] * A[1] + A[4] * A[4] + A[7] * A[7];
  AtA[5] = AtA[7] = A[1] * A[2] + A[4] * A[5] + A[7] * A[8];
  AtA[8] = A[2] * A[2] + A[5] * A[5] + A[8] * A[8];
}

template <typename T>
inline void i_mult_AtA_nx3(const T *A, T AtA[9], int n) {
  T xx = (T)0.0;
  T xy = (T)0.0;
  T xz = (T)0.0;
  T yy = (T)0.0;
  T yz = (T)0.0;
  T zz = (T)0.0;
  T x, y, z;
  int i, j;
  for (i = 0, j = 0; i < n; i++) {
    x = A[j++];
    y = A[j++];
    z = A[j++];
    xx += x * x;
    xy += x * y;
    xz += x * z;
    yy += y * y;
    yz += y * z;
    zz += z * z;
  }
  AtA[0] = xx;
  AtA[1] = AtA[3] = xy;
  AtA[2] = AtA[6] = xz;
  AtA[4] = yy;
  AtA[5] = AtA[7] = yz;
  AtA[8] = zz;
}

template <typename T>
inline void i_mult_AtA_4x4(const T A[16], T AtA[16]) {
  AtA[0] = A[0] * A[0] + A[4] * A[4] + A[8] * A[8] + A[12] * A[12];
  AtA[1] = AtA[4] = A[0] * A[1] + A[4] * A[5] + A[8] * A[9] + A[12] * A[13];
  AtA[2] = AtA[8] = A[0] * A[2] + A[4] * A[6] + A[8] * A[10] + A[12] * A[14];
  AtA[3] = AtA[12] = A[0] * A[3] + A[4] * A[7] + A[8] * A[11] + A[12] * A[15];
  AtA[5] = A[1] * A[1] + A[5] * A[5] + A[9] * A[9] + A[13] * A[13];
  AtA[6] = AtA[9] = A[1] * A[2] + A[5] * A[6] + A[9] * A[10] + A[13] * A[14];
  AtA[7] = AtA[13] = A[1] * A[3] + A[5] * A[7] + A[9] * A[11] + A[13] * A[15];
  AtA[10] = A[2] * A[2] + A[6] * A[6] + A[10] * A[10] + A[14] * A[14];
  AtA[11] = AtA[14] = A[2] * A[3] + A[6] * A[7] + A[10] * A[11] + A[14] * A[15];
  AtA[15] = A[3] * A[3] + A[7] * A[7] + A[11] * A[11] + A[15] * A[15];
}

template <typename T>
inline void i_mult_AtB_2x2_2x2(const T A[4], const T B[4], T AtB[4]) {
  AtB[0] = A[0] * B[0] + A[2] * B[2];
  AtB[1] = A[0] * B[1] + A[2] * B[3];
  AtB[2] = A[1] * B[0] + A[3] * B[2];
  AtB[3] = A[1] * B[1] + A[3] * B[3];
}

template <typename T>
inline void i_mult_AtB_3x3_3x3(const T A[9], const T B[9], T AtB[9]) {
  AtB[0] = A[0] * B[0] + A[3] * B[3] + A[6] * B[6];
  AtB[1] = A[0] * B[1] + A[3] * B[4] + A[6] * B[7];
  AtB[2] = A[0] * B[2] + A[3] * B[5] + A[6] * B[8];
  AtB[3] = A[1] * B[0] + A[4] * B[3] + A[7] * B[6];
  AtB[4] = A[1] * B[1] + A[4] * B[4] + A[7] * B[7];
  AtB[5] = A[1] * B[2] + A[4] * B[5] + A[7] * B[8];
  AtB[6] = A[2] * B[0] + A[5] * B[3] + A[8] * B[6];
  AtB[7] = A[2] * B[1] + A[5] * B[4] + A[8] * B[7];
  AtB[8] = A[2] * B[2] + A[5] * B[5] + A[8] * B[8];
}

template <typename T>
inline void i_mult_AAt_2x3(const T A[6], T AAt[4]) {
  AAt[0] = A[0] * A[0] + A[1] * A[1] + A[2] * A[2];
  AAt[1] = AAt[2] = A[0] * A[3] + A[1] * A[4] + A[2] * A[5];
  AAt[3] = A[3] * A[3] + A[4] * A[4] + A[5] * A[5];
}

template <typename T>
inline void i_mult_AAt_3x3(const T A[9], T AAt[9]) {
  AAt[0] = (A[0] * A[0] + A[1] * A[1] + A[2] * A[2]);
  AAt[1] = AAt[3] = (A[0] * A[3] + A[1] * A[4] + A[2] * A[5]);
  AAt[2] = AAt[6] = (A[0] * A[6] + A[1] * A[7] + A[2] * A[8]);
  AAt[4] = (A[3] * A[3] + A[4] * A[4] + A[5] * A[5]);
  AAt[5] = AAt[7] = (A[3] * A[6] + A[4] * A[7] + A[5] * A[8]);
  AAt[8] = (A[6] * A[6] + A[7] * A[7] + A[8] * A[8]);
}

template <typename T>
inline void i_mult_AAt_4x1(const T A[4], T AAt[16]) {
  AAt[0] = A[0] * A[0];
  AAt[1] = AAt[4] = (A[0] * A[1]);
  AAt[2] = AAt[8] = (A[0] * A[2]);
  AAt[3] = AAt[12] = (A[0] * A[3]);

  AAt[5] = A[1] * A[1];
  AAt[6] = AAt[9] = (A[1] * A[2]);
  AAt[7] = AAt[13] = (A[1] * A[3]);

  AAt[10] = A[2] * A[2];
  AAt[11] = AAt[14] = (A[2] * A[3]);

  AAt[15] = A[3] * A[3];
}

template <typename T>
inline void i_mult_ABC_3x3_3x3_3x3(const T A[9], const T B[9], const T C[9],
                                   T ABC[9]) {
  T BC[9];
  i_mult_AB_3x3_3x3(B, C, BC);
  i_mult_AB_3x3_3x3(A, BC, ABC);
}

template <typename T>
inline void i_mult_AtBC_3x3_3x3_3x3(const T A[9], const T B[9], const T C[9],
                                    T AtBC[9]) {
  T BC[9];
  i_mult_AB_3x3_3x3(B, C, BC);
  i_mult_AtB_3x3_3x3(A, BC, AtBC);
}

template <typename T>
inline void i_mult_AB_4x4_w_A_block_diagonal(const T A1[4], const T A2[4],
                                             const T B[16], T AB[16]) {
  /*A = A1   0
        0   A2
    B = B1  B2
        B3  B4*/

  /*A1B1*/
  AB[0] = A1[0] * B[0] + A1[1] * B[4];
  AB[1] = A1[0] * B[1] + A1[1] * B[5];
  AB[4] = A1[2] * B[0] + A1[3] * B[4];
  AB[5] = A1[2] * B[1] + A1[3] * B[5];

  /*A1B2*/
  AB[2] = A1[0] * B[2] + A1[1] * B[6];
  AB[3] = A1[0] * B[3] + A1[1] * B[7];
  AB[6] = A1[2] * B[2] + A1[3] * B[6];
  AB[7] = A1[2] * B[3] + A1[3] * B[7];

  /*A2B3*/
  AB[8] = A2[0] * B[8] + A2[1] * B[12];
  AB[9] = A2[0] * B[9] + A2[1] * B[13];
  AB[12] = A2[2] * B[8] + A2[3] * B[12];
  AB[13] = A2[2] * B[9] + A2[3] * B[13];

  /*A2B4*/
  AB[10] = A2[0] * B[10] + A2[1] * B[14];
  AB[11] = A2[0] * B[11] + A2[1] * B[15];
  AB[14] = A2[2] * B[10] + A2[3] * B[14];
  AB[15] = A2[2] * B[11] + A2[3] * B[15];
}

/*Swap rows r1 and r2 of a mxn matrix A*/
template <typename T>
inline void i_swap_rows(T *A, int r1, int r2, int /*m*/, int n) {
  i_swap(A + r1 * n, A + r2 * n, n);
}
/*Swap columns c1 and c2 of a mxn matrix A*/
template <typename T>
inline void i_swap_cols(T *A, int c1, int c2, int m, int n) {
  T *ref = A;
  for (int i = 0; i < m; i++, ref += n) {
    i_swap(ref[c1], ref[c2]);
  }
}

/*Swap interval [i1,i2) of rows r1 and r2 of a mxn matrix A*/
template <typename T>
inline void i_swap_rows_interval(T *A, int i1, int i2, int r1, int r2,
                                 int /*m*/, int n) {
  int i_begin = i_max(0, i1);
  int i_end = i_min(n, i2);
  i_swap(A + r1 * n + i_begin, A + r2 * n + i_begin, (i_end - i_begin));
}
/*Swap interval [i1,i2) of columns c1 and c2 of a mxn matrix A*/
template <typename T>
inline void i_swap_cols_interval(T *A, int i1, int i2, int c1, int c2, int m,
                                 int n) {
  int i_begin = i_max(0, i1);
  int i_end = i_min(m, i2);
  T *ref = A + i_begin * n;
  for (int i = i_begin; i < i_end; i++, ref += n) {
    i_swap(ref[c1], ref[c2]);
  }
}

/*Shift array elements in place so that {x1, y1, ?, x2, y2, ?, x3, y3, ? ... xn,
yn, ?} becomes {x1, y1, x2, y2, x3, y3, ..., xn, yn, ?, ?, ?, ...?} after
shifting
The size of input array A is n*3*/
template <typename T>
inline void i_shift_homogeneous3(T *A, int n) {
  if (n <= 1) {
    return;
  }
  int i, nm1 = n - 1;
  int dst = 2;
  int src = 3;
  for (i = 0; i < nm1; ++i) {
    i_swap(A[dst], A[src]);
    i_swap(A[dst + 1], A[src + 1]);
    dst += 2;
    src += 3;
  }
}

/*Shift array elements in place so that {x1, y1, z1, ?, x2, y2, z2, ?, x3, y3,
z3, ? ... xn, yn, zn, ?} becomes {x1, y1, z1, x2, y2, z2, x3, y3, z3..., xn, yn,
zn, ?, ?, ?, ...?} after shifting
The size of input array A is n*4*/
template <typename T>
inline void i_shift_homogeneous4(T *A, int n) {
  if (n <= 1) {
    return;
  }
  int i, nm1 = n - 1;
  int dst = 3;
  int src = 4;
  for (i = 0; i < nm1; ++i) {
    i_swap(A[dst], A[src]);
    i_swap(A[dst + 1], A[src + 1]);
    i_swap(A[dst + 2], A[src + 2]);
    dst += 3;
    src += 4;
  }
}
} /* namespace idl */