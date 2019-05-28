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

// Copy of 1D arrays
template <typename T>
inline void ICopy(const T *src, T *dst, int n) {
  memcpy(dst, src, n * sizeof(T));
}
template <typename T>
inline void ICopy1(const T src[1], T dst[1]) {
  dst[0] = src[0];
}
template <typename T>
inline void ICopy2(const T src[2], T dst[2]) {
  dst[0] = src[0];
  dst[1] = src[1];
}
template <typename T>
inline void ICopy3(const T src[3], T dst[3]) {
  dst[0] = src[0];
  dst[1] = src[1];
  dst[2] = src[2];
}
template <typename T>
inline void ICopy4(const T src[4], T dst[4]) {
  dst[0] = src[0];
  dst[1] = src[1];
  dst[2] = src[2];
  dst[3] = src[3];
}
template <typename T>
inline void ICopy5(const T src[5], T dst[5]) {
  dst[0] = src[0];
  dst[1] = src[1];
  dst[2] = src[2];
  dst[3] = src[3];
  dst[4] = src[4];
}
template <typename T>
inline void ICopy6(const T src[6], T dst[6]) {
  dst[0] = src[0];
  dst[1] = src[1];
  dst[2] = src[2];
  dst[3] = src[3];
  dst[4] = src[4];
  dst[5] = src[5];
}
template <typename T>
inline void ICopy7(const T src[7], T dst[7]) {
  dst[0] = src[0];
  dst[1] = src[1];
  dst[2] = src[2];
  dst[3] = src[3];
  dst[4] = src[4];
  dst[5] = src[5];
  dst[6] = src[6];
}
template <typename T>
inline void ICopy8(const T src[8], T dst[8]) {
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
inline void ICopy9(const T src[9], T dst[9]) {
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
inline void ICopy10(const T src[10], T dst[10]) {
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
inline void ICopy11(const T src[11], T dst[11]) {
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
inline void ICopy12(const T src[12], T dst[12]) {
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
inline void ICopy13(const T src[13], T dst[13]) {
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
inline void ICopy14(const T src[14], T dst[14]) {
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
inline void ICopy15(const T src[15], T dst[15]) {
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
inline void ICopy16(const T src[16], T dst[16]) {
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
// Copy of 2D arrays
template <typename T>
inline void ICopy(const T *const *src, T **dst, int m, int n) {
  int i;
  for (i = 0; i < m; i++) ICopy<T>(src[i], dst[i], n);
}

// Copy of 1D arrays with different types
template <typename T, typename S>
inline void ICopy(const T *src, S *dst, int n) {
  int i;
  for (i = 0; i < n; ++i) {
    dst[i] = (S)(src[i]);
  }
}
// Copy of 2D arrays with different types
template <typename T, typename S>
inline void ICopy(const T *const *src, S **dst, int m, int n) {
  int i;
  for (i = 0; i < m; i++) ICopy<T, S>(src[i], dst[i], n);
}

// Fill array elements with constant value
template <typename T>
inline void IFill(T *a, int n, T val) {
  for (int i = 0; i < n; i++) a[i] = val;
}
template <typename T>
inline void IFill1(T a[1], T val) {
  a[0] = val;
}
template <typename T>
inline void IFill2(T a[2], T val) {
  a[0] = a[1] = val;
}
template <typename T>
inline void IFill3(T a[3], T val) {
  a[0] = a[1] = a[2] = val;
}
template <typename T>
inline void IFill4(T a[4], T val) {
  a[0] = a[1] = a[2] = a[3] = val;
}
template <typename T>
inline void IFill5(T a[5], T val) {
  a[0] = a[1] = a[2] = a[3] = a[4] = val;
}
template <typename T>
inline void IFill6(T a[6], T val) {
  a[0] = a[1] = a[2] = a[3] = a[4] = a[5] = val;
}
template <typename T>
inline void IFill7(T a[7], T val) {
  a[0] = a[1] = a[2] = a[3] = a[4] = a[5] = a[6] = val;
}
template <typename T>
inline void IFill8(T a[8], T val) {
  a[0] = a[1] = a[2] = a[3] = a[4] = a[5] = a[6] = a[7] = val;
}
template <typename T>
inline void IFill9(T a[9], T val) {
  a[0] = a[1] = a[2] = a[3] = a[4] = a[5] = a[6] = a[7] = a[8] = val;
}
template <typename T>
inline void IFill10(T a[10], T val) {
  a[0] = a[1] = a[2] = a[3] = a[4] = a[5] = a[6] = a[7] = a[8] = a[9] = val;
}
template <typename T>
inline void IFill11(T a[11], T val) {
  a[0] = a[1] = a[2] = a[3] = a[4] = a[5] = a[6] = a[7] = a[8] = a[9] = a[10] =
      val;
}
template <typename T>
inline void IFill12(T a[12], T val) {
  a[0] = a[1] = a[2] = a[3] = a[4] = a[5] = a[6] = a[7] = a[8] = a[9] = a[10] =
      a[11] = val;
}
template <typename T>
inline void IFill13(T a[13], T val) {
  a[0] = a[1] = a[2] = a[3] = a[4] = a[5] = a[6] = a[7] = a[8] = a[9] = a[10] =
      a[11] = a[12] = val;
}
template <typename T>
inline void IFill14(T a[14], T val) {
  a[0] = a[1] = a[2] = a[3] = a[4] = a[5] = a[6] = a[7] = a[8] = a[9] = a[10] =
      a[11] = a[12] = a[13] = val;
}
template <typename T>
inline void IFill15(T a[15], T val) {
  a[0] = a[1] = a[2] = a[3] = a[4] = a[5] = a[6] = a[7] = a[8] = a[9] = a[10] =
      a[11] = a[12] = a[13] = a[14] = val;
}
template <typename T>
inline void IFill16(T a[16], T val) {
  a[0] = a[1] = a[2] = a[3] = a[4] = a[5] = a[6] = a[7] = a[8] = a[9] = a[10] =
      a[11] = a[12] = a[13] = a[14] = a[15] = val;
}

// Fill array elements with zeroes
template <typename T>
inline void IZero(T *a, int n) {
  for (int i = 0; i < n; i++) a[i] = static_cast<T>(0.0);
}
template <typename T>
inline void IZero1(T a[1]) {
  a[0] = static_cast<T>(0.0);
}
template <typename T>
inline void IZero2(T a[2]) {
  a[0] = a[1] = static_cast<T>(0.0);
}
template <typename T>
inline void IZero3(T a[3]) {
  a[0] = a[1] = a[2] = static_cast<T>(0.0);
}
template <typename T>
inline void IZero4(T a[4]) {
  a[0] = a[1] = a[2] = a[3] = static_cast<T>(0.0);
}
template <typename T>
inline void IZero5(T a[5]) {
  a[0] = a[1] = a[2] = a[3] = a[4] = static_cast<T>(0.0);
}
template <typename T>
inline void IZero6(T a[6]) {
  a[0] = a[1] = a[2] = a[3] = a[4] = a[5] = static_cast<T>(0.0);
}
template <typename T>
inline void IZero7(T a[7]) {
  a[0] = a[1] = a[2] = a[3] = a[4] = a[5] = a[6] = static_cast<T>(0.0);
}
template <typename T>
inline void IZero8(T a[8]) {
  a[0] = a[1] = a[2] = a[3] = a[4] = a[5] = a[6] = a[7] = static_cast<T>(0.0);
}
template <typename T>
inline void IZero9(T a[9]) {
  a[0] = a[1] = a[2] = a[3] = a[4] = a[5] = a[6] = a[7] = a[8] =
      static_cast<T>(0.0);
}
template <typename T>
inline void IZero10(T a[10]) {
  a[0] = a[1] = a[2] = a[3] = a[4] = a[5] = a[6] = a[7] = a[8] = a[9] =
      static_cast<T>(0.0);
}
template <typename T>
inline void IZero11(T a[11]) {
  a[0] = a[1] = a[2] = a[3] = a[4] = a[5] = a[6] = a[7] = a[8] = a[9] = a[10] =
      static_cast<T>(0.0);
}
template <typename T>
inline void IZero12(T a[12]) {
  a[0] = a[1] = a[2] = a[3] = a[4] = a[5] = a[6] = a[7] = a[8] = a[9] = a[10] =
      a[11] = static_cast<T>(0.0);
}
template <typename T>
inline void IZero13(T a[13]) {
  a[0] = a[1] = a[2] = a[3] = a[4] = a[5] = a[6] = a[7] = a[8] = a[9] = a[10] =
      a[11] = a[12] = static_cast<T>(0.0);
}
template <typename T>
inline void IZero14(T a[14]) {
  a[0] = a[1] = a[2] = a[3] = a[4] = a[5] = a[6] = a[7] = a[8] = a[9] = a[10] =
      a[11] = a[12] = a[13] = static_cast<T>(0.0);
}
template <typename T>
inline void IZero15(T a[15]) {
  a[0] = a[1] = a[2] = a[3] = a[4] = a[5] = a[6] = a[7] = a[8] = a[9] = a[10] =
      a[11] = a[12] = a[13] = a[14] = static_cast<T>(0.0);
}
template <typename T>
inline void IZero16(T a[16]) {
  a[0] = a[1] = a[2] = a[3] = a[4] = a[5] = a[6] = a[7] = a[8] = a[9] = a[10] =
      a[11] = a[12] = a[13] = a[14] = a[15] = static_cast<T>(0.0);
}

// Negate a vector x of length n inplace
template <typename T>
inline void INeg(T *x, int n) {
  for (int i = 0; i < n; i++) {
    x[i] = -x[i];
  }
}
template <typename T>
inline void INeg1(T x[1]) {
  x[0] = -x[0];
}
template <typename T>
inline void INeg2(T x[2]) {
  x[0] = -x[0];
  x[1] = -x[1];
}
template <typename T>
inline void INeg3(T x[3]) {
  x[0] = -x[0];
  x[1] = -x[1];
  x[2] = -x[2];
}
template <typename T>
inline void INeg4(T x[4]) {
  x[0] = -x[0];
  x[1] = -x[1];
  x[2] = -x[2];
  x[3] = -x[3];
}
template <typename T>
inline void INeg5(T x[5]) {
  x[0] = -x[0];
  x[1] = -x[1];
  x[2] = -x[2];
  x[3] = -x[3];
  x[4] = -x[4];
}
template <typename T>
inline void INeg6(T x[6]) {
  x[0] = -x[0];
  x[1] = -x[1];
  x[2] = -x[2];
  x[3] = -x[3];
  x[4] = -x[4];
  x[5] = -x[5];
}
template <typename T>
inline void INeg7(T x[7]) {
  x[0] = -x[0];
  x[1] = -x[1];
  x[2] = -x[2];
  x[3] = -x[3];
  x[4] = -x[4];
  x[5] = -x[5];
  x[6] = -x[6];
}
template <typename T>
inline void INeg8(T x[8]) {
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
inline void INeg9(T x[9]) {
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
inline void INeg10(T x[10]) {
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
inline void INeg11(T x[11]) {
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
inline void INeg12(T x[12]) {
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
inline void INeg13(T x[13]) {
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
inline void INeg14(T x[14]) {
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
inline void INeg15(T x[15]) {
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
inline void INeg16(T x[16]) {
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
// Negate a vector x of length n, save results in a vector y
template <typename T>
inline void INeg1(const T x[1], T y[1]) {
  y[0] = -x[0];
}
template <typename T>
inline void INeg2(const T x[2], T y[2]) {
  y[0] = -x[0];
  y[1] = -x[1];
}
template <typename T>
inline void INeg3(const T x[3], T y[3]) {
  y[0] = -x[0];
  y[1] = -x[1];
  y[2] = -x[2];
}
template <typename T>
inline void INeg4(const T x[4], T y[4]) {
  y[0] = -x[0];
  y[1] = -x[1];
  y[2] = -x[2];
  y[3] = -x[3];
}
template <typename T>
inline void INeg5(const T x[5], T y[5]) {
  y[0] = -x[0];
  y[1] = -x[1];
  y[2] = -x[2];
  y[3] = -x[3];
  y[4] = -x[4];
}
template <typename T>
inline void INeg6(const T x[6], T y[6]) {
  y[0] = -x[0];
  y[1] = -x[1];
  y[2] = -x[2];
  y[3] = -x[3];
  y[4] = -x[4];
  y[5] = -x[5];
}
template <typename T>
inline void INeg7(const T x[7], T y[7]) {
  y[0] = -x[0];
  y[1] = -x[1];
  y[2] = -x[2];
  y[3] = -x[3];
  y[4] = -x[4];
  y[5] = -x[5];
  y[6] = -x[6];
}
template <typename T>
inline void INeg8(const T x[8], T y[8]) {
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
inline void INeg9(const T x[9], T y[9]) {
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
inline void INeg10(const T x[10], T y[10]) {
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
inline void INeg11(const T x[11], T y[11]) {
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
inline void INeg12(const T x[12], T y[12]) {
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
inline void INeg13(const T x[13], T y[13]) {
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
inline void INeg14(const T x[14], T y[14]) {
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
inline void INeg15(const T x[15], T y[15]) {
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
inline void INeg16(const T x[16], T y[16]) {
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

// Negate the cth column of mxn matrix A
template <typename T>
inline void INegCol(T *A, int c, int m, int n) {
  T *ref = A;
  for (int r = 0; r < m; ++r, ref += n) {
    ref[c] = -ref[c];
  }
}

// Compute x=x+c where x is n-dimensional vectors and c is a constant
template <typename T>
inline void IAdd(T *x, int n, T k) {
  for (int i = 0; i < n; i++) {
    x[i] += k;
  }
}
// Compute z=x+y where x, y, and z are n-dimensional vectors
template <typename T>
inline void IAdd(const T *x, const T *y, int n, T *z) {
  for (int i = 0; i < n; i++) {
    z[i] = x[i] + y[i];
  }
}
template <typename T>
inline void IAdd1(const T x[1], const T y[1], T z[1]) {
  z[0] = x[0] + y[0];
}
template <typename T>
inline void IAdd2(const T x[2], const T y[2], T z[2]) {
  z[0] = x[0] + y[0];
  z[1] = x[1] + y[1];
}
template <typename T>
inline void IAdd3(const T x[3], const T y[3], T z[3]) {
  z[0] = x[0] + y[0];
  z[1] = x[1] + y[1];
  z[2] = x[2] + y[2];
}
template <typename T>
inline void IAdd4(const T x[4], const T y[4], T z[4]) {
  z[0] = x[0] + y[0];
  z[1] = x[1] + y[1];
  z[2] = x[2] + y[2];
  z[3] = x[3] + y[3];
}
template <typename T>
inline void IAdd5(const T x[5], const T y[5], T z[5]) {
  z[0] = x[0] + y[0];
  z[1] = x[1] + y[1];
  z[2] = x[2] + y[2];
  z[3] = x[3] + y[3];
  z[4] = x[4] + y[4];
}
template <typename T>
inline void IAdd6(const T x[6], const T y[6], T z[6]) {
  z[0] = x[0] + y[0];
  z[1] = x[1] + y[1];
  z[2] = x[2] + y[2];
  z[3] = x[3] + y[3];
  z[4] = x[4] + y[4];
  z[5] = x[5] + y[5];
}
template <typename T>
inline void IAdd7(const T x[7], const T y[7], T z[7]) {
  z[0] = x[0] + y[0];
  z[1] = x[1] + y[1];
  z[2] = x[2] + y[2];
  z[3] = x[3] + y[3];
  z[4] = x[4] + y[4];
  z[5] = x[5] + y[5];
  z[6] = x[6] + y[6];
}
template <typename T>
inline void IAdd8(const T x[8], const T y[8], T z[8]) {
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
inline void IAdd9(const T x[9], const T y[9], T z[9]) {
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
inline void IAdd10(const T x[10], const T y[10], T z[10]) {
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
inline void IAdd11(const T x[11], const T y[11], T z[11]) {
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
inline void IAdd12(const T x[12], const T y[12], T z[12]) {
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
inline void IAdd13(const T x[13], const T y[13], T z[13]) {
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
inline void IAdd14(const T x[14], const T y[14], T z[14]) {
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
inline void IAdd15(const T x[15], const T y[15], T z[15]) {
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
inline void IAdd16(const T x[16], const T y[16], T z[16]) {
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
inline void IAdd20(const T x[20], const T y[20], T z[20]) {
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

// Compute y=y+x where x and y are n-dimensional vectors
template <typename T>
inline void IAdd(const T *x, T *y, int n) {
  for (int i = 0; i < n; i++) {
    y[i] += x[i];
  }
}
template <typename T>
inline void IAdd1(const T x[1], T y[1]) {
  y[0] += x[0];
}
template <typename T>
inline void IAdd2(const T x[2], T y[2]) {
  y[0] += x[0];
  y[1] += x[1];
}
template <typename T>
inline void IAdd3(const T x[3], T y[3]) {
  y[0] += x[0];
  y[1] += x[1];
  y[2] += x[2];
}
template <typename T>
inline void IAdd4(const T x[4], T y[4]) {
  y[0] += x[0];
  y[1] += x[1];
  y[2] += x[2];
  y[3] += x[3];
}
template <typename T>
inline void IAdd5(const T x[5], T y[5]) {
  y[0] += x[0];
  y[1] += x[1];
  y[2] += x[2];
  y[3] += x[3];
  y[4] += x[4];
}
template <typename T>
inline void IAdd6(const T x[6], T y[6]) {
  y[0] += x[0];
  y[1] += x[1];
  y[2] += x[2];
  y[3] += x[3];
  y[4] += x[4];
  y[5] += x[5];
}
template <typename T>
inline void IAdd7(const T x[7], T y[7]) {
  y[0] += x[0];
  y[1] += x[1];
  y[2] += x[2];
  y[3] += x[3];
  y[4] += x[4];
  y[5] += x[5];
  y[6] += x[6];
}
template <typename T>
inline void IAdd8(const T x[8], T y[8]) {
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
inline void IAdd9(const T x[9], T y[9]) {
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
inline void IAdd10(const T x[10], T y[10]) {
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
inline void IAdd11(const T x[11], T y[11]) {
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
inline void IAdd12(const T x[12], T y[12]) {
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
inline void IAdd13(const T x[13], T y[13]) {
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
inline void IAdd14(const T x[14], T y[14]) {
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
inline void IAdd15(const T x[15], T y[15]) {
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
inline void IAdd16(const T x[16], T y[16]) {
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
inline void IAdd20(const T x[20], T y[20]) {
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

// Compute y=y+x*k where x and y are n-dimensional vectors, k is constant
template <typename T>
inline void IAddScaled(const T *x, T *y, int n, T k) {
  for (int i = 0; i < n; i++) {
    y[i] += (x[i] * k);
  }
}
template <typename T>
inline void IAddScaled1(const T x[1], T y[1], T k) {
  y[0] += x[0] * k;
}
template <typename T>
inline void IAddScaled2(const T x[2], T y[2], T k) {
  y[0] += x[0] * k;
  y[1] += x[1] * k;
}
template <typename T>
inline void IAddScaled3(const T x[3], T y[3], T k) {
  y[0] += x[0] * k;
  y[1] += x[1] * k;
  y[2] += x[2] * k;
}
template <typename T>
inline void IAddScaled4(const T x[4], T y[4], T k) {
  y[0] += x[0] * k;
  y[1] += x[1] * k;
  y[2] += x[2] * k;
  y[3] += x[3] * k;
}
template <typename T>
inline void IAddScaled5(const T x[5], T y[5], T k) {
  y[0] += x[0] * k;
  y[1] += x[1] * k;
  y[2] += x[2] * k;
  y[3] += x[3] * k;
  y[4] += x[4] * k;
}
template <typename T>
inline void IAddScaled6(const T x[6], T y[6], T k) {
  y[0] += x[0] * k;
  y[1] += x[1] * k;
  y[2] += x[2] * k;
  y[3] += x[3] * k;
  y[4] += x[4] * k;
  y[5] += x[5] * k;
}
template <typename T>
inline void IAddScaled7(const T x[7], T y[7], T k) {
  y[0] += x[0] * k;
  y[1] += x[1] * k;
  y[2] += x[2] * k;
  y[3] += x[3] * k;
  y[4] += x[4] * k;
  y[5] += x[5] * k;
  y[6] += x[6] * k;
}
template <typename T>
inline void IAddScaled8(const T x[8], T y[8], T k) {
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
inline void IAddScaled9(const T x[9], T y[9], T k) {
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

// Compute z=x+y*k where x, y and z are n-dimensional vectors, k is constant
template <typename T>
inline void IAddScaled(const T *x, const T *y, T *z, int n, T k) {
  for (int i = 0; i < n; i++) {
    z[i] = x[i] + y[i] * k;
  }
}
template <typename T>
inline void IAddScaled1(const T x[1], const T y[1], T z[1], T k) {
  z[0] = x[0] + y[0] * k;
}
template <typename T>
inline void IAddScaled2(const T x[2], const T y[2], T z[2], T k) {
  z[0] = x[0] + y[0] * k;
  z[1] = x[1] + y[1] * k;
}
template <typename T>
inline void IAddScaled3(const T x[3], const T y[3], T z[3], T k) {
  z[0] = x[0] + y[0] * k;
  z[1] = x[1] + y[1] * k;
  z[2] = x[2] + y[2] * k;
}
template <typename T>
inline void IAddScaled4(const T x[4], const T y[4], T z[4], T k) {
  z[0] = x[0] + y[0] * k;
  z[1] = x[1] + y[1] * k;
  z[2] = x[2] + y[2] * k;
  z[3] = x[3] + y[3] * k;
}
template <typename T>
inline void IAddScaled5(const T x[5], const T y[5], T z[5], T k) {
  z[0] = x[0] + y[0] * k;
  z[1] = x[1] + y[1] * k;
  z[2] = x[2] + y[2] * k;
  z[3] = x[3] + y[3] * k;
  z[4] = x[4] + y[4] * k;
}
template <typename T>
inline void IAddScaled6(const T x[6], const T y[6], T z[6], T k) {
  z[0] = x[0] + y[0] * k;
  z[1] = x[1] + y[1] * k;
  z[2] = x[2] + y[2] * k;
  z[3] = x[3] + y[3] * k;
  z[4] = x[4] + y[4] * k;
  z[5] = x[5] + y[5] * k;
}
template <typename T>
inline void IAddScaled7(const T x[7], const T y[7], T z[7], T k) {
  z[0] = x[0] + y[0] * k;
  z[1] = x[1] + y[1] * k;
  z[2] = x[2] + y[2] * k;
  z[3] = x[3] + y[3] * k;
  z[4] = x[4] + y[4] * k;
  z[5] = x[5] + y[5] * k;
  z[6] = x[6] + y[6] * k;
}
template <typename T>
inline void IAddScaled8(const T x[8], const T y[8], T z[8], T k) {
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
inline void IAddScaled9(const T x[9], const T y[9], T z[9], T k) {
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

// Compute x=x-c where x is n-dimensional vectors and c is a constant
template <typename T>
inline void ISub(T *x, int n, T k) {
  for (int i = 0; i < n; i++) {
    x[i] -= k;
  }
}
// Compute z=x-y where x, y, and z are n-dimensional vectors
template <typename T>
inline void ISub(const T *x, const T *y, int n, T *z) {
  for (int i = 0; i < n; i++) {
    z[i] = x[i] - y[i];
  }
}
template <typename T>
inline void ISub1(const T x[1], const T y[1], T z[1]) {
  z[0] = x[0] - y[0];
}
template <typename T>
inline void ISub2(const T x[2], const T y[2], T z[2]) {
  z[0] = x[0] - y[0];
  z[1] = x[1] - y[1];
}
template <typename T>
inline void ISub3(const T x[3], const T y[3], T z[3]) {
  z[0] = x[0] - y[0];
  z[1] = x[1] - y[1];
  z[2] = x[2] - y[2];
}
template <typename T>
inline void ISub4(const T x[4], const T y[4], T z[4]) {
  z[0] = x[0] - y[0];
  z[1] = x[1] - y[1];
  z[2] = x[2] - y[2];
  z[3] = x[3] - y[3];
}
template <typename T>
inline void ISub5(const T x[5], const T y[5], T z[5]) {
  z[0] = x[0] - y[0];
  z[1] = x[1] - y[1];
  z[2] = x[2] - y[2];
  z[3] = x[3] - y[3];
  z[4] = x[4] - y[4];
}
template <typename T>
inline void ISub6(const T x[6], const T y[6], T z[6]) {
  z[0] = x[0] - y[0];
  z[1] = x[1] - y[1];
  z[2] = x[2] - y[2];
  z[3] = x[3] - y[3];
  z[4] = x[4] - y[4];
  z[5] = x[5] - y[5];
}
template <typename T>
inline void ISub7(const T x[7], const T y[7], T z[7]) {
  z[0] = x[0] - y[0];
  z[1] = x[1] - y[1];
  z[2] = x[2] - y[2];
  z[3] = x[3] - y[3];
  z[4] = x[4] - y[4];
  z[5] = x[5] - y[5];
  z[6] = x[6] - y[6];
}
template <typename T>
inline void ISub8(const T x[8], const T y[8], T z[8]) {
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
inline void ISub9(const T x[9], const T y[9], T z[9]) {
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
inline void ISub10(const T x[10], const T y[10], T z[10]) {
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
inline void ISub11(const T x[11], const T y[11], T z[11]) {
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
inline void ISub12(const T x[12], const T y[12], T z[12]) {
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
inline void ISub13(const T x[13], const T y[13], T z[13]) {
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
inline void ISub14(const T x[14], const T y[14], T z[14]) {
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
inline void ISub15(const T x[15], const T y[15], T z[15]) {
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
inline void ISub16(const T x[16], const T y[16], T z[16]) {
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

// Compute y=y-x where x and y are n-dimensional vectors
template <typename T>
inline void ISub(const T *x, T *y, int n) {
  for (int i = 0; i < n; i++) {
    y[i] -= x[i];
  }
}
template <typename T>
inline void ISub1(const T x[1], T y[1]) {
  y[0] -= x[0];
}
template <typename T>
inline void ISub2(const T x[2], T y[2]) {
  y[0] -= x[0];
  y[1] -= x[1];
}
template <typename T>
inline void ISub3(const T x[3], T y[3]) {
  y[0] -= x[0];
  y[1] -= x[1];
  y[2] -= x[2];
}
template <typename T>
inline void ISub4(const T x[4], T y[4]) {
  y[0] -= x[0];
  y[1] -= x[1];
  y[2] -= x[2];
  y[3] -= x[3];
}
template <typename T>
inline void ISub5(const T x[5], T y[5]) {
  y[0] -= x[0];
  y[1] -= x[1];
  y[2] -= x[2];
  y[3] -= x[3];
  y[4] -= x[4];
}
template <typename T>
inline void ISub6(const T x[6], T y[6]) {
  y[0] -= x[0];
  y[1] -= x[1];
  y[2] -= x[2];
  y[3] -= x[3];
  y[4] -= x[4];
  y[5] -= x[5];
}
template <typename T>
inline void ISub7(const T x[7], T y[7]) {
  y[0] -= x[0];
  y[1] -= x[1];
  y[2] -= x[2];
  y[3] -= x[3];
  y[4] -= x[4];
  y[5] -= x[5];
  y[6] -= x[6];
}
template <typename T>
inline void ISub8(const T x[8], T y[8]) {
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
inline void ISub9(const T x[9], T y[9]) {
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
inline void ISub10(const T x[10], T y[10]) {
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
inline void ISub11(const T x[11], T y[11]) {
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
inline void ISub12(const T x[12], T y[12]) {
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
inline void ISub13(const T x[13], T y[13]) {
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
inline void ISub14(const T x[14], T y[14]) {
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
inline void ISub15(const T x[15], T y[15]) {
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
inline void ISub16(const T x[16], T y[16]) {
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
// Compute y=y-x*k where x and y are n-dimensional vectors, k is constant
template <typename T>
inline void ISubScaled(const T *x, T *y, int n, T k) {
  for (int i = 0; i < n; i++) {
    y[i] -= (x[i] * k);
  }
}
template <typename T>
inline void ISubScaled1(const T x[1], T y[1], T k) {
  y[0] -= x[0] * k;
}
template <typename T>
inline void ISubScaled2(const T x[2], T y[2], T k) {
  y[0] -= x[0] * k;
  y[1] -= x[1] * k;
}
template <typename T>
inline void ISubScaled3(const T x[3], T y[3], T k) {
  y[0] -= x[0] * k;
  y[1] -= x[1] * k;
  y[2] -= x[2] * k;
}
template <typename T>
inline void ISubScaled4(const T x[4], T y[4], T k) {
  y[0] -= x[0] * k;
  y[1] -= x[1] * k;
  y[2] -= x[2] * k;
  y[3] -= x[3] * k;
}
template <typename T>
inline void ISubScaled5(const T x[5], T y[5], T k) {
  y[0] -= x[0] * k;
  y[1] -= x[1] * k;
  y[2] -= x[2] * k;
  y[3] -= x[3] * k;
  y[4] -= x[4] * k;
}
template <typename T>
inline void ISubScaled6(const T x[6], T y[6], T k) {
  y[0] -= x[0] * k;
  y[1] -= x[1] * k;
  y[2] -= x[2] * k;
  y[3] -= x[3] * k;
  y[4] -= x[4] * k;
  y[5] -= x[5] * k;
}
template <typename T>
inline void ISubScaled7(const T x[7], T y[7], T k) {
  y[0] -= x[0] * k;
  y[1] -= x[1] * k;
  y[2] -= x[2] * k;
  y[3] -= x[3] * k;
  y[4] -= x[4] * k;
  y[5] -= x[5] * k;
  y[6] -= x[6] * k;
}
template <typename T>
inline void ISubScaled8(const T x[8], T y[8], T k) {
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
inline void ISubScaled9(const T x[9], T y[9], T k) {
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

// Rescale n-dimensional vector x with a scale factor sf (inplace)
template <typename T>
inline void IScale(T *x, int n, T sf) {
  for (int i = 0; i < n; i++) x[i] *= sf;
}
template <typename T>
inline void IScale1(T x[1], T sf) {
  x[0] *= sf;
}
template <typename T>
inline void IScale2(T x[2], T sf) {
  x[0] *= sf;
  x[1] *= sf;
}
template <typename T>
inline void IScale3(T x[3], T sf) {
  x[0] *= sf;
  x[1] *= sf;
  x[2] *= sf;
}
template <typename T>
inline void IScale4(T x[4], T sf) {
  x[0] *= sf;
  x[1] *= sf;
  x[2] *= sf;
  x[3] *= sf;
}
template <typename T>
inline void IScale5(T x[5], T sf) {
  x[0] *= sf;
  x[1] *= sf;
  x[2] *= sf;
  x[3] *= sf;
  x[4] *= sf;
}
template <typename T>
inline void IScale6(T x[6], T sf) {
  x[0] *= sf;
  x[1] *= sf;
  x[2] *= sf;
  x[3] *= sf;
  x[4] *= sf;
  x[5] *= sf;
}
template <typename T>
inline void IScale7(T x[7], T sf) {
  x[0] *= sf;
  x[1] *= sf;
  x[2] *= sf;
  x[3] *= sf;
  x[4] *= sf;
  x[5] *= sf;
  x[6] *= sf;
}
template <typename T>
inline void IScale8(T x[8], T sf) {
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
inline void IScale9(T x[9], T sf) {
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
inline void IScale10(T x[10], T sf) {
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
inline void IScale11(T x[11], T sf) {
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
inline void IScale12(T x[12], T sf) {
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
inline void IScale13(T x[13], T sf) {
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
inline void IScale14(T x[14], T sf) {
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
inline void IScale15(T x[15], T sf) {
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
inline void IScale16(T x[16], T sf) {
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
// Rescale n-dimensional vector x with a scale factor sf, save the result to
//  * n-dimensional vector y
template <typename T>
inline void IScale(const T *x, T *y, int n, T sf) {
  for (int i = 0; i < n; i++) {
    y[i] = x[i] * sf;
  }
}
template <typename T>
inline void IScale1(const T x[1], T y[1], T sf) {
  y[0] = x[0] * sf;
}
template <typename T>
inline void IScale2(const T x[2], T y[2], T sf) {
  y[0] = x[0] * sf;
  y[1] = x[1] * sf;
}
template <typename T>
inline void IScale3(const T x[3], T y[3], T sf) {
  y[0] = x[0] * sf;
  y[1] = x[1] * sf;
  y[2] = x[2] * sf;
}
template <typename T>
inline void IScale4(const T x[4], T y[4], T sf) {
  y[0] = x[0] * sf;
  y[1] = x[1] * sf;
  y[2] = x[2] * sf;
  y[3] = x[3] * sf;
}
template <typename T>
inline void IScale5(const T x[5], T y[5], T sf) {
  y[0] = x[0] * sf;
  y[1] = x[1] * sf;
  y[2] = x[2] * sf;
  y[3] = x[3] * sf;
  y[4] = x[4] * sf;
}
template <typename T>
inline void IScale6(const T x[6], T y[6], T sf) {
  y[0] = x[0] * sf;
  y[1] = x[1] * sf;
  y[2] = x[2] * sf;
  y[3] = x[3] * sf;
  y[4] = x[4] * sf;
  y[5] = x[5] * sf;
}
template <typename T>
inline void IScale7(const T x[7], T y[7], T sf) {
  y[0] = x[0] * sf;
  y[1] = x[1] * sf;
  y[2] = x[2] * sf;
  y[3] = x[3] * sf;
  y[4] = x[4] * sf;
  y[5] = x[5] * sf;
  y[6] = x[6] * sf;
}
template <typename T>
inline void IScale8(const T x[8], T y[8], T sf) {
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
inline void IScale9(const T x[9], T y[9], T sf) {
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
inline void IScale10(const T x[10], T y[10], T sf) {
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
inline void IScale11(const T x[11], T y[11], T sf) {
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
inline void IScale12(const T x[12], T y[12], T sf) {
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
inline void IScale13(const T x[13], T y[13], T sf) {
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
inline void IScale14(const T x[14], T y[14], T sf) {
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
inline void IScale15(const T x[15], T y[15], T sf) {
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
inline void IScale16(const T x[16], T y[16], T sf) {
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

// Compute dot product of x and y
// inline int IDotU(const unsigned char *x, const unsigned char *y, int n) {
//   int acc = 0;
//   for (int i = 0; i < n; ++i) {
//     acc += (int) x[i] * (int) y[i];
//   }
//   return acc;
// }
template <typename T>
inline T IDot(const T *x, const T *y, int n) {
  T acc = static_cast<T>(0.0);
  for (int i = 0; i < n; ++i) {
    acc += x[i] * y[i];
  }
  return acc;
}
template <typename T>
inline T IDot1(const T x[1], const T y[1]) {
  return (x[0] * y[0]);
}
template <typename T>
inline T IDot2(const T x[2], const T y[2]) {
  return (x[0] * y[0] + x[1] * y[1]);
}
template <typename T>
inline T IDot3(const T x[3], const T y[3]) {
  return (x[0] * y[0] + x[1] * y[1] + x[2] * y[2]);
}
template <typename T>
inline T IDot4(const T x[4], const T y[4]) {
  return (x[0] * y[0] + x[1] * y[1] + x[2] * y[2] + x[3] * y[3]);
}
template <typename T>
inline T IDot5(const T x[5], const T y[5]) {
  return (x[0] * y[0] + x[1] * y[1] + x[2] * y[2] + x[3] * y[3] + x[4] * y[4]);
}
template <typename T>
inline T IDot6(const T x[6], const T y[6]) {
  return (x[0] * y[0] + x[1] * y[1] + x[2] * y[2] + x[3] * y[3] + x[4] * y[4] +
          x[5] * y[5]);
}
template <typename T>
inline T IDot7(const T x[7], const T y[7]) {
  return (x[0] * y[0] + x[1] * y[1] + x[2] * y[2] + x[3] * y[3] + x[4] * y[4] +
          x[5] * y[5] + x[6] * y[6]);
}
template <typename T>
inline T IDot8(const T x[8], const T y[8]) {
  return (x[0] * y[0] + x[1] * y[1] + x[2] * y[2] + x[3] * y[3] + x[4] * y[4] +
          x[5] * y[5] + x[6] * y[6] + x[7] * y[7]);
}
template <typename T>
inline T IDot9(const T x[9], const T y[9]) {
  return (x[0] * y[0] + x[1] * y[1] + x[2] * y[2] + x[3] * y[3] + x[4] * y[4] +
          x[5] * y[5] + x[6] * y[6] + x[7] * y[7] + x[8] * y[8]);
}
template <typename T>
inline T IDot10(const T x[10], const T y[10]) {
  return (x[0] * y[0] + x[1] * y[1] + x[2] * y[2] + x[3] * y[3] + x[4] * y[4] +
          x[5] * y[5] + x[6] * y[6] + x[7] * y[7] + x[8] * y[8] + x[9] * y[9]);
}
template <typename T>
inline T IDot11(const T x[11], const T y[11]) {
  return (x[0] * y[0] + x[1] * y[1] + x[2] * y[2] + x[3] * y[3] + x[4] * y[4] +
          x[5] * y[5] + x[6] * y[6] + x[7] * y[7] + x[8] * y[8] + x[9] * y[9] +
          x[10] * y[10]);
}
template <typename T>
inline T IDot12(const T x[12], const T y[12]) {
  return (x[0] * y[0] + x[1] * y[1] + x[2] * y[2] + x[3] * y[3] + x[4] * y[4] +
          x[5] * y[5] + x[6] * y[6] + x[7] * y[7] + x[8] * y[8] + x[9] * y[9] +
          x[10] * y[10] + x[11] * y[11]);
}
template <typename T>
inline T IDot13(const T x[13], const T y[13]) {
  return (x[0] * y[0] + x[1] * y[1] + x[2] * y[2] + x[3] * y[3] + x[4] * y[4] +
          x[5] * y[5] + x[6] * y[6] + x[7] * y[7] + x[8] * y[8] + x[9] * y[9] +
          x[10] * y[10] + x[11] * y[11] + x[12] * y[12]);
}
template <typename T>
inline T IDot14(const T x[14], const T y[14]) {
  return (x[0] * y[0] + x[1] * y[1] + x[2] * y[2] + x[3] * y[3] + x[4] * y[4] +
          x[5] * y[5] + x[6] * y[6] + x[7] * y[7] + x[8] * y[8] + x[9] * y[9] +
          x[10] * y[10] + x[11] * y[11] + x[12] * y[12] + x[13] * y[13]);
}
template <typename T>
inline T IDot15(const T x[15], const T y[15]) {
  return (x[0] * y[0] + x[1] * y[1] + x[2] * y[2] + x[3] * y[3] + x[4] * y[4] +
          x[5] * y[5] + x[6] * y[6] + x[7] * y[7] + x[8] * y[8] + x[9] * y[9] +
          x[10] * y[10] + x[11] * y[11] + x[12] * y[12] + x[13] * y[13] +
          x[14] * y[14]);
}
template <typename T>
inline T IDot16(const T x[16], const T y[16]) {
  return (x[0] * y[0] + x[1] * y[1] + x[2] * y[2] + x[3] * y[3] + x[4] * y[4] +
          x[5] * y[5] + x[6] * y[6] + x[7] * y[7] + x[8] * y[8] + x[9] * y[9] +
          x[10] * y[10] + x[11] * y[11] + x[12] * y[12] + x[13] * y[13] +
          x[14] * y[14] + x[15] * y[15]);
}

// Compute sum of n-dimensional vector x
inline int ISumU(const unsigned char *x, int n) {
  int acc = 0;
  for (int i = 0; i < n; ++i) {
    acc += static_cast<int>(x[i]);
  }
  return acc;
}
template <typename T>
inline T ISum(const T *x, int n) {
  T acc = static_cast<T>(0.0);
  for (int i = 0; i < n; ++i) {
    acc += x[i];
  }
  return acc;
}
template <typename T>
inline T ISum1(const T x[1]) {
  return (x[0]);
}
template <typename T>
inline T ISum2(const T x[2]) {
  return (x[0] + x[1]);
}
template <typename T>
inline T ISum3(const T x[3]) {
  return (x[0] + x[1] + x[2]);
}
template <typename T>
inline T ISum4(const T x[4]) {
  return (x[0] + x[1] + x[2] + x[3]);
}
template <typename T>
inline T ISum5(const T x[5]) {
  return (x[0] + x[1] + x[2] + x[3] + x[4]);
}
template <typename T>
inline T ISum6(const T x[6]) {
  return (x[0] + x[1] + x[2] + x[3] + x[4] + x[5]);
}
template <typename T>
inline T ISum7(const T x[7]) {
  return (x[0] + x[1] + x[2] + x[3] + x[4] + x[5] + x[6]);
}
template <typename T>
inline T ISum8(const T x[8]) {
  return (x[0] + x[1] + x[2] + x[3] + x[4] + x[5] + x[6] + x[7]);
}
template <typename T>
inline T ISum9(const T x[9]) {
  return (x[0] + x[1] + x[2] + x[3] + x[4] + x[5] + x[6] + x[7] + x[8]);
}
template <typename T>
inline T ISum10(const T x[10]) {
  return (x[0] + x[1] + x[2] + x[3] + x[4] + x[5] + x[6] + x[7] + x[8] + x[9]);
}
template <typename T>
inline T ISum11(const T x[11]) {
  return (x[0] + x[1] + x[2] + x[3] + x[4] + x[5] + x[6] + x[7] + x[8] + x[9] +
          x[10]);
}
template <typename T>
inline T ISum12(const T x[12]) {
  return (x[0] + x[1] + x[2] + x[3] + x[4] + x[5] + x[6] + x[7] + x[8] + x[9] +
          x[10] + x[11]);
}
template <typename T>
inline T ISum13(const T x[13]) {
  return (x[0] + x[1] + x[2] + x[3] + x[4] + x[5] + x[6] + x[7] + x[8] + x[9] +
          x[10] + x[11] + x[12]);
}
template <typename T>
inline T ISum14(const T x[14]) {
  return (x[0] + x[1] + x[2] + x[3] + x[4] + x[5] + x[6] + x[7] + x[8] + x[9] +
          x[10] + x[11] + x[12] + x[13]);
}
template <typename T>
inline T ISum15(const T x[15]) {
  return (x[0] + x[1] + x[2] + x[3] + x[4] + x[5] + x[6] + x[7] + x[8] + x[9] +
          x[10] + x[11] + x[12] + x[13] + x[14]);
}
template <typename T>
inline T ISum16(const T x[16]) {
  return (x[0] + x[1] + x[2] + x[3] + x[4] + x[5] + x[6] + x[7] + x[8] + x[9] +
          x[10] + x[11] + x[12] + x[13] + x[14] + x[15]);
}

template <typename T>
inline T IAbsSum(const T *x, int n) {
  T acc = static_cast<T>(0.0);
  for (int i = 0; i < n; ++i) {
    acc += IAbs(x[i]);
  }
  return acc;
}

// Compute mean of n-dimensional vector x
inline int IMeanU(const unsigned char *x, int n) { return ISumU(x, n) / n; }
template <typename T>
inline T IMean(const T *x, int n) {
  return ISum(x, n) / n;
}
template <typename T>
inline T IMean2(const T x[2]) {
  return ISum2(x) / 2;
}
template <typename T>
inline T IMean3(const T x[3]) {
  return ISum3(x) / 3;
}
template <typename T>
inline T IMean4(const T x[4]) {
  return ISum4(x) / 4;
}
template <typename T>
inline T IMean5(const T x[5]) {
  return ISum5(x) / 5;
}
template <typename T>
inline T IMean6(const T x[6]) {
  return ISum6(x) / 6;
}
template <typename T>
inline T IMean7(const T x[7]) {
  return ISum7(x) / 7;
}
template <typename T>
inline T IMean8(const T x[8]) {
  return ISum8(x) / 8;
}
template <typename T>
inline T IMean9(const T x[9]) {
  return ISum9(x) / 9;
}
template <typename T>
inline T IMean10(const T x[10]) {
  return ISum10(x) / 10;
}
template <typename T>
inline T IMean11(const T x[11]) {
  return ISum11(x) / 11;
}
template <typename T>
inline T IMean12(const T x[12]) {
  return ISum12(x) / 12;
}
template <typename T>
inline T IMean13(const T x[13]) {
  return ISum13(x) / 13;
}
template <typename T>
inline T IMean14(const T x[14]) {
  return ISum14(x) / 14;
}
template <typename T>
inline T IMean15(const T x[15]) {
  return ISum15(x) / 15;
}
template <typename T>
inline T IMean16(const T x[16]) {
  return ISum16(x) / 16;
}

// Compute the sample standard deviation of sample data x
template <typename T>
inline T ISdv(const T *x, T mean, int n) {
  if (n < 2) return static_cast<T>(0.0);
  T sdv = static_cast<T>(0.0);
  for (int i = 0; i < n; ++i) {
    sdv += ISqr(x[i] - mean);
  }
  return ISqrt(IDiv(sdv, n - 1));
}

// Compute square sum of n-dimensional vector x
inline int ISquaresumU(const unsigned char *x, int n) {
  int acc = 0;
  for (int i = 0; i < n; i++) {
    acc += ISqr(x[i]);
  }
  return (acc);
}
template <typename T>
inline T ISquaresum(const T *x, int n) {
  T acc = static_cast<T>(0.0);
  for (int i = 0; i < n; ++i) {
    acc += ISqr(x[i]);
  }
  return acc;
}
template <typename T>
inline T ISquaresum1(const T x[1]) {
  return (ISqr(x[0]));
}
template <typename T>
inline T ISquaresum2(const T x[2]) {
  return (ISqr(x[0]) + ISqr(x[1]));
}
template <typename T>
inline T ISquaresum3(const T x[3]) {
  return (ISqr(x[0]) + ISqr(x[1]) + ISqr(x[2]));
}
template <typename T>
inline T ISquaresum4(const T x[4]) {
  return (ISqr(x[0]) + ISqr(x[1]) + ISqr(x[2]) + ISqr(x[3]));
}
template <typename T>
inline T ISquaresum5(const T x[5]) {
  return (ISqr(x[0]) + ISqr(x[1]) + ISqr(x[2]) + ISqr(x[3]) + ISqr(x[4]));
}
template <typename T>
inline T ISquaresum6(const T x[6]) {
  return (ISqr(x[0]) + ISqr(x[1]) + ISqr(x[2]) + ISqr(x[3]) + ISqr(x[4]) +
          ISqr(x[5]));
}
template <typename T>
inline T ISquaresum7(const T x[7]) {
  return (ISqr(x[0]) + ISqr(x[1]) + ISqr(x[2]) + ISqr(x[3]) + ISqr(x[4]) +
          ISqr(x[5]) + ISqr(x[6]));
}
template <typename T>
inline T ISquaresum8(const T x[8]) {
  return (ISqr(x[0]) + ISqr(x[1]) + ISqr(x[2]) + ISqr(x[3]) + ISqr(x[4]) +
          ISqr(x[5]) + ISqr(x[6]) + ISqr(x[7]));
}
template <typename T>
inline T ISquaresum9(const T x[9]) {
  return (ISqr(x[0]) + ISqr(x[1]) + ISqr(x[2]) + ISqr(x[3]) + ISqr(x[4]) +
          ISqr(x[5]) + ISqr(x[6]) + ISqr(x[7]) + ISqr(x[8]));
}
template <typename T>
inline T ISquaresum10(const T x[10]) {
  return (ISqr(x[0]) + ISqr(x[1]) + ISqr(x[2]) + ISqr(x[3]) + ISqr(x[4]) +
          ISqr(x[5]) + ISqr(x[6]) + ISqr(x[7]) + ISqr(x[8]) + ISqr(x[9]));
}
template <typename T>
inline T ISquaresum11(const T x[11]) {
  return (ISqr(x[0]) + ISqr(x[1]) + ISqr(x[2]) + ISqr(x[3]) + ISqr(x[4]) +
          ISqr(x[5]) + ISqr(x[6]) + ISqr(x[7]) + ISqr(x[8]) + ISqr(x[9]) +
          ISqr(x[10]));
}
template <typename T>
inline T ISquaresum12(const T x[12]) {
  return (ISqr(x[0]) + ISqr(x[1]) + ISqr(x[2]) + ISqr(x[3]) + ISqr(x[4]) +
          ISqr(x[5]) + ISqr(x[6]) + ISqr(x[7]) + ISqr(x[8]) + ISqr(x[9]) +
          ISqr(x[10]) + ISqr(x[11]));
}
template <typename T>
inline T ISquaresum13(const T x[13]) {
  return (ISqr(x[0]) + ISqr(x[1]) + ISqr(x[2]) + ISqr(x[3]) + ISqr(x[4]) +
          ISqr(x[5]) + ISqr(x[6]) + ISqr(x[7]) + ISqr(x[8]) + ISqr(x[9]) +
          ISqr(x[10]) + ISqr(x[11]) + ISqr(x[12]));
}
template <typename T>
inline T ISquaresum14(const T x[14]) {
  return (ISqr(x[0]) + ISqr(x[1]) + ISqr(x[2]) + ISqr(x[3]) + ISqr(x[4]) +
          ISqr(x[5]) + ISqr(x[6]) + ISqr(x[7]) + ISqr(x[8]) + ISqr(x[9]) +
          ISqr(x[10]) + ISqr(x[11]) + ISqr(x[12]) + ISqr(x[13]));
}
template <typename T>
inline T ISquaresum15(const T x[15]) {
  return (ISqr(x[0]) + ISqr(x[1]) + ISqr(x[2]) + ISqr(x[3]) + ISqr(x[4]) +
          ISqr(x[5]) + ISqr(x[6]) + ISqr(x[7]) + ISqr(x[8]) + ISqr(x[9]) +
          ISqr(x[10]) + ISqr(x[11]) + ISqr(x[12]) + ISqr(x[13]) + ISqr(x[14]));
}
template <typename T>
inline T ISquaresum16(const T x[16]) {
  return (ISqr(x[0]) + ISqr(x[1]) + ISqr(x[2]) + ISqr(x[3]) + ISqr(x[4]) +
          ISqr(x[5]) + ISqr(x[6]) + ISqr(x[7]) + ISqr(x[8]) + ISqr(x[9]) +
          ISqr(x[10]) + ISqr(x[11]) + ISqr(x[12]) + ISqr(x[13]) + ISqr(x[14]) +
          ISqr(x[15]));
}

// Compute square sum of the diff (x-y) between two n-dimensional vector x and
//  * y
inline int ISquaresumDiffU(const unsigned char *x, const unsigned char *y,
                           int n) {
  int acc = 0;
  for (int i = 0; i < n; i++) {
    acc += ISqr(static_cast<int>(x[i]) - static_cast<int>(y[i]));
  }
  return acc;
}
template <typename T>
inline T ISquaresumDiffU(const T *x, const T *y, int n) {
  T acc = static_cast<T>(0.0);
  for (int i = 0; i < n; i++) {
    acc += ISqr(x[i] - y[i]);
  }
  return acc;
}
template <typename T>
inline T ISquaresumDiffU1(const T x[1], const T y[1]) {
  return (ISqr(x[0] - y[0]));
}
template <typename T>
inline T ISquaresumDiffU2(const T x[2], const T y[2]) {
  return (ISqr(x[0] - y[0]) + ISqr(x[1] - y[1]));
}
template <typename T>
inline T ISquaresumDiffU3(const T x[3], const T y[3]) {
  return (ISqr(x[0] - y[0]) + ISqr(x[1] - y[1]) + ISqr(x[2] - y[2]));
}
template <typename T>
inline T ISquaresumDiffU4(const T x[4], const T y[4]) {
  return (ISqr(x[0] - y[0]) + ISqr(x[1] - y[1]) + ISqr(x[2] - y[2]) +
          ISqr(x[3] - y[3]));
}
template <typename T>
inline T ISquaresumDiffU5(const T x[5], const T y[5]) {
  return (ISqr(x[0] - y[0]) + ISqr(x[1] - y[1]) + ISqr(x[2] - y[2]) +
          ISqr(x[3] - y[3]) + ISqr(x[4] - y[4]));
}
template <typename T>
inline T ISquaresumDiffU6(const T x[6], const T y[6]) {
  return (ISqr(x[0] - y[0]) + ISqr(x[1] - y[1]) + ISqr(x[2] - y[2]) +
          ISqr(x[3] - y[3]) + ISqr(x[4] - y[4]) + ISqr(x[5] - y[5]));
}
template <typename T>
inline T ISquaresumDiffU7(const T x[7], const T y[7]) {
  return (ISqr(x[0] - y[0]) + ISqr(x[1] - y[1]) + ISqr(x[2] - y[2]) +
          ISqr(x[3] - y[3]) + ISqr(x[4] - y[4]) + ISqr(x[5] - y[5]) +
          ISqr(x[6] - y[6]));
}
template <typename T>
inline T ISquaresumDiffU8(const T x[8], const T y[8]) {
  return (ISqr(x[0] - y[0]) + ISqr(x[1] - y[1]) + ISqr(x[2] - y[2]) +
          ISqr(x[3] - y[3]) + ISqr(x[4] - y[4]) + ISqr(x[5] - y[5]) +
          ISqr(x[6] - y[6]) + ISqr(x[7] - y[7]));
}
template <typename T>
inline T ISquaresumDiffU9(const T x[9], const T y[9]) {
  return (ISqr(x[0] - y[0]) + ISqr(x[1] - y[1]) + ISqr(x[2] - y[2]) +
          ISqr(x[3] - y[3]) + ISqr(x[4] - y[4]) + ISqr(x[5] - y[5]) +
          ISqr(x[6] - y[6]) + ISqr(x[7] - y[7]) + ISqr(x[8] - y[8]));
}
template <typename T>
inline T ISquaresumDiffU10(const T x[10], const T y[10]) {
  return (ISqr(x[0] - y[0]) + ISqr(x[1] - y[1]) + ISqr(x[2] - y[2]) +
          ISqr(x[3] - y[3]) + ISqr(x[4] - y[4]) + ISqr(x[5] - y[5]) +
          ISqr(x[6] - y[6]) + ISqr(x[7] - y[7]) + ISqr(x[8] - y[8]) +
          ISqr(x[9] - y[9]));
}
template <typename T>
inline T ISquaresumDiffU11(const T x[11], const T y[11]) {
  return (ISqr(x[0] - y[0]) + ISqr(x[1] - y[1]) + ISqr(x[2] - y[2]) +
          ISqr(x[3] - y[3]) + ISqr(x[4] - y[4]) + ISqr(x[5] - y[5]) +
          ISqr(x[6] - y[6]) + ISqr(x[7] - y[7]) + ISqr(x[8] - y[8]) +
          ISqr(x[9] - y[9]) + ISqr(x[10] - y[10]));
}
template <typename T>
inline T ISquaresumDiffU12(const T x[12], const T y[12]) {
  return (ISqr(x[0] - y[0]) + ISqr(x[1] - y[1]) + ISqr(x[2] - y[2]) +
          ISqr(x[3] - y[3]) + ISqr(x[4] - y[4]) + ISqr(x[5] - y[5]) +
          ISqr(x[6] - y[6]) + ISqr(x[7] - y[7]) + ISqr(x[8] - y[8]) +
          ISqr(x[9] - y[9]) + ISqr(x[10] - y[10]) + ISqr(x[11] - y[11]));
}
template <typename T>
inline T ISquaresumDiffU13(const T x[13], const T y[13]) {
  return (ISqr(x[0] - y[0]) + ISqr(x[1] - y[1]) + ISqr(x[2] - y[2]) +
          ISqr(x[3] - y[3]) + ISqr(x[4] - y[4]) + ISqr(x[5] - y[5]) +
          ISqr(x[6] - y[6]) + ISqr(x[7] - y[7]) + ISqr(x[8] - y[8]) +
          ISqr(x[9] - y[9]) + ISqr(x[10] - y[10]) + ISqr(x[11] - y[11]) +
          ISqr(x[12] - y[12]));
}
template <typename T>
inline T ISquaresumDiffU14(const T x[14], const T y[14]) {
  return (ISqr(x[0] - y[0]) + ISqr(x[1] - y[1]) + ISqr(x[2] - y[2]) +
          ISqr(x[3] - y[3]) + ISqr(x[4] - y[4]) + ISqr(x[5] - y[5]) +
          ISqr(x[6] - y[6]) + ISqr(x[7] - y[7]) + ISqr(x[8] - y[8]) +
          ISqr(x[9] - y[9]) + ISqr(x[10] - y[10]) + ISqr(x[11] - y[11]) +
          ISqr(x[12] - y[12]) + ISqr(x[13] - y[13]));
}
template <typename T>
inline T ISquaresumDiffU15(const T x[15], const T y[15]) {
  return (ISqr(x[0] - y[0]) + ISqr(x[1] - y[1]) + ISqr(x[2] - y[2]) +
          ISqr(x[3] - y[3]) + ISqr(x[4] - y[4]) + ISqr(x[5] - y[5]) +
          ISqr(x[6] - y[6]) + ISqr(x[7] - y[7]) + ISqr(x[8] - y[8]) +
          ISqr(x[9] - y[9]) + ISqr(x[10] - y[10]) + ISqr(x[11] - y[11]) +
          ISqr(x[12] - y[12]) + ISqr(x[13] - y[13]) + ISqr(x[14] - y[14]));
}
template <typename T>
inline T ISquaresumDiffU16(const T x[16], const T y[16]) {
  return (ISqr(x[0] - y[0]) + ISqr(x[1] - y[1]) + ISqr(x[2] - y[2]) +
          ISqr(x[3] - y[3]) + ISqr(x[4] - y[4]) + ISqr(x[5] - y[5]) +
          ISqr(x[6] - y[6]) + ISqr(x[7] - y[7]) + ISqr(x[8] - y[8]) +
          ISqr(x[9] - y[9]) + ISqr(x[10] - y[10]) + ISqr(x[11] - y[11]) +
          ISqr(x[12] - y[12]) + ISqr(x[13] - y[13]) + ISqr(x[14] - y[14]) +
          ISqr(x[15] - y[15]));
}

// Compute the Hamming distance between two (unsigned) integer arrays
// (considered
//  * as binary values, that is, as sequences of bits)
inline unsigned int IHammingDiff(const unsigned int *x, const unsigned int *y,
                                 int n) {
  unsigned int distance = 0;
  for (int i = 0; i < n; ++i) {
    distance += IHammingLut(x[i], y[i]);
  }
  return distance;  // Return the number of differing bits
}
inline unsigned int IHammingDiff2(const unsigned int x[2],
                                  const unsigned int y[2]) {
  unsigned int distance = 0;
  distance += IHammingLut(x[0], y[0]);
  distance += IHammingLut(x[1], y[1]);
  return distance;
}
inline unsigned int IHammingDiff4(const unsigned int x[4],
                                  const unsigned int y[4]) {
  unsigned int distance = 0;
  distance += IHammingLut(x[0], y[0]);
  distance += IHammingLut(x[1], y[1]);
  distance += IHammingLut(x[2], y[2]);
  distance += IHammingLut(x[3], y[3]);
  return distance;
}
inline unsigned int IHammingDiff8(const unsigned int x[8],
                                  const unsigned int y[8]) {
  unsigned int distance = 0;
  distance += IHammingLut(x[0], y[0]);
  distance += IHammingLut(x[1], y[1]);
  distance += IHammingLut(x[2], y[2]);
  distance += IHammingLut(x[3], y[3]);
  distance += IHammingLut(x[4], y[4]);
  distance += IHammingLut(x[5], y[5]);
  distance += IHammingLut(x[6], y[6]);
  distance += IHammingLut(x[7], y[7]);
  return distance;
}
inline unsigned int IHammingDiff16(const unsigned int x[16],
                                   const unsigned int y[16]) {
  unsigned int distance = 0;
  distance += IHammingLut(x[0], y[0]);
  distance += IHammingLut(x[1], y[1]);
  distance += IHammingLut(x[2], y[2]);
  distance += IHammingLut(x[3], y[3]);
  distance += IHammingLut(x[4], y[4]);
  distance += IHammingLut(x[5], y[5]);
  distance += IHammingLut(x[6], y[6]);
  distance += IHammingLut(x[7], y[7]);
  distance += IHammingLut(x[8], y[8]);
  distance += IHammingLut(x[9], y[9]);
  distance += IHammingLut(x[10], y[10]);
  distance += IHammingLut(x[11], y[11]);
  distance += IHammingLut(x[12], y[12]);
  distance += IHammingLut(x[13], y[13]);
  distance += IHammingLut(x[14], y[14]);
  distance += IHammingLut(x[15], y[15]);
  return distance;
}

// Compute the L2 norm of n-dimensional vector x
inline double IL2Norm(const unsigned char *x, int n) {
  return (ISqrt(ISquaresumU(x, n)));
}
inline double IL2Norm(const int *x, int n) { return (ISqrt(ISquaresum(x, n))); }
inline float IL2Norm(const float *x, int n) {
  return (ISqrt(ISquaresum(x, n)));
}
inline double IL2Norm(const double *x, int n) {
  return (ISqrt(ISquaresum(x, n)));
}

// For type float and double only
template <typename T>
inline T IL2Norm1(const T x[1]) {
  return (x[0]);
}
template <typename T>
inline T IL2Norm2(const T x[2]) {
  return (ISqrt(ISquaresum2(x)));
}
template <typename T>
inline T IL2Norm3(const T x[3]) {
  return (ISqrt(ISquaresum3(x)));
}
template <typename T>
inline T IL2Norm4(const T x[4]) {
  return (ISqrt(ISquaresum4(x)));
}
template <typename T>
inline T IL2Norm5(const T x[5]) {
  return (ISqrt(ISquaresum5(x)));
}
template <typename T>
inline T IL2Norm6(const T x[6]) {
  return (ISqrt(ISquaresum6(x)));
}
template <typename T>
inline T IL2Norm7(const T x[7]) {
  return (ISqrt(ISquaresum7(x)));
}
template <typename T>
inline T IL2Norm8(const T x[8]) {
  return (ISqrt(ISquaresum8(x)));
}
template <typename T>
inline T IL2Norm9(const T x[9]) {
  return (ISqrt(ISquaresum9(x)));
}
template <typename T>
inline T IL2Norm10(const T x[10]) {
  return (ISqrt(ISquaresum10(x)));
}
template <typename T>
inline T IL2Norm11(const T x[11]) {
  return (ISqrt(ISquaresum11(x)));
}
template <typename T>
inline T IL2Norm12(const T x[12]) {
  return (ISqrt(ISquaresum12(x)));
}
template <typename T>
inline T IL2Norm13(const T x[13]) {
  return (ISqrt(ISquaresum13(x)));
}
template <typename T>
inline T IL2Norm14(const T x[14]) {
  return (ISqrt(ISquaresum14(x)));
}
template <typename T>
inline T IL2Norm15(const T x[15]) {
  return (ISqrt(ISquaresum15(x)));
}
template <typename T>
inline T IL2Norm16(const T x[16]) {
  return (ISqrt(ISquaresum16(x)));
}

// Compute sqrt( a^2 + b^2 ) with decent precision, for type float and double
// only
template <typename T>
inline T IL2NormAdv(T a, T b) {
  T absa = IAbs(a);
  T absb = IAbs(b);
  if (absa > absb) {
    return (absa * ISqrt(static_cast<T>(1.0) + ISqr(IDiv(absb, absa))));
  } else {
    return (absb == static_cast<T>(0.0)
                ? static_cast<T>(0.0)
                : absb * ISqrt(static_cast<T>(1.0) + ISqr(IDiv(absa, absb))));
  }
}

// Compute sqrt( x[0]^2 + x[1]^2 ) with decent precision, for type float and
// double only
template <typename T>
inline T IL2NormAdv(const T x[2]) {
  return IL2NormAdv<T>(x[0], x[1]);
}

// Compute the infinity-norm of a m x n matrix A
template <typename T>
inline T IInfinityNorm(const T *A, int m, int n) {
  T infinity_norm = IAbsSum(A, n);
  T tmp;
  int i, ni = n;
  for (i = 1; i < m; ++i, ni += n) {
    tmp = IAbsSum(A + ni, n);
    if (infinity_norm < tmp) {
      infinity_norm = tmp;
    }
  }
  return infinity_norm;
}

// Unitize n-dimensional vector x by dividing its L2-norm and save the result
//  * inplace, for type float and double only
inline void IUnitize(double *x, int n) { IScale(x, n, IRec(IL2Norm(x, n))); }
inline void IUnitize(float *x, int n) { IScale(x, n, IRec(IL2Norm(x, n))); }
inline void ISafeUnitize(double *x, int n) {
  double norm = IL2Norm(x, n);
  if (norm < Constant<double>::MIN_ABS_SAFE_DIVIDEND()) {
    IFill(x, n, IRec(ISqrt(n)));
  } else {
    IScale(x, n, 1.0 / norm);
  }
}
inline void ISafeUnitize(float *x, int n) {
  float norm = IL2Norm(x, n);
  if (norm < Constant<float>::MIN_ABS_SAFE_DIVIDEND()) {
    IFill(x, n, static_cast<float>(IRec(ISqrt(n))));
  } else {
    IScale(x, n, static_cast<float>(1.0) / norm);
  }
}
// Unitize n-dimensional vector x by dividing its L2-norm and save the result in
//  * vector y, for type float and double only
inline void IUnitize(const double *x, double *y, int n) {
  IScale(x, y, n, IRec(IL2Norm(x, n)));
}
inline void IUnitize(const float *x, float *y, int n) {
  IScale(x, y, n, IRec(IL2Norm(x, n)));
}
inline void ISafeUnitize(const double *x, double *y, int n) {
  double norm = IL2Norm(x, n);
  if (norm < Constant<double>::MIN_ABS_SAFE_DIVIDEND()) {
    IFill(y, n, IRec(ISqrt(n)));
  } else {
    IScale(x, y, n, 1.0 / norm);
  }
}
inline void ISafeUnitize(float *x, float *y, int n) {
  float norm = IL2Norm(x, n);
  if (norm < Constant<float>::MIN_ABS_SAFE_DIVIDEND()) {
    IFill(y, n, static_cast<float>(IRec(ISqrt(n))));
  } else {
    IScale(x, y, n, static_cast<float>(1.0) / norm);
  }
}
// For type float and double only!
template <typename T>
inline void IUnitize2(T x[2]) {
  IScale2(x, IRec(IL2Norm2(x)));
}
template <typename T>
inline void IUnitize3(T x[3]) {
  IScale3(x, IRec(IL2Norm3(x)));
}
template <typename T>
inline void IUnitize4(T x[4]) {
  IScale4(x, IRec(IL2Norm4(x)));
}
template <typename T>
inline void IUnitize5(T x[5]) {
  IScale5(x, IRec(IL2Norm5(x)));
}
template <typename T>
inline void IUnitize6(T x[6]) {
  IScale6(x, IRec(IL2Norm6(x)));
}
template <typename T>
inline void IUnitize7(T x[7]) {
  IScale7(x, IRec(IL2Norm7(x)));
}
template <typename T>
inline void IUnitize8(T x[8]) {
  IScale8(x, IRec(IL2Norm8(x)));
}
template <typename T>
inline void IUnitize9(T x[9]) {
  IScale9(x, IRec(IL2Norm9(x)));
}
template <typename T>
inline void IUnitize10(T x[10]) {
  IScale10(x, IRec(IL2Norm10(x)));
}
template <typename T>
inline void IUnitize11(T x[11]) {
  IScale11(x, IRec(IL2Norm11(x)));
}
template <typename T>
inline void IUnitize12(T x[12]) {
  IScale12(x, IRec(IL2Norm12(x)));
}
template <typename T>
inline void IUnitize13(T x[13]) {
  IScale13(x, IRec(IL2Norm13(x)));
}
template <typename T>
inline void IUnitize14(T x[14]) {
  IScale14(x, IRec(IL2Norm14(x)));
}
template <typename T>
inline void IUnitize15(T x[15]) {
  IScale15(x, IRec(IL2Norm15(x)));
}
template <typename T>
inline void IUnitize16(T x[16]) {
  IScale16(x, IRec(IL2Norm16(x)));
}
template <typename T>
inline void IUnitize2(const T x[2], T y[2]) {
  IScale2(x, y, IRec(IL2Norm2(x)));
}
template <typename T>
inline void IUnitize3(const T x[3], T y[3]) {
  IScale3(x, y, IRec(IL2Norm3(x)));
}
template <typename T>
inline void IUnitize4(const T x[4], T y[4]) {
  IScale4(x, y, IRec(IL2Norm4(x)));
}
template <typename T>
inline void IUnitize5(const T x[5], T y[5]) {
  IScale5(x, y, IRec(IL2Norm5(x)));
}
template <typename T>
inline void IUnitize6(const T x[6], T y[6]) {
  IScale6(x, y, IRec(IL2Norm6(x)));
}
template <typename T>
inline void IUnitize7(const T x[7], T y[7]) {
  IScale7(x, y, IRec(IL2Norm7(x)));
}
template <typename T>
inline void IUnitize8(const T x[8], T y[8]) {
  IScale8(x, y, IRec(IL2Norm8(x)));
}
template <typename T>
inline void IUnitize9(const T x[9], T y[9]) {
  IScale9(x, y, IRec(IL2Norm9(x)));
}
template <typename T>
inline void IUnitize10(const T x[10], T y[10]) {
  IScale10(x, y, IRec(IL2Norm10(x)));
}
template <typename T>
inline void IUnitize11(const T x[11], T y[11]) {
  IScale11(x, y, IRec(IL2Norm11(x)));
}
template <typename T>
inline void IUnitize12(const T x[12], T y[12]) {
  IScale12(x, y, IRec(IL2Norm12(x)));
}
template <typename T>
inline void IUnitize13(const T x[13], T y[13]) {
  IScale13(x, y, IRec(IL2Norm13(x)));
}
template <typename T>
inline void IUnitize14(const T x[14], T y[14]) {
  IScale14(x, y, IRec(IL2Norm14(x)));
}
template <typename T>
inline void IUnitize15(const T x[15], T y[15]) {
  IScale15(x, y, IRec(IL2Norm15(x)));
}
template <typename T>
inline void IUnitize16(const T x[16], T y[16]) {
  IScale16(x, y, IRec(IL2Norm16(x)));
}

template <typename T>
inline void ISignedUnitize2(T x[2]) {
  IScale2(x, ISignNeverZero(x[1]) * IRec(IL2Norm2(x)));
}
template <typename T>
inline void ISignedUnitize3(T x[3]) {
  IScale3(x, ISignNeverZero(x[2]) * IRec(IL2Norm3(x)));
}
template <typename T>
inline void ISignedUnitize4(T x[4]) {
  IScale4(x, ISignNeverZero(x[3]) * IRec(IL2Norm4(x)));
}

template <typename T>
inline void IHomogeneousUnitize(T *x, int n) {
  IScale(x, n, IRec(x[n - 1]));
}
template <typename T>
inline void IHomogeneousUnitize2(T x[2]) {
  IScale2(x, IRec(x[1]));
}
template <typename T>
inline void IHomogeneousUnitize3(T x[3]) {
  IScale3(x, IRec(x[2]));
}
template <typename T>
inline void IHomogeneousUnitize4(T x[4]) {
  IScale4(x, IRec(x[3]));
}
template <typename T>
inline void IHomogeneousUnitize9(T x[9]) {
  IScale9(x, IRec(x[8]));
}

template <typename T>
inline void IHomogeneousUnitize(const T *x, T *y, int n) {
  IScale(x, y, n, IRec(x[n - 1]));
}
template <typename T>
inline void IHomogeneousUnitize2(const T x[2], T y[2]) {
  IScale2(x, y, IRec(x[1]));
}
template <typename T>
inline void IHomogeneousUnitize3(const T x[3], T y[3]) {
  IScale3(x, y, IRec(x[2]));
}
template <typename T>
inline void IHomogeneousUnitize4(const T x[4], T y[4]) {
  IScale4(x, y, IRec(x[3]));
}
template <typename T>
inline void IHomogeneousUnitize9(const T x[9], T y[9]) {
  IScale9(x, y, IRec(x[8]));
}

// Compute the centroid of n 3-dimensional vectors
inline void ICentroid3(const double *a, int n, double centroid[3]) {
  int length = 3 * n;
  IFill3(centroid, 0.0);
  for (int i = 0; i < length; i += 3) {
    IAdd3(a + i, centroid);
  }
  IScale3(centroid, IRec(n));
}
inline void ICentroid3(const float *a, int n, float centroid[3]) {
  int length = 3 * n;
  IFill3(centroid, 0.f);
  for (int i = 0; i < length; i += 3) {
    IAdd3(a + i, centroid);
  }
  IScale3(centroid, static_cast<float>(IRec(n)));
}
// Compute the centroid of n 2-dimensional vectors
inline void ICentroid2(const double *a, int n, double centroid[2]) {
  int length = 2 * n;
  IFill2(centroid, 0.0);
  for (int i = 0; i < length; i += 2) {
    IAdd2(a + i, centroid);
  }
  IScale2(centroid, IRec(n));
}
inline void ICentroid2(const float *a, int n, float centroid[2]) {
  int length = 2 * n;
  IFill2(centroid, 0.f);
  for (int i = 0; i < length; i += 2) {
    IAdd2(a + i, centroid);
  }
  IScale2(centroid, static_cast<float>(IRec(n)));
}

// Compute the centroid of n 3-dimensional vectors and their Euclidean distances
//  * to the centroid
inline void ICentroid3(const double *a, int n, double centroid[3],
                       double *distances) {
  int length = 3 * n;
  int i, j;
  IFill3(centroid, 0.0);
  for (i = 0; i < length; i += 3) {
    IAdd3(a + i, centroid);
  }
  IScale3(centroid, IRec(n));
  for (i = 0, j = 0; i < n; ++i, j += 3) {
    distances[i] = ISqrt(ISquaresumDiffU3(a + j, centroid));
  }
}
inline void ICentroid3(const float *a, int n, float centroid[3],
                       float *distances) {
  int length = 3 * n;
  int i, j;
  IFill3(centroid, 0.f);
  for (i = 0; i < length; i += 3) {
    IAdd3(a + i, centroid);
  }
  IScale3(centroid, static_cast<float>(IRec(n)));
  for (i = 0, j = 0; i < n; ++i, j += 3) {
    distances[i] = ISqrt(ISquaresumDiffU3(a + j, centroid));
  }
}
// Compute the centroid of n 2-dimensional vectors and their Euclidean distances
//  * to the centroid
inline void ICentroid2(const double *a, int n, double centroid[2],
                       double *distances) {
  int length = 2 * n;
  int i, j;
  IFill2(centroid, 0.0);
  for (i = 0; i < length; i += 2) {
    IAdd2(a + i, centroid);
  }
  IScale2(centroid, IRec(n));
  for (i = 0, j = 0; i < n; ++i, j += 2) {
    distances[i] = ISqrt(ISquaresumDiffU2(a + j, centroid));
  }
}
inline void ICentroid2(const float *a, int n, float centroid[2],
                       float *distances) {
  int length = 2 * n;
  int i, j;
  IFill2(centroid, 0.f);
  for (i = 0; i < length; i += 2) {
    IAdd2(a + i, centroid);
  }
  IScale2(centroid, static_cast<float>(IRec(n)));
  for (i = 0, j = 0; i < n; ++i, j += 2) {
    distances[i] = ISqrt(ISquaresumDiffU2(a + j, centroid));
  }
}

// Compute the minimum element in array
template <typename T>
inline T IMinElement(const T *a, int n) {
  T val, temp;
  if (n <= 0) return (static_cast<T>(0.0));
  val = a[0];
  for (int i = 1; i < n; i++) {
    temp = a[i];
    if (temp < val) {
      val = temp;
    }
  }
  return (val);
}
// Compute the maximum element in array
template <typename T>
inline T IMaxElement(const T *a, int n) {
  T val, temp;
  if (n <= 0) return (static_cast<T>(0.0));
  val = a[0];
  for (int i = 1; i < n; i++) {
    temp = a[i];
    if (temp > val) {
      val = temp;
    }
  }
  return (val);
}
// Compute the minimum element and its index in array
// template <typename T>
// inline void IMinElement(const T *a, int n, T &min_val, int &index) {
//   T temp;
//   index = 0;
//   if (n <= 0) {
//     min_val = static_cast<T>(0.0);
//     return;
//   }
//   min_val = a[0];
//   for (int i = 1; i < n; i++) {
//     temp = a[i];
//     if (temp < min_val) {
//       min_val = temp;
//       index = i;
//     }
//   }
// }
// Compute the maximum element and its index in array
// template <typename T>
// inline void IMaxElement(const T *a, int n, T &max_val, int &index) {
//   T temp;
//   index = 0;
//   if (n <= 0) {
//     max_val = static_cast<T>(0.0);
//     return;
//   }
//   max_val = a[0];
//   for (int i = 1; i < n; i++) {
//     temp = a[i];
//     if (temp > max_val) {
//       max_val = temp;
//       index = i;
//     }
//   }
// }

// Compute the maximum diagonal element of a n x n square matrix
template <typename T>
inline T IMaxDiagonalElement(const T *a, int n) {
  T val, temp;
  if (n <= 0) return (static_cast<T>(0.0));
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

// Compute the index of element with largest element in array
inline int IMaxIndex(const double *a, int n) {
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
inline int IMaxIndex(const float *a, int n) {
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
inline int IMaxIndex(const int *a, int n) {
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

// Compute the index of element with largest  magnitude element in array
inline int IMaxAbsIndex(const double *a, int n) {
  int bi;
  double b, t;
  if (n <= 0) return (0);
  b = IAbs(a[0]);
  bi = 0;
  for (int i = 1; i < n; i++)
    if ((t = IAbs(a[i])) > b) {
      b = t;
      bi = i;
    }
  return (bi);
}
inline int IMaxAbsIndex(const float *a, int n) {
  int bi;
  float b, t;
  if (n <= 0) return (0);
  b = IAbs(a[0]);
  bi = 0;
  for (int i = 1; i < n; i++)
    if ((t = IAbs(a[i])) > b) {
      b = t;
      bi = i;
    }
  return (bi);
}
inline int IMaxAbsIndex(const int *a, int n) {
  int bi;
  int b, t;
  if (n <= 0) return (0);
  b = IAbs(a[0]);
  bi = 0;
  for (int i = 1; i < n; i++)
    if ((t = IAbs(a[i])) > b) {
      b = t;
      bi = i;
    }
  return (bi);
}
// Compute the index of element with smallest magnitude element in array
inline int IMinAbsIndex(const double *a, int n) {
  int bi;
  double b, t;
  if (n <= 0) return (0);
  b = IAbs(a[0]);
  bi = 0;
  for (int i = 1; i < n; i++)
    if ((t = IAbs(a[i])) < b) {
      b = t;
      bi = i;
    }
  return (bi);
}
inline int IMinAbsIndex(const float *a, int n) {
  int bi;
  float b, t;
  if (n <= 0) return (0);
  b = IAbs(a[0]);
  bi = 0;
  for (int i = 1; i < n; i++)
    if ((t = IAbs(a[i])) < b) {
      b = t;
      bi = i;
    }
  return (bi);
}
inline int IMinAbsIndex(const int *a, int n) {
  int bi;
  int b, t;
  if (n <= 0) return (0);
  b = IAbs(a[0]);
  bi = 0;
  for (int i = 1; i < n; i++)
    if ((t = IAbs(a[i])) < b) {
      b = t;
      bi = i;
    }
  return (bi);
}

// Compute the index of element in interval [i1,i2) with largest magnitude
//  * element in array
inline int IMaxAbsIndexInterval(const double *a, int i1, int i2) {
  int bi;
  double b, t;
  b = IAbs(a[i1]);
  bi = i1;
  for (int i = i1 + 1; i < i2; i++) {
    if ((t = IAbs(a[i])) > b) {
      b = t;
      bi = i;
    }
  }
  return (bi);
}
inline int IMaxAbsIndexInterval(const float *a, int i1, int i2) {
  int bi;
  float b, t;
  b = IAbs(a[i1]);
  bi = i1;
  for (int i = i1 + 1; i < i2; i++) {
    if ((t = IAbs(a[i])) > b) {
      b = t;
      bi = i;
    }
  }
  return (bi);
}
inline int IMaxAbsIndexInterval(const int *a, int i1, int i2) {
  int bi;
  int b, t;
  b = IAbs(a[i1]);
  bi = i1;
  for (int i = i1 + 1; i < i2; i++) {
    if ((t = IAbs(a[i])) > b) {
      b = t;
      bi = i;
    }
  }
  return (bi);
}
// Compute the index of element in interval [i1,i2) with smallest magnitude
//  * element in array
inline int IMinAbsIndexInterval(const double *a, int i1, int i2) {
  int bi;
  double b, t;
  b = IAbs(a[i1]);
  bi = i1;
  for (int i = i1 + 1; i < i2; i++) {
    if ((t = IAbs(a[i])) < b) {
      b = t;
      bi = i;
    }
  }
  return (bi);
}
inline int IMinAbsIndexInterval(const float *a, int i1, int i2) {
  int bi;
  float b, t;
  b = IAbs(a[i1]);
  bi = i1;
  for (int i = i1 + 1; i < i2; i++) {
    if ((t = IAbs(a[i])) < b) {
      b = t;
      bi = i;
    }
  }
  return (bi);
}
inline int IMinAbsIndexInterval(const int *a, int i1, int i2) {
  int bi;
  int b, t;
  b = IAbs(a[i1]);
  bi = i1;
  for (int i = i1 + 1; i < i2; i++) {
    if ((t = IAbs(a[i])) < b) {
      b = t;
      bi = i;
    }
  }
  return (bi);
}

// Compute the index of element in interval [i1,i2) with largest magnitude
//  * element in a column of a mxn matrix
inline int IMaxAbsIndexIntervalColumn(const double *a, int i1, int i2, int n) {
  int bi;
  double b, t;
  const double *ref = a + n * i1;
  b = IAbs(*ref);
  bi = i1;
  ref += n;
  for (int i = i1 + 1; i < i2; ++i, ref += n) {
    if ((t = IAbs(*ref)) > b) {
      b = t;
      bi = i;
    }
  }
  return (bi);
}
inline int IMaxAbsIndexIntervalColumn(const float *a, int i1, int i2, int n) {
  int bi;
  float b, t;
  const float *ref = a + i1 * n;
  b = IAbs(*ref);
  bi = i1;
  for (int i = i1 + 1; i < i2; ++i, ref += n) {
    if ((t = IAbs(*ref)) > b) {
      b = t;
      bi = i;
    }
  }
  return (bi);
}
inline int IMaxAbsIndexIntervalColumn(const int *a, int i1, int i2, int n) {
  int b, bi, t;
  const int *ref = a + i1 * n;
  b = IAbs(*ref);
  bi = i1;
  for (int i = i1; i < i2; ++i, ref += n) {
    if ((t = IAbs(*ref)) > b) {
      b = t;
      bi = i;
    }
  }
  return (bi);
}

// Compute the index of element in interval [i1,i2) with smallest magnitude
//  * element in a column of a mxn matrix
inline int IMinAbsIndexIntervalColumn(const double *a, int i1, int i2, int n) {
  int bi;
  double b, t;
  const double *ref = a + n * i1;
  b = IAbs(*ref);
  bi = i1;
  for (int i = i1 + 1; i < i2; ++i, ref += n) {
    if ((t = IAbs(*ref)) < b) {
      b = t;
      bi = i;
    }
  }
  return (bi);
}
inline int IMinAbsIndexIntervalColumn(const float *a, int i1, int i2, int n) {
  int bi;
  float b, t;
  const float *ref = a + i1 * n;
  b = IAbs(*ref);
  bi = i1;
  for (int i = i1 + 1; i < i2; ++i, ref += n) {
    if ((t = IAbs(*ref)) < b) {
      b = t;
      bi = i;
    }
  }
  return (bi);
}
inline int IMinAbsIndexIntervalColumn(const int *a, int i1, int i2, int n) {
  int b, bi, t;
  const int *ref = a + i1 * n;
  b = IAbs(*ref);
  bi = i1;
  for (int i = i1; i < i2; ++i, ref += n) {
    if ((t = IAbs(*ref)) < b) {
      b = t;
      bi = i;
    }
  }
  return (bi);
}

// Find row-index of element on or below the diagonal of column i of (n x n)
// matrix A
// with the largest absolute value.
template <typename T>
inline int IMaxAbsIndexSubdiagonalColumn(const T *A, int i, int n) {
  int j, largest_j;
  T largest_val, temp;
  largest_val = IAbs(A[i * n + i]);
  largest_j = i;
  for (j = i + 1; j < n; j++) {
    temp = IAbs(A[j * n + i]);
    if (temp > largest_val) {
      largest_val = temp;
      largest_j = j;
    }
  }
  return largest_j;
}

// Compute the minimum and maximum elements in an array
template <typename T>
inline void IMinMaxElements(const T *a, int n, T *min_val, T *max_val) {
  T temp;
  if (n <= 0) {
    *min_val = *max_val = static_cast<T>(0.0);
    return;
  }
  *min_val = *max_val = a[0];
  for (int i = 1; i < n; i++) {
    temp = a[i];
    if (temp > *max_val) {
      *max_val = temp;
    }
    if (temp < *min_val) {
      *min_val = temp;
    }
  }
  return;
}

// Compute the minimum and maximum elements in an array, ingoring the
//  * "ignored_val" in a
// template <typename T>
// inline void IMinMaxElements(const T *a, int n, T &min_val, T &max_val,
//                             const T ignored_val) {
//   T temp;
//   int i;
//   if (n <= 0) {
//     min_val = max_val = static_cast<T>(0.0);
//     return;
//   }
//   for (i = 0; i < n; ++i) {
//     if (a[i] != ignored_val) {
//       break;
//     }
//   }
//   min_val = max_val = a[i];
//   for (; i < n; i++) {
//     temp = a[i];
//     if (temp == ignored_val) {
//       continue;
//     }
//     if (temp > max_val) {
//       max_val = temp;
//     }
//     if (temp < min_val) {
//       min_val = temp;
//     }
//   }
//   return;
// }

// Given a n-dimensional vector x, construct its homogeneous representation
//  * (n+1-dimensional vector) y by adding 1 to the last entry
template <typename T>
inline void IHomogenize(const T *x, T *y, int n) {
  for (int i = 0; i < n; ++i) {
    y[i] = x[i];
  }
  y[n] = static_cast<T>(1.0);
}
template <typename T>
inline void IHomogenize1(const T x[1], T y[2]) {
  y[0] = x[0];
  y[1] = static_cast<T>(1.0);
}
template <typename T>
inline void IHomogenize2(const T x[2], T y[3]) {
  y[0] = x[0];
  y[1] = x[1];
  y[2] = static_cast<T>(1.0);
}
template <typename T>
inline void IHomogenize3(const T x[3], T y[4]) {
  y[0] = x[0];
  y[1] = x[1];
  y[2] = x[2];
  y[3] = static_cast<T>(1.0);
}

// Compute the cross product between two 3-dimensional vectors x and y
template <typename T>
inline void ICross(const T x[3], const T y[3], T xxy[3]) {
  xxy[0] = x[1] * y[2] - x[2] * y[1];
  xxy[1] = x[2] * y[0] - x[0] * y[2];
  xxy[2] = x[0] * y[1] - x[1] * y[0];
}

// Compute the 3x3 skew-symmetric matrix [e]_x such that [e]_x * y is the cross
//  * product of 3-dimensional vectors x and y for all y
template <typename T>
inline void IAxiator(const T x[3], T e_x[9]) {
  e_x[0] = static_cast<T>(0.0);
  e_x[1] = -x[2];
  e_x[2] = x[1];
  e_x[3] = x[2];
  e_x[4] = static_cast<T>(0.0);
  e_x[5] = -x[0];
  e_x[6] = -x[1];
  e_x[7] = x[0];
  e_x[8] = static_cast<T>(0.0);
}

// Compute the square of a 3x3 skew-symmetric matrix [e]_x, e_x2 = [e_x]^2 =
//  * [e]_x*[e]_x
template <typename T>
inline void ISqrSkewSymmetric3x3(const T x[3], T e_x2[9]) {
  T x0_sqr = ISqr(x[0]);
  T x1_sqr = ISqr(x[1]);
  T x2_sqr = ISqr(x[2]);
  e_x2[0] = -(x1_sqr + x2_sqr);
  e_x2[1] = e_x2[3] = x[0] * x[1];
  e_x2[4] = -(x2_sqr + x0_sqr);
  e_x2[2] = e_x2[6] = x[2] * x[0];
  e_x2[8] = -(x0_sqr + x1_sqr);
  e_x2[5] = e_x2[7] = x[1] * x[2];
}

// Set the nxn matrix A to be an identity matrix
template <typename T>
inline void IEye(T *A, int n) {
  int in = 0;
  IZero(A, n * n);
  for (int i = 0; i < n; ++i, in += n) {
    A[in + i] = static_cast<T>(1.0);
  }
}
template <typename T>
inline void IEye2x2(T A[4]) {
  A[0] = A[3] = static_cast<T>(1.0);
  A[1] = A[2] = static_cast<T>(0.0);
}
template <typename T>
inline void IEye3x3(T A[9]) {
  A[0] = A[4] = A[8] = static_cast<T>(1.0);
  A[1] = A[2] = A[3] = A[5] = A[6] = A[7] = static_cast<T>(0.0);
}
template <typename T>
inline void IEye4x4(T A[16]) {
  A[0] = A[5] = A[10] = A[15] = static_cast<T>(1.0);
  A[1] = A[2] = A[3] = A[4] = A[6] = A[7] = A[8] = A[9] = A[11] = A[12] =
      A[13] = A[14] = static_cast<T>(0.0);
}

// Construct the 2x2 upper triangular matrix A
template <typename T>
inline void IUpperTriangular2x2(T a0, T a1, T a3, T A[4]) {
  A[0] = a0;
  A[1] = a1;
  A[2] = static_cast<T>(0.0);
  A[3] = a3;
}

// Construct the 3x3 upper triangular matrix A
template <typename T>
inline void IUpperTriangular3x3(T a0, T a1, T a2, T a4, T a5, T a8, T A[9]) {
  A[0] = a0;
  A[1] = a1;
  A[2] = a2;
  A[3] = static_cast<T>(0.0);
  A[4] = a4;
  A[5] = a5;
  A[6] = static_cast<T>(0.0);
  A[7] = static_cast<T>(0.0);
  A[8] = a8;
}

// Compute the trace of a 2x2 matrix A
inline double ITrace2x2(const double A[4]) { return (A[0] + A[3]); }
inline float ITrace2x2(const float A[4]) { return (A[0] + A[3]); }
// Compute the trace of a 3x3 matrix A
inline double ITrace3x3(const double A[9]) { return (A[0] + A[4] + A[8]); }
inline float ITrace3x3(const float A[9]) { return (A[0] + A[4] + A[8]); }

// Compute the determinant of a 2x2 matrix A
inline double IDeterminant2x2(const double A[4]) {
  return (A[0] * A[3] - A[1] * A[2]);
}
inline float IDeterminant2x2(const float A[4]) {
  return (A[0] * A[3] - A[1] * A[2]);
}
inline int IDeterminant2x2(const int A[4]) {
  return (A[0] * A[3] - A[1] * A[2]);
}

// Compute the determinant of a 3x3 matrix A
inline double IDeterminant3x3(const double A[9]) {
  double r0_x_r1[3];
  ICross(A, A + 3, r0_x_r1);
  return (IDot3(r0_x_r1, A + 6));
}
inline float IDeterminant3x3(const float A[9]) {
  float r0_x_r1[3];
  ICross(A, A + 3, r0_x_r1);
  return (IDot3(r0_x_r1, A + 6));
}
inline int IDeterminant3x3(const int A[9]) {
  int r0_x_r1[3];
  ICross(A, A + 3, r0_x_r1);
  return (IDot3(r0_x_r1, A + 6));
}

// Compute the 6 subdeterminants {sd[0],...,sd[5]} of the 2x4 matrix formed by x
//  * as row 0 and y as row 1
inline void ISubdeterminants2x4(const double x[4], const double y[4],
                                double sd[6]) {
  sd[0] = x[0] * y[1] - x[1] * y[0];
  sd[1] = x[0] * y[2] - x[2] * y[0];
  sd[2] = x[0] * y[3] - x[3] * y[0];
  sd[3] = x[1] * y[2] - x[2] * y[1];
  sd[4] = x[1] * y[3] - x[3] * y[1];
  sd[5] = x[2] * y[3] - x[3] * y[2];
}
inline void ISubdeterminants2x4(const float x[4], const float y[4],
                                float sd[6]) {
  sd[0] = x[0] * y[1] - x[1] * y[0];
  sd[1] = x[0] * y[2] - x[2] * y[0];
  sd[2] = x[0] * y[3] - x[3] * y[0];
  sd[3] = x[1] * y[2] - x[2] * y[1];
  sd[4] = x[1] * y[3] - x[3] * y[1];
  sd[5] = x[2] * y[3] - x[3] * y[2];
}
inline void ISubdeterminants2x4(const int x[4], const int y[4], int sd[6]) {
  sd[0] = x[0] * y[1] - x[1] * y[0];
  sd[1] = x[0] * y[2] - x[2] * y[0];
  sd[2] = x[0] * y[3] - x[3] * y[0];
  sd[3] = x[1] * y[2] - x[2] * y[1];
  sd[4] = x[1] * y[3] - x[3] * y[1];
  sd[5] = x[2] * y[3] - x[3] * y[2];
}

// Compute the 4 subdeterminants {sd[0],...,sd[3]} of the 3x4 matrix formed by x
//  * as row 0, y as row 1, and z as row 2
inline void ISubdeterminants3x4(const double x[4], const double y[4],
                                const double z[4], double sd[4]) {
  double ssd[6];
  ISubdeterminants2x4(x, y, ssd);
  sd[0] = z[1] * ssd[5] - z[2] * ssd[4] + z[3] * ssd[3];
  sd[1] = z[0] * ssd[5] - z[2] * ssd[2] + z[3] * ssd[1];
  sd[2] = z[0] * ssd[4] - z[1] * ssd[2] + z[3] * ssd[0];
  sd[3] = z[0] * ssd[3] - z[1] * ssd[1] + z[2] * ssd[0];
}
inline void ISubdeterminants3x4(const float x[4], const float y[4],
                                const float z[4], float sd[4]) {
  float ssd[6];
  ISubdeterminants2x4(x, y, ssd);
  sd[0] = z[1] * ssd[5] - z[2] * ssd[4] + z[3] * ssd[3];
  sd[1] = z[0] * ssd[5] - z[2] * ssd[2] + z[3] * ssd[1];
  sd[2] = z[0] * ssd[4] - z[1] * ssd[2] + z[3] * ssd[0];
  sd[3] = z[0] * ssd[3] - z[1] * ssd[1] + z[2] * ssd[0];
}
inline void ISubdeterminants3x4(const int x[4], const int y[4], const int z[4],
                                int sd[4]) {
  int ssd[6];
  ISubdeterminants2x4(x, y, ssd);
  sd[0] = z[1] * ssd[5] - z[2] * ssd[4] + z[3] * ssd[3];
  sd[1] = z[0] * ssd[5] - z[2] * ssd[2] + z[3] * ssd[1];
  sd[2] = z[0] * ssd[4] - z[1] * ssd[2] + z[3] * ssd[0];
  sd[3] = z[0] * ssd[3] - z[1] * ssd[1] + z[2] * ssd[0];
}

// Compute the determinant of a 4x4 matrix A
inline double IDeterminant4x4(const double A[16]) {
  double sd[4];
  ISubdeterminants3x4(A, A + 4, A + 8, sd);
  return -(A[12] * sd[0]) + (A[13] * sd[1]) - (A[14] * sd[2]) + (A[15] * sd[3]);
}
inline float IDeterminant4x4(const float A[16]) {
  float sd[4];
  ISubdeterminants3x4(A, A + 4, A + 8, sd);
  return -(A[12] * sd[0]) + (A[13] * sd[1]) - (A[14] * sd[2]) + (A[15] * sd[3]);
}
inline int IDeterminant4x4(const int A[16]) {
  int sd[4];
  ISubdeterminants3x4(A, A + 4, A + 8, sd);
  return -(A[12] * sd[0]) + (A[13] * sd[1]) - (A[14] * sd[2]) + (A[15] * sd[3]);
}

// Compute the nullptrvector to x,y and z => intersection of three planes x, y,
// z
// is
//  * a point
template <typename T>
inline void ICross(const T x[4], const T y[4], const T z[4], T xxyxz[4]) {
  ISubdeterminants3x4(x, y, z, xxyxz);
  xxyxz[0] = -xxyxz[0];
  xxyxz[2] = -xxyxz[2];
}

// Compute the inverse of a 2x2 matrix A using the Cramer's rule
inline void IInvert2x2(const double A[4], double Ai[4]) {
  double d = IDeterminant2x2(A);
  double sf = IRec(d);
  Ai[0] = sf * (A[3]);
  Ai[1] = sf * (-A[1]);
  Ai[2] = sf * (-A[2]);
  Ai[3] = sf * (A[0]);
}

inline void IInvert2x2(const float A[4], float Ai[4]) {
  float d = IDeterminant2x2(A);
  float sf = IRec(d);
  Ai[0] = sf * (A[3]);
  Ai[1] = sf * (-A[1]);
  Ai[2] = sf * (-A[2]);
  Ai[3] = sf * (A[0]);
}

inline void IInvert2x2(const int A[4], double Ai[4]) {
  int d = IDeterminant2x2(A);
  double sf = IRec(d);
  Ai[0] = sf * (A[3]);
  Ai[1] = sf * (-A[1]);
  Ai[2] = sf * (-A[2]);
  Ai[3] = sf * (A[0]);
}

// inline void ISafeInvert2x2(const double A[4], double Ai[4],
//                            bool &is_invertible) {
//   double d = IDeterminant2x2(A);
//   double sf = IRec(d);
//   Ai[0] = sf * (A[3]);
//   Ai[1] = sf * (-A[1]);
//   Ai[2] = sf * (-A[2]);
//   Ai[3] = sf * (A[0]);
//   is_invertible =
//       (IAbs(d) > Constant<double>::MIN_ABS_SAFE_DIVIDEND()) ? true : false;
// }

// inline void ISafeInvert2x2(const float A[4], float Ai[4], bool
// &is_invertible) {
//   float d = IDeterminant2x2(A);
//   float sf = IRec(d);
//   Ai[0] = sf * (A[3]);
//   Ai[1] = sf * (-A[1]);
//   Ai[2] = sf * (-A[2]);
//   Ai[3] = sf * (A[0]);
//   is_invertible =
//       (IAbs(d) > Constant<float>::MIN_ABS_SAFE_DIVIDEND()) ? true : false;
// }

// inline void ISafeInvert2x2(const int A[4], double Ai[4], bool &is_invertible)
// {
//   int d = IDeterminant2x2(A);
//   double sf = IRec(d);
//   Ai[0] = sf * (A[3]);
//   Ai[1] = sf * (-A[1]);
//   Ai[2] = sf * (-A[2]);
//   Ai[3] = sf * (A[0]);
//   is_invertible = (d != 0) ? true : false;
// }

// Compute the inverse of a 3x3 matrix A using the Cramer's rule
inline void IInvert3x3(const double A[9], double Ai[9]) {
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
  double d_rec = IRec(d);
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
inline void IInvert3x3(const float A[9], float Ai[9]) {
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
  float d_rec = IRec(d);
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
inline void IInvert3x3(const int A[9], double Ai[9]) {
  // subdeterminants:
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
  double d_rec = IRec(d);
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
// inline void ISafeInvert3x3(const double A[9], double Ai[9],
//                            bool &is_invertible) {
//   // subdeterminants:
//   double sd0 = A[4] * A[8] - A[5] * A[7];
//   double sd1 = A[5] * A[6] - A[3] * A[8];
//   double sd2 = A[3] * A[7] - A[4] * A[6];
//   double sd3 = A[2] * A[7] - A[1] * A[8];
//   double sd4 = A[0] * A[8] - A[2] * A[6];
//   double sd5 = A[1] * A[6] - A[0] * A[7];
//   double sd6 = A[1] * A[5] - A[2] * A[4];
//   double sd7 = A[2] * A[3] - A[0] * A[5];
//   double sd8 = A[0] * A[4] - A[1] * A[3];
//   double d = A[0] * sd0 + A[1] * sd1 + A[2] * sd2;
//   is_invertible =
//       (IAbs(d) > Constant<double>::MIN_ABS_SAFE_DIVIDEND()) ? true : false;
//   double d_rec = is_invertible ? IRec(d) : 1.0;
//   Ai[0] = d_rec * sd0;
//   Ai[1] = d_rec * sd3;
//   Ai[2] = d_rec * sd6;
//   Ai[3] = d_rec * sd1;
//   Ai[4] = d_rec * sd4;
//   Ai[5] = d_rec * sd7;
//   Ai[6] = d_rec * sd2;
//   Ai[7] = d_rec * sd5;
//   Ai[8] = d_rec * sd8;
// }
// inline void ISafeInvert3x3(const float A[9], float Ai[9], bool
// &is_invertible) {
//   // subdeterminants:
//   float sd0 = A[4] * A[8] - A[5] * A[7];
//   float sd1 = A[5] * A[6] - A[3] * A[8];
//   float sd2 = A[3] * A[7] - A[4] * A[6];
//   float sd3 = A[2] * A[7] - A[1] * A[8];
//   float sd4 = A[0] * A[8] - A[2] * A[6];
//   float sd5 = A[1] * A[6] - A[0] * A[7];
//   float sd6 = A[1] * A[5] - A[2] * A[4];
//   float sd7 = A[2] * A[3] - A[0] * A[5];
//   float sd8 = A[0] * A[4] - A[1] * A[3];
//   float d = A[0] * sd0 + A[1] * sd1 + A[2] * sd2;
//   is_invertible =
//       (IAbs(d) > Constant<float>::MIN_ABS_SAFE_DIVIDEND()) ? true : false;
//   float d_rec = is_invertible ? IRec(d) : 1.f;
//   Ai[0] = d_rec * sd0;
//   Ai[1] = d_rec * sd3;
//   Ai[2] = d_rec * sd6;
//   Ai[3] = d_rec * sd1;
//   Ai[4] = d_rec * sd4;
//   Ai[5] = d_rec * sd7;
//   Ai[6] = d_rec * sd2;
//   Ai[7] = d_rec * sd5;
//   Ai[8] = d_rec * sd8;
// }
// inline void ISafeInvert3x3(const int A[9], double Ai[9], bool &is_invertible)
// {
//   // subdeterminants:
//   int sd0 = A[4] * A[8] - A[5] * A[7];
//   int sd1 = A[5] * A[6] - A[3] * A[8];
//   int sd2 = A[3] * A[7] - A[4] * A[6];
//   int sd3 = A[2] * A[7] - A[1] * A[8];
//   int sd4 = A[0] * A[8] - A[2] * A[6];
//   int sd5 = A[1] * A[6] - A[0] * A[7];
//   int sd6 = A[1] * A[5] - A[2] * A[4];
//   int sd7 = A[2] * A[3] - A[0] * A[5];
//   int sd8 = A[0] * A[4] - A[1] * A[3];
//   int d = A[0] * sd0 + A[1] * sd1 + A[2] * sd2;
//   is_invertible = (d != 0) ? true : false;
//   double d_rec = is_invertible ? IRec(d) : 1.0;
//   Ai[0] = d_rec * sd0;
//   Ai[1] = d_rec * sd3;
//   Ai[2] = d_rec * sd6;
//   Ai[3] = d_rec * sd1;
//   Ai[4] = d_rec * sd4;
//   Ai[5] = d_rec * sd7;
//   Ai[6] = d_rec * sd2;
//   Ai[7] = d_rec * sd5;
//   Ai[8] = d_rec * sd8;
// }

inline void IInvert3x3UpperTriangular(const double A[9], double Ai[9]) {
  double A4A8 = A[4] * A[8];
  double A0rec = IRec(A[0]);
  Ai[0] = A0rec;
  Ai[1] = -IDiv(A[1], A[0] * A[4]);
  Ai[2] = (IDiv(A[1] * A[5], A4A8) - IDiv(A[2], A[8])) * A0rec;
  Ai[3] = 0;
  Ai[4] = IRec(A[4]);
  Ai[5] = -IDiv(A[5], A4A8);
  Ai[6] = 0;
  Ai[7] = 0;
  Ai[8] = IRec(A[8]);
}
inline void IInvert3x3UpperTriangular(const float A[9], float Ai[9]) {
  float A4A8 = A[4] * A[8];
  float A0rec = IRec(A[0]);
  Ai[0] = A0rec;
  Ai[1] = -IDiv(A[1], A[0] * A[4]);
  Ai[2] = (IDiv(A[1] * A[5], A4A8) - IDiv(A[2], A[8])) * A0rec;
  Ai[3] = 0;
  Ai[4] = IRec(A[4]);
  Ai[5] = -IDiv(A[5], A4A8);
  Ai[6] = 0;
  Ai[7] = 0;
  Ai[8] = IRec(A[8]);
}
inline void IInvert3x3UpperTriangular(const int A[9], double Ai[9]) {
  double A4A8 = static_cast<double>(A[4] * A[8]);
  double A0rec = IRec(A[0]);
  Ai[0] = A0rec;
  Ai[1] = -IDiv(static_cast<double>(A[1]), static_cast<double>(A[0] * A[4]));
  Ai[2] =
      (IDiv(static_cast<double>(A[1] * A[5]), A4A8) - IDiv(A[2], A[8])) * A0rec;
  Ai[3] = 0;
  Ai[4] = IRec(A[4]);
  Ai[5] = -IDiv(static_cast<double>(A[5]), A4A8);
  Ai[6] = 0;
  Ai[7] = 0;
  Ai[8] = IRec(A[8]);
}

// Solve 2x2 linear equation system Ax=b using the Cramer's rule
inline void ISolve2x2(const double A[4], const double b[2], double x[2]) {
  double d, rec;
  d = IDeterminant2x2(A);
  rec = IRec(d);
  x[0] = rec * (A[3] * b[0] - A[1] * b[1]);
  x[1] = rec * (A[0] * b[1] - A[2] * b[0]);
}
inline void ISolve2x2(const float A[4], const float b[2], float x[2]) {
  float d, rec;
  d = IDeterminant2x2(A);
  rec = IRec(d);
  x[0] = rec * (A[3] * b[0] - A[1] * b[1]);
  x[1] = rec * (A[0] * b[1] - A[2] * b[0]);
}
// Solve 3x3 linear equation system Ax=b using the Cramer's rule
inline void ISolve3x3(const double A[9], const double b[3], double x[3]) {
  double d, rec, da0, da1, da2;
  d = IDeterminant3x3(A);
  rec = IRec(d);
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
inline void ISolve3x3(const float A[9], const float b[3], float x[3]) {
  float d, rec, da0, da1, da2;
  d = IDeterminant3x3(A);
  rec = IRec(d);
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

// Compute the transpose of a suqare nxn matrix inplace
template <typename T>
inline void ITranspose(T *A, int n) {
  for (int r = 0; r < n; ++r) {
    for (int c = r; c < n; ++c) {
      ISwap(A[r * n + c], A[c * n + r]);
    }
  }
}
// Compute the transpose of a mxn matrix A, At is nxm
template <typename T>
inline void ITranspose(const T *A, T *At, int m, int n) {
  for (int r = 0; r < m; ++r) {
    for (int c = 0; c < n; ++c) {
      At[c * m + r] = A[r * n + c];
    }
  }
}
template <typename T>
inline void ITranspose2x2(T A[4]) {
  ISwap(A[1], A[2]);
}
template <typename T>
inline void ITranspose2x2(const T A[4], T At[4]) {
  At[0] = A[0];
  At[1] = A[2];
  At[2] = A[1];
  At[3] = A[3];
}
template <typename T>
inline void ITranspose3x3(T A[9]) {
  ISwap(A[1], A[3]);
  ISwap(A[2], A[6]);
  ISwap(A[5], A[7]);
}
template <typename T>
inline void ITranspose3x3(const T A[9], T At[9]) {
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
inline void ITranspose4x4(T A[16]) {
  ISwap(A[1], A[4]);
  ISwap(A[2], A[8]);
  ISwap(A[6], A[9]);
  ISwap(A[3], A[12]);
  ISwap(A[7], A[13]);
  ISwap(A[11], A[14]);
}
template <typename T>
inline void ITranspose4x4(const T A[16], T At[16]) {
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

// Add a constant lambda (inplace) to the diagonal elements of a nxn square
//  * matrix A
template <typename T>
inline void IAugmentDiagonal(T *A, int n, const T lambda) {
  T *Ai;
  int i, ni = 0;
  for (i = 0; i < n; ++i, ni += n) {
    Ai = A + ni;
    Ai[i] += lambda;
  }
}

// Multiply m x n matrix A with n-dimensional vector x
template <typename T>
inline void IMultAx(const T *A, const T *x, T *Ax, int m, int n) {
  for (int i = 0; i < m; i++) {
    Ax[i] = IDot(A + i * n, x, n);
  }
}

// Multiply m x n matrix A's transpose (n x m matrix At) with m-dimensional
//  * vector x
template <typename T>
inline void IMultAtx(const T *A, const T *x, T *Atx, int m, int n) {
  T acc;
  const T *Ai = nullptr;
  for (int i = 0; i < n; i++) {
    Ai = A + i;
    acc = T(0.0);
    for (int j = 0; j < m; j++, Ai += n) {
      acc += (*Ai) * x[j];
    }
    Atx[i] = acc;
  }
}

// Multiply 1 x 3 matrix A with 3-dimensional vector x
template <typename T>
inline void IMultAx1x3(const T A[3], const T x[3], T Ax[1]) {
  Ax[0] = A[0] * x[0] + A[1] * x[1] + A[2] * x[2];
}
// Multiply 2 x 2 matrix A with 2-dimensional vector x
template <typename T>
inline void IMultAx2x2(const T A[4], const T x[2], T Ax[2]) {
  T x0, x1;
  x0 = x[0];
  x1 = x[1];
  Ax[0] = A[0] * x0 + A[1] * x1;
  Ax[1] = A[2] * x0 + A[3] * x1;
}
// Multiply 2 x 3 matrix A with 3-dimensional vector x
template <typename T>
inline void IMultAx2x3(const T A[6], const T x[3], T Ax[2]) {
  T x0, x1, x2;
  x0 = x[0];
  x1 = x[1];
  x2 = x[2];
  Ax[0] = A[0] * x0 + A[1] * x1 + A[2] * x2;
  Ax[1] = A[3] * x0 + A[4] * x1 + A[5] * x2;
}
// Multiply 3 x 3 matrix A with 3-dimensional vector x
template <typename T>
inline void IMultAx3x3(const T A[9], const T x[3], T Ax[3]) {
  T x0, x1, x2;
  x0 = x[0];
  x1 = x[1];
  x2 = x[2];
  Ax[0] = A[0] * x0 + A[1] * x1 + A[2] * x2;
  Ax[1] = A[3] * x0 + A[4] * x1 + A[5] * x2;
  Ax[2] = A[6] * x0 + A[7] * x1 + A[8] * x2;
}
// Multiply 3 x 3 matrix A^t with 3-dimensional vector x
template <typename T>
inline void IMultAtx3x3(const T A[9], const T x[3], T Atx[3]) {
  T x0, x1, x2;
  x0 = x[0];
  x1 = x[1];
  x2 = x[2];
  Atx[0] = A[0] * x0 + A[3] * x1 + A[6] * x2;
  Atx[1] = A[1] * x0 + A[4] * x1 + A[7] * x2;
  Atx[2] = A[2] * x0 + A[5] * x1 + A[8] * x2;
}
// Multiply 3 x 4 matrix A with 4-dimensional vector x
template <typename T>
inline void IMultAx3x4(const T A[12], const T x[4], T Ax[3]) {
  T x0, x1, x2, x3;
  x0 = x[0];
  x1 = x[1];
  x2 = x[2];
  x3 = x[3];
  Ax[0] = A[0] * x0 + A[1] * x1 + A[2] * x2 + A[3] * x3;
  Ax[1] = A[4] * x0 + A[5] * x1 + A[6] * x2 + A[7] * x3;
  Ax[2] = A[8] * x0 + A[9] * x1 + A[10] * x2 + A[11] * x3;
}
// Multiply 4 x 3 matrix A with 3-dimensional vector x
template <typename T>
inline void IMultAx4x3(const T A[12], const T x[3], T Ax[4]) {
  T x0, x1, x2;
  x0 = x[0];
  x1 = x[1];
  x2 = x[2];
  Ax[0] = A[0] * x0 + A[1] * x1 + A[2] * x2;
  Ax[1] = A[3] * x0 + A[4] * x1 + A[5] * x2;
  Ax[2] = A[6] * x0 + A[7] * x1 + A[8] * x2;
  Ax[3] = A[9] * x0 + A[10] * x1 + A[11] * x2;
}
// Multiply 4 x 3 matrix A^t with 3-dimensional vector x
template <typename T>
inline void IMultAtx4x3(const T A[12], const T x[3], T Atx[4]) {
  Atx[0] = A[0] * x[0] + A[4] * x[1] + A[8] * x[2];
  Atx[1] = A[1] * x[0] + A[5] * x[1] + A[9] * x[2];
  Atx[2] = A[2] * x[0] + A[6] * x[1] + A[10] * x[2];
  Atx[3] = A[3] * x[0] + A[7] * x[1] + A[11] * x[2];
}
// Multiply 4 x 4 matrix A with 4-dimensional vector x
template <typename T>
inline void IMultAx4x4(const T A[16], const T x[4], T Ax[4]) {
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
// Multiply m x n matrix A with n x o matrix B
template <typename T>
inline void IMultAB(const T *A, const T *B, T *AB, int m, int n, int o) {
  int in, io;
  T acc;
  for (int i = 0; i < m; i++) {
    in = i * n;
    io = i * o;
    for (int j = 0; j < o; j++) {
      acc = static_cast<T>(0.0);
      for (int k = 0; k < n; k++) {
        acc += A[in + k] * B[k * o + j];
      }
      AB[io + j] = acc;
    }
  }
}

// Multiply 3 x 1 matrix A with 1 x 3 matrix B
template <typename T>
inline void IMultAB3x1And1x3(const T A[3], const T B[3], T AB[9]) {
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

// Multiply 2 x 2 matrix A with 2 x 2 matrix B
template <typename T>
inline void IMultAB2x2And2x2(const T A[4], const T B[4], T AB[4]) {
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

// Multiply 2 x 3 matrix A with 3 x 2 matrix B
template <typename T>
inline void IMultAB2x3And3x2(const T A[6], const T B[6], T AB[4]) {
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

// Multiply 2 x 3 matrix A with 3 x 3 matrix B
template <typename T>
inline void IMultAB2x3And3x3(const T A[6], const T B[9], T AB[6]) {
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

// Multiply 2 x 3 matrix A with 3 x 4 matrix B
template <typename T>
inline void IMultAB2x3And3x4(const T A[6], const T B[12], T AB[8]) {
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

// Multiply 3 x 3 matrix A with 3 x 3 matrix B
template <typename T>
inline void IMultAB3x3And3x3(const T A[9], const T B[9], T AB[9]) {
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

// Multiply 4 x 4 matrix A with 4 x 4 matrix B
template <typename T>
inline void IMultAB4x4And4x4(const T A[16], const T B[16], T AB[16]) {
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

// Multiply 4 x 1 matrix A with 1 x 4 matrix B
template <typename T>
inline void IMultAB4x1And1x4(const T A[4], const T B[4], T AB[16]) {
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

// Multiply upper-triangular 3 x 3 matrix A with 3 x 3 matrix B
template <typename T>
inline void IMultAB3x3And3x3WAUpperTriangular(const T A[9], const T B[9],
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

// Multiply 3 x 3 matrix A with upper-triangular 3 x 3 matrix B
template <typename T>
inline void IMultAB3x3And3x3WBUpperTriangular(const T A[9], const T B[9],
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
inline void IMultAB3x3And3x4(const T A[9], const T B[12], T AB[12]) {
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
inline void IMultAB3x4And4x3(const T A[12], const T B[12], T AB[9]) {
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

// Multiply 3 x 3 matrix A with 3 x 3 matrix B^t
template <typename T>
inline void IMultABt3x3And3x3(const T A[9], const T B[9], T ABt[9]) {
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

// Multiply 2 x 3 matrix A with 3 x 2 matrix B^t
template <typename T>
inline void IMultABt2x3And2x3(const T A[6], const T B[6], T ABt[4]) {
  ABt[0] = A[0] * B[0] + A[1] * B[1] + A[2] * B[2];
  ABt[1] = A[0] * B[3] + A[1] * B[4] + A[2] * B[5];
  ABt[2] = A[3] * B[0] + A[4] * B[1] + A[5] * B[2];
  ABt[3] = A[3] * B[3] + A[4] * B[4] + A[5] * B[5];
}

// Multiply 4 x 4 matrix A with 4 x 3 matrix B^t
template <typename T>
inline void IMultABt4x4And3x4(const T A[16], const T B[12], T ABt[12]) {
  ABt[0] = IDot4(A, B);
  ABt[1] = IDot4(A, B + 4);
  ABt[2] = IDot4(A, B + 8);

  ABt[3] = IDot4(A + 4, B);
  ABt[4] = IDot4(A + 4, B + 4);
  ABt[5] = IDot4(A + 4, B + 8);

  ABt[6] = IDot4(A + 8, B);
  ABt[7] = IDot4(A + 8, B + 4);
  ABt[8] = IDot4(A + 8, B + 8);

  ABt[9] = IDot4(A + 12, B);
  ABt[10] = IDot4(A + 12, B + 4);
  ABt[11] = IDot4(A + 12, B + 8);
}

// Multiply m x n matrix A's transpose At (n x m) with A and get AtA (n x n)
template <typename T>
inline void IMultAtA(const T *A, T *AtA, int m, int n) {
  int i, j, k, ni;
  T acc;
  const T *Ai, *Aj;
  for (i = 0; i < n; ++i) {
    ni = n * i;
    for (j = 0; j < i; ++j) {
      AtA[ni + j] = AtA[j * n + i];
    }
    for (j = i; j < n; ++j) {
      acc = static_cast<T>(0.0);
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
inline void IMultAtA2x2(const T A[4], T AtA[4]) {
  AtA[0] = A[0] * A[0] + A[2] * A[2];
  AtA[1] = AtA[2] = A[0] * A[1] + A[2] * A[3];
  AtA[3] = A[1] * A[1] + A[3] * A[3];
}

template <typename T>
inline void IMultAtA2x2(const T *A, T AtA[4], int n) {
  T xx = static_cast<T>(0.0);
  T xy = static_cast<T>(0.0);
  T yy = static_cast<T>(0.0);
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
inline void IMultAtAnx2(const T *A, T *AtA, int n) {
  T xx = static_cast<T>(0.0);
  T xy = static_cast<T>(0.0);
  T yy = static_cast<T>(0.0);
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
inline void IMultAtA3x3(const T A[9], T AtA[9]) {
  AtA[0] = A[0] * A[0] + A[3] * A[3] + A[6] * A[6];
  AtA[1] = AtA[3] = A[0] * A[1] + A[3] * A[4] + A[6] * A[7];
  AtA[2] = AtA[6] = A[0] * A[2] + A[3] * A[5] + A[6] * A[8];
  AtA[4] = A[1] * A[1] + A[4] * A[4] + A[7] * A[7];
  AtA[5] = AtA[7] = A[1] * A[2] + A[4] * A[5] + A[7] * A[8];
  AtA[8] = A[2] * A[2] + A[5] * A[5] + A[8] * A[8];
}

template <typename T>
inline void IMultAtAnx3(const T *A, T AtA[9], int n) {
  T xx = static_cast<T>(0.0);
  T xy = static_cast<T>(0.0);
  T xz = static_cast<T>(0.0);
  T yy = static_cast<T>(0.0);
  T yz = static_cast<T>(0.0);
  T zz = static_cast<T>(0.0);
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
inline void IMultAtA4x4(const T A[16], T AtA[16]) {
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
inline void IMultAtB2x2And2x2(const T A[4], const T B[4], T AtB[4]) {
  AtB[0] = A[0] * B[0] + A[2] * B[2];
  AtB[1] = A[0] * B[1] + A[2] * B[3];
  AtB[2] = A[1] * B[0] + A[3] * B[2];
  AtB[3] = A[1] * B[1] + A[3] * B[3];
}

template <typename T>
inline void IMultAtB3x3And3x3(const T A[9], const T B[9], T AtB[9]) {
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
inline void IMultAAt2x3(const T A[6], T AAt[4]) {
  AAt[0] = A[0] * A[0] + A[1] * A[1] + A[2] * A[2];
  AAt[1] = AAt[2] = A[0] * A[3] + A[1] * A[4] + A[2] * A[5];
  AAt[3] = A[3] * A[3] + A[4] * A[4] + A[5] * A[5];
}

template <typename T>
inline void IMultAAt3x3(const T A[9], T AAt[9]) {
  AAt[0] = (A[0] * A[0] + A[1] * A[1] + A[2] * A[2]);
  AAt[1] = AAt[3] = (A[0] * A[3] + A[1] * A[4] + A[2] * A[5]);
  AAt[2] = AAt[6] = (A[0] * A[6] + A[1] * A[7] + A[2] * A[8]);
  AAt[4] = (A[3] * A[3] + A[4] * A[4] + A[5] * A[5]);
  AAt[5] = AAt[7] = (A[3] * A[6] + A[4] * A[7] + A[5] * A[8]);
  AAt[8] = (A[6] * A[6] + A[7] * A[7] + A[8] * A[8]);
}

template <typename T>
inline void IMultAAt4x1(const T A[4], T AAt[16]) {
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
inline void IMultABC3x3And3x3And3x3(const T A[9], const T B[9], const T C[9],
                                    T ABC[9]) {
  T BC[9];
  IMultAB3x3And3x3(B, C, BC);
  IMultAB3x3And3x3(A, BC, ABC);
}

template <typename T>
inline void IMultAtBC3x3And3x3And3x3(const T A[9], const T B[9], const T C[9],
                                     T AtBC[9]) {
  T BC[9];
  IMultAB3x3And3x3(B, C, BC);
  IMultAtB3x3And3x3(A, BC, AtBC);
}

template <typename T>
inline void IMultAB4x4WABlockDiagonal(const T A1[4], const T A2[4],
                                      const T B[16], T AB[16]) {
  // A = A1   0
  //       0   A2
  //   B = B1  B2
  //       B3  B4

  // A1B1
  AB[0] = A1[0] * B[0] + A1[1] * B[4];
  AB[1] = A1[0] * B[1] + A1[1] * B[5];
  AB[4] = A1[2] * B[0] + A1[3] * B[4];
  AB[5] = A1[2] * B[1] + A1[3] * B[5];

  // A1B2
  AB[2] = A1[0] * B[2] + A1[1] * B[6];
  AB[3] = A1[0] * B[3] + A1[1] * B[7];
  AB[6] = A1[2] * B[2] + A1[3] * B[6];
  AB[7] = A1[2] * B[3] + A1[3] * B[7];

  // A2B3
  AB[8] = A2[0] * B[8] + A2[1] * B[12];
  AB[9] = A2[0] * B[9] + A2[1] * B[13];
  AB[12] = A2[2] * B[8] + A2[3] * B[12];
  AB[13] = A2[2] * B[9] + A2[3] * B[13];

  // A2B4
  AB[10] = A2[0] * B[10] + A2[1] * B[14];
  AB[11] = A2[0] * B[11] + A2[1] * B[15];
  AB[14] = A2[2] * B[10] + A2[3] * B[14];
  AB[15] = A2[2] * B[11] + A2[3] * B[15];
}

// Swap rows r1 and r2 of a mxn matrix A
template <typename T>
inline void ISwapRows(T *A, int r1, int r2, int m, int n) {
  ISwap(A + r1 * n, A + r2 * n, n);
}
// Swap columns c1 and c2 of a mxn matrix A
template <typename T>
inline void ISwapCols(T *A, int c1, int c2, int m, int n) {
  T *ref = A;
  for (int i = 0; i < m; i++, ref += n) {
    ISwap(ref[c1], ref[c2]);
  }
}

// Swap interval [i1,i2) of rows r1 and r2 of a mxn matrix A
template <typename T>
inline void ISwapRowsInterval(T *A, int i1, int i2, int r1, int r2, int m,
                              int n) {
  int i_begin = IMax(0, i1);
  int i_end = IMin(n, i2);
  ISwap(A + r1 * n + i_begin, A + r2 * n + i_begin, (i_end - i_begin));
}
// Swap interval [i1,i2) of columns c1 and c2 of a mxn matrix A
template <typename T>
inline void ISwapColsInterval(T *A, int i1, int i2, int c1, int c2, int m,
                              int n) {
  int i_begin = IMax(0, i1);
  int i_end = IMin(m, i2);
  T *ref = A + i_begin * n;
  for (int i = i_begin; i < i_end; i++, ref += n) {
    ISwap(ref[c1], ref[c2]);
  }
}

// Shift array elements in place so that {x1, y1, ?, x2, y2, ?, x3, y3, ? ...
// xn,
// yn, ?} becomes {x1, y1, x2, y2, x3, y3, ..., xn, yn, ?, ?, ?, ...?} after
// shifting
// The size of input array A is n*3
template <typename T>
inline void IShiftHomogeneous3(T *A, int n) {
  if (n <= 1) {
    return;
  }
  int i, nm1 = n - 1;
  int dst = 2;
  int src = 3;
  for (i = 0; i < nm1; ++i) {
    ISwap(A[dst], A[src]);
    ISwap(A[dst + 1], A[src + 1]);
    dst += 2;
    src += 3;
  }
}

// Shift array elements in place so that {x1, y1, z1, ?, x2, y2, z2, ?, x3, y3,
// z3, ? ... xn, yn, zn, ?} becomes {x1, y1, z1, x2, y2, z2, x3, y3, z3..., xn,
// yn,
// zn, ?, ?, ?, ...?} after shifting
// The size of input array A is n*4
template <typename T>
inline void IShiftHomogeneous4(T *A, int n) {
  if (n <= 1) {
    return;
  }
  int i, nm1 = n - 1;
  int dst = 3;
  int src = 4;
  for (i = 0; i < nm1; ++i) {
    ISwap(A[dst], A[src]);
    ISwap(A[dst + 1], A[src + 1]);
    ISwap(A[dst + 2], A[src + 2]);
    dst += 3;
    src += 4;
  }
}

}  // namespace common
}  // namespace perception
}  // namespace apollo
