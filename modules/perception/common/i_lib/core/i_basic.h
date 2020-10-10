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

#include <cmath>
#include <cstring>

#include "modules/perception/common/i_lib/core/i_constant.h"

namespace apollo {
namespace perception {
namespace common {

// Compute abs(a)
inline float IAbs(float a) { return a < 0.f ? -a : a; }
inline int IAbs(int a) { return a < 0 ? -a : a; }
inline double IAbs(double a) { return a < 0.0 ? -a : a; }

// Compute a/b, should not template this function, IRec(int, int) should return
//  * double
inline float IDiv(float a, float b) { return ((b != 0.f) ? (a / b) : 1.0f); }
inline float IDiv(float a, int b) {
  return ((b != 0) ? (a / static_cast<float>(b)) : 1.0f);
}
inline float IDiv(float a, unsigned int b) {
  float result = 1.0f;
  if (b != 0) {
    result = a / static_cast<float>(b);
  }
  return result;
}
inline double IDiv(int a, int b) {
  return ((b != 0) ? (static_cast<double>(a) / b) : 1.0);
}
inline double IDiv(unsigned int a, unsigned int b) {
  double result = 1.0;
  if (b != 0) {
    result = static_cast<double>(a) / b;
  }
  return result;
}
inline double IDiv(double a, double b) { return ((b != 0.0) ? (a / b) : 1.0); }
inline double IDiv(double a, int b) { return ((b != 0) ? (a / b) : 1.0); }
inline double IDiv(double a, unsigned int b) {
  double result = 1.0;
  if (b != 0) {
    result = a / b;
  }
  return result;
}

// Compute 1/a, should not template this function,
// IRecstatic_cast<int> should return
//  * double
inline float IRec(float a) { return ((a != 0.0f) ? ((1.0f) / a) : 1.0f); }
inline double IRec(int a) { return ((a != 0) ? ((1.0) / a) : 1.0); }
inline double IRec(unsigned int a) { return ((a != 0) ? ((1.0) / a) : 1.0); }
inline double IRec(double a) { return ((a != 0.0) ? ((1.0) / a) : 1.0); }

// Compute sqrt(a), should not template this function, ISqrt(int) should return
//  * double
inline float ISqrt(float a) { return (a >= 0.0f) ? (sqrtf(a)) : 0.0f; }
inline double ISqrt(int a) {
  return (a > 0) ? (sqrt(static_cast<double>(a))) : 0.0;
}
inline double ISqrt(unsigned int a) {
  return (a > 0) ? (sqrt(static_cast<double>(a))) : 0.0;
}
inline double ISqrt(double a) { return (a >= 0.0) ? (sqrt(a)) : 0.0; }

// Compute cubic root of a, should not template this function, ICbrt(int) should
//  * return double
inline float ICbrt(float a) {
  return (a >= 0.f) ? powf(a, 1.f / 3) : -powf(-a, 1.f / 3);
}
inline double ICbrt(int a) {
  return (a >= 0) ? pow(static_cast<double>(a), 1.0 / 3)
                  : -pow(-static_cast<double>(a), 1.0 / 3);
}
inline double ICbrt(unsigned int a) {
  return pow(static_cast<double>(a), 1.0 / 3);
}
inline double ICbrt(double a) {
  return (a >= 0.0) ? pow(a, 1.0 / 3) : -pow(-a, 1.0 / 3);
}

// Compute a^2, should not template this function, ISqr(char) and ISqr(unsigned
//  * char) should return int
inline float ISqr(float a) { return (a * a); }
inline int ISqr(int a) { return (a * a); }
inline unsigned int ISqr(unsigned int a) { return (a * a); }
inline double ISqr(double a) { return (a * a); }
inline int ISqr(char a) { return (static_cast<int>(a) * static_cast<int>(a)); }
inline int ISqr(unsigned char a) {
  return (static_cast<int>(a) * static_cast<int>(a));
}

// Compute a^3, should not template this function, ICub(char) and ICub(unsigned
//  * char) should return int
inline float ICub(float a) { return (a * a * a); }
inline int ICub(int a) { return (a * a * a); }
inline unsigned int ICub(unsigned int a) { return (a * a * a); }
inline double ICub(double a) { return (a * a * a); }
inline int ICub(char a) {
  return (static_cast<int>(a) * static_cast<int>(a) * static_cast<int>(a));
}
inline int ICub(unsigned char a) {
  return (static_cast<int>(a) * static_cast<int>(a) * static_cast<int>(a));
}

// Compute log(x)
inline float ILog(float x) { return ((x > 0.f) ? logf(x) : 0.f); }
inline double ILog(int x) {
  return ((x > 0) ? log(static_cast<double>(x)) : 0.0);
}
inline double ILog(unsigned int x) {
  return ((x > 0) ? log(static_cast<double>(x)) : 0.0);
}
inline double ILog(double x) { return ((x > 0.0) ? log(x) : 0.0); }

// Compute exp(x)
inline float IExp(float x) { return (expf(x)); }
inline double IExp(int x) { return exp(static_cast<double>(x)); }
inline double IExp(unsigned int x) { return exp(static_cast<double>(x)); }
inline double IExp(double x) { return exp(x); }

// Compute pow(a, b) => a^b
inline float IPow(float a, float b) { return powf(a, b); }
inline float IPow(float a, int b) { return powf(a, static_cast<float>(b)); }
inline double IPow(int a, int b) {
  return pow(static_cast<double>(a), static_cast<double>(b));
}
inline double IPow(unsigned int a, unsigned int b) {
  return pow(static_cast<double>(a), static_cast<double>(b));
}
inline double IPow(double a, double b) { return pow(a, b); }
inline double IPow(double a, int b) { return pow(a, static_cast<double>(b)); }

// Compute min(a,b)
template <typename T>
inline T IMin(T a, T b) {
  return ((a <= b) ? a : b);
}

// Compute max(a,b)
template <typename T>
inline T IMax(T a, T b) {
  return ((a >= b) ? a : b);
}

// Compute the average of a and b
template <typename T>
inline T IAverage(T a, T b) {
  return (a + b) / 2;
}

// Compute the sign of a, return:
// 1 if a>0
// -1 if a<0
// 0 if a==0
template <typename T>
inline T ISign(T a) {
  if (a > T(0.0))
    return (T(1.0));
  else if (a < T(0.0))
    return (T(-1.0));
  else
    return (T(0.0));
}

// Compute the sign of a, return:
// 1 if a>=0
// -1 if a<0
template <typename T>
inline T ISignNeverZero(T a) {
  if (a >= T(0.0))
    return (T(1.0));
  else
    return (T(-1.0));
}

// Round a to the nearest integer
inline int IRound(int a) { return (a); }
inline int IRound(float a) {
  return ((a >= 0.f) ? (static_cast<int>(a + 0.5f))
                     : (static_cast<int>(a - 0.5f)));
}
inline int IRound(double a) {
  return ((a >= 0.0) ? (static_cast<int>(a + 0.5))
                     : (static_cast<int>(a - 0.5)));
}

// Rounds an upward, returning the smallest integral value that is not less than
//  * a
inline int ICeil(int a) { return (a); }
inline int ICeil(float a) { return static_cast<int>(ceilf(a)); }
inline int ICeil(double a) { return static_cast<int>(ceil(a)); }

// Trigonometric functions
inline float ISin(float alpha) { return sinf(alpha); }
inline double ISin(double alpha) { return sin(alpha); }
inline float ICos(float alpha) { return cosf(alpha); }
inline double ICos(double alpha) { return cos(alpha); }
inline float ITan(float alpha) { return tanf(alpha); }
inline double ITan(double alpha) { return tan(alpha); }

inline float IAsin(float alpha) {
  if (alpha >= 1.f) {
    return Constant<float>::HALF_PI();
  }
  if (alpha < -1.f) {
    return -Constant<float>::HALF_PI();
  }
  return asinf(alpha);
}
inline double IAsin(double alpha) {
  if (alpha >= 1.0) {
    return Constant<double>::HALF_PI();
  }
  if (alpha < -1.0) {
    return -Constant<double>::HALF_PI();
  }
  return asin(alpha);
}
inline float IAcos(float alpha) {
  if (alpha >= 1.f) {
    return 0.f;
  }
  if (alpha < -1.f) {
    return Constant<float>::PI();
  }
  return acosf(alpha);
}
inline double IAcos(double alpha) {
  if (alpha >= 1.0) {
    return 0.0;
  }
  if (alpha < -1.0) {
    return Constant<double>::PI();
  }
  return acos(alpha);
}
inline float IAtan2(float y, float x) { return atan2f(y, x); }
inline double IAtan2(double y, double x) { return atan2(y, x); }

inline float IRadiansToDegree(float r) {
  return (r * Constant<float>::RADIAN_TO_DEGREE());
}
inline double IRadiansToDegree(double r) {
  return (r * Constant<double>::RADIAN_TO_DEGREE());
}
inline float IDegreeToRadians(float d) {
  return (d * Constant<float>::DEGREE_TO_RADIAN());
}
inline double IDegreeToRadians(double d) {
  return (d * Constant<double>::DEGREE_TO_RADIAN());
}

// Compute the Hamming distance between two (unsigned) integers (considered as
//  * binary values, that is, as sequences of bits)
inline unsigned int IHamming(unsigned int a, unsigned int b) {
  unsigned int distance = 0;
  unsigned int val = a ^ b;  // XOR
                             // Count the number of bits set
  while (val != 0) {
    // A bit is set, so increment the count and clear the bit
    distance++;
    val &= val - 1;
  }
  return distance;  // Return the number of different bits
}

inline unsigned int IHammingLut(unsigned int a, unsigned int b) {
  unsigned int distance = 0;
  unsigned int val = a ^ b;  // XOR
  unsigned char *p = (unsigned char *)&val;
  // count the total # of "1s" in a xor b
  distance = kIUByteBitCountLut[p[0]] + kIUByteBitCountLut[p[1]] +
             kIUByteBitCountLut[p[2]] + kIUByteBitCountLut[p[3]];
  return distance;  // Return the number of different bits
}

// Swap of numbers
template <typename T>
inline void ISwap(T &a, T &b) {
  T temp;
  temp = a;
  a = b;
  b = temp;
}
template <typename T>
inline void ISwap(T *a, T *b, int n) {
  for (int i = 0; i < n; i++) {
    ISwap(a[i], b[i]);
  }
}
template <typename T>
inline void ISwap2(T *a, T *b) {
  ISwap(a[0], b[0]);
  ISwap(a[1], b[1]);
}
template <typename T>
inline void ISwap3(T *a, T *b) {
  ISwap(a[0], b[0]);
  ISwap(a[1], b[1]);
  ISwap(a[2], b[2]);
}
template <typename T>
inline void ISwap4(T *a, T *b) {
  ISwap(a[0], b[0]);
  ISwap(a[1], b[1]);
  ISwap(a[2], b[2]);
  ISwap(a[3], b[3]);
}

// Return:
// min_val if a <= min_val
// max_val if a >= max_val
template <typename T>
inline T IInterval(T a, T min_val, T max_val) {
  if (a <= min_val) {
    return (min_val);
  }
  if (a >= max_val) {
    return (max_val);
  }
  return (a);
}

template <typename T>
inline T IIntervalHalfopen(T a, T min_val, T max_val) {
  if (a <= min_val) {
    return (min_val);
  }
  if (a >= max_val) {
    return (max_val - 1);
  }
  return (a);
}

// Make p[i] to be the reference of a's ith row, where a is a mxn matrix
template <typename T>
inline void IMakeReference(T *a, T **p, int m, int n) {
  for (int i = 0; i < m; i++) {
    p[i] = &a[i * n];
  }
}
template <typename T>
inline void IMakeReference2x2(T a[4], T *p[2]) {
  p[0] = &a[0];
  p[1] = &a[2];
}
template <typename T>
inline void IMakeReference3x3(T a[9], T *p[3]) {
  p[0] = &a[0];
  p[1] = &a[3];
  p[2] = &a[6];
}
template <typename T>
inline void IMakeReference4x4(T a[16], T *p[4]) {
  p[0] = &a[0];
  p[1] = &a[4];
  p[2] = &a[8];
  p[3] = &a[12];
}
template <typename T>
inline void IMakeReference4x9(T a[36], T *p[4]) {
  p[0] = &a[0];
  p[1] = &a[9];
  p[2] = &a[18];
  p[3] = &a[27];
}
template <typename T>
inline void IMakeReference5x9(T a[45], T *p[5]) {
  p[0] = &a[0];
  p[1] = &a[9];
  p[2] = &a[18];
  p[3] = &a[27];
  p[4] = &a[36];
}
template <typename T>
inline void IMakeReference9x9(T a[81], T *p[9]) {
  p[0] = &a[0];
  p[1] = &a[9];
  p[2] = &a[18];
  p[3] = &a[27];
  p[4] = &a[36];
  p[5] = &a[45];
  p[6] = &a[54];
  p[7] = &a[63];
  p[8] = &a[72];
}
template <typename T>
inline void IMakeReference12x12(T a[144], T *p[12]) {
  p[0] = &a[0];
  p[1] = &a[12];
  p[2] = &a[24];
  p[3] = &a[36];
  p[4] = &a[48];
  p[5] = &a[60];
  p[6] = &a[72];
  p[7] = &a[84];
  p[8] = &a[96];
  p[9] = &a[108];
  p[10] = &a[120];
  p[11] = &a[132];
}

template <typename T>
inline void IMakeConstReference(const T *a, const T **p, int m, int n) {
  for (int i = 0; i < m; i++) {
    p[i] = &a[i * n];
  }
}
template <typename T>
inline void IMakeConstReference2x2(const T a[4], const T *p[2]) {
  p[0] = &a[0];
  p[1] = &a[2];
}
template <typename T>
inline void IMakeConstReference3x3(const T a[9], const T *p[3]) {
  p[0] = &a[0];
  p[1] = &a[3];
  p[2] = &a[6];
}
template <typename T>
inline void IMakeConstReference4x4(const T a[16], const T *p[4]) {
  p[0] = &a[0];
  p[1] = &a[4];
  p[2] = &a[8];
  p[3] = &a[12];
}
template <typename T>
inline void IMakeConstReference4x9(const T a[36], const T *p[4]) {
  p[0] = &a[0];
  p[1] = &a[9];
  p[2] = &a[18];
  p[3] = &a[27];
}
template <typename T>
inline void IMakeConstReference5x9(const T a[45], const T *p[5]) {
  p[0] = &a[0];
  p[1] = &a[9];
  p[2] = &a[18];
  p[3] = &a[27];
  p[4] = &a[36];
}
template <typename T>
inline void IMakeConstReference9x9(const T a[81], const T *p[9]) {
  p[0] = &a[0];
  p[1] = &a[9];
  p[2] = &a[18];
  p[3] = &a[27];
  p[4] = &a[36];
  p[5] = &a[45];
  p[6] = &a[54];
  p[7] = &a[63];
  p[8] = &a[72];
}
template <typename T>
inline void IMakeConstReference12x12(const T a[144], const T *p[12]) {
  p[0] = &a[0];
  p[1] = &a[12];
  p[2] = &a[24];
  p[3] = &a[36];
  p[4] = &a[48];
  p[5] = &a[60];
  p[6] = &a[72];
  p[7] = &a[84];
  p[8] = &a[96];
  p[9] = &a[108];
  p[10] = &a[120];
  p[11] = &a[132];
}

// Create an array of pointers (in p) to the l*m*n memory area a
template <typename T>
inline void IMakeReference(T *a, T ***p, int l, int m, int n) {
  T *temp = a;
  for (int i = 0; i < l; i++) {
    for (int j = 0; j < m; j++) {
      p[i][j] = temp;
      temp += n;
    }
  }
}

// Assign a[i] = i to a n-dimensional vector a
template <typename T>
inline void IRamp(T *a, int n) {
  for (int i = 0; i < n; i++) {
    a[i] = static_cast<T>(i);
  }
}

// Construct the square 2D Gaussian Kernel (nxn) given Kernel width "n" and
// sigma
//  * value "sigma"
inline void IGaussian2D(float *kernel, int n, const float sigma) {
  int r, c, i = 0;
  const float cen = (static_cast<float>(n - 1)) / 2;
  const float nf =
      IDiv(0.5f, (sigma * sigma));  // normalization factor = 1/(2sigma*sigma)
  float dr, drsqr, dc, dcsqr, v, ksum = 0.f;
  // pre-compute filter
  for (r = 0; r < n; ++r) {
    dr = static_cast<float>(r) - cen;
    drsqr = ISqr(dr);
    for (c = 0; c < n; ++c) {
      dc = static_cast<float>(c) - cen;
      dcsqr = ISqr(dc);
      v = IExp(-(drsqr + dcsqr) * nf);
      ksum += v;
      kernel[i++] = v;
    }
  }
  // normalize the kernel
  v = IDiv(1.0f, ksum);
  for (i = 0; i < n * n; ++i) {
    kernel[i] *= v;
  }
}

inline void IGaussian2D(double *kernel, int n, const double sigma) {
  int r, c, i = 0;
  const double cen = (static_cast<double>(n - 1)) / 2;
  const double nf =
      IDiv(0.5, (sigma * sigma));  // normalization factor = 1/(2sigma*sigma)
  double dr, drsqr, dc, dcsqr, v, ksum = 0.0;
  // pre-compute filter
  for (r = 0; r < n; ++r) {
    dr = static_cast<double>(r) - cen;
    drsqr = ISqr(dr);
    for (c = 0; c < n; ++c) {
      dc = static_cast<double>(c) - cen;
      dcsqr = ISqr(dc);
      v = IExp(-(drsqr + dcsqr) * nf);
      ksum += v;
      kernel[i++] = v;
    }
  }
  // normalize the kernel
  v = IDiv(1.0, ksum);
  for (i = 0; i < n * n; ++i) {
    kernel[i] *= v;
  }
}

// Move an into b, without necessarily preserving the value of a. This function
// is * specialized for types that are expensive to copy
template <typename T>
inline void IMove(const T &a, T *b) {
  *b = a;
}

// Check if a point x is within the 1D rectangle bounding box defined by range
//  * [start, start + length)
template <typename T>
inline bool IWithin1D(const T x, const T start, const T length) {
  return (x >= start && x < (length + start));
}

// Check if a point p(x,y) is within the 2D rectangle bounding box defined by
//  * [x_upper_left, y_upper_left, width, height], where width and height are
// width
//  * and height of the bounding box
template <typename T>
inline bool IWithin2D(const T p[2], const T x_upper_left, const T y_upper_left,
                      const T width, const T height) {
  return (p[0] >= x_upper_left && p[1] >= y_upper_left &&
          p[0] < width + x_upper_left && p[1] < height + y_upper_left);
}
// Check if a point (x,y) is within the 2D rectangle bounding box defined by
//  * [x_upper_left, y_upper_left, width, height], where width and height are
// width
//  * and height of the bounding box
template <typename T>
inline bool IWithin2D(const T x, const T y, const T x_upper_left,
                      const T y_upper_left, const T width, const T height) {
  return (x >= x_upper_left && y >= y_upper_left && x < width + x_upper_left &&
          y < height + y_upper_left);
}

}  // namespace common
}  // namespace perception
}  // namespace apollo
