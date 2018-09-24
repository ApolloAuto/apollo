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

#include "i_constant.h"

#include <math.h>
#include <string.h>

namespace idl {
/*Compute abs(a)*/
inline float i_abs(float a) { return a < 0.f ? -a : a; }
inline int i_abs(int a) { return a < 0 ? -a : a; }
inline double i_abs(double a) { return a < 0.0 ? -a : a; }

/*Compute a/b, should not template this function, i_rec(int, int) should return
 * double*/
inline float i_div(float a, float b) { return ((b != 0.f) ? (a / b) : 1.0f); }
inline float i_div(float a, int b) { return ((b != 0) ? (a / b) : 1.0f); }
inline float i_div(float a, unsigned int b) {
  float result = 1.0f;
  if (b != 0) {
    result = a / b;
  }
  return result;
}
inline double i_div(int a, int b) { return ((b != 0) ? ((double)a / b) : 1.0); }
inline double i_div(unsigned int a, unsigned int b) {
  double result = 1.0;
  if (b != 0) {
    result = (double)a / b;
  }
  return result;
}
inline double i_div(double a, double b) { return ((b != 0.0) ? (a / b) : 1.0); }
inline double i_div(double a, int b) { return ((b != 0) ? (a / b) : 1.0); }
inline double i_div(double a, unsigned int b) {
  double result = 1.0;
  if (b != 0) {
    result = a / b;
  }
  return result;
}

/*Compute 1/a, should not template this function, i_rec(int) should return
 * double*/
inline float i_rec(float a) { return ((a != 0.0f) ? ((1.0f) / a) : 1.0f); }
inline double i_rec(int a) { return ((a != 0) ? ((1.0) / a) : 1.0); }
inline double i_rec(unsigned int a) { return ((a != 0) ? ((1.0) / a) : 1.0); }
inline double i_rec(double a) { return ((a != 0.0) ? ((1.0) / a) : 1.0); }

/*Compute sqrt(a), should not template this function, i_sqrt(int) should return
 * double*/
inline float i_sqrt(float a) { return (a >= 0.0f) ? (sqrtf(a)) : 0.0f; }
inline double i_sqrt(int a) { return (a > 0) ? (sqrt((double)a)) : 0.0; }
inline double i_sqrt(unsigned int a) {
  return (a > 0) ? (sqrt((double)a)) : 0.0;
}
inline double i_sqrt(double a) { return (a >= 0.0) ? (sqrt(a)) : 0.0; }

/*Compute cubic root of a, should not template this function, i_cbrt(int) should
 * return double*/
inline float i_cbrt(float a) {
  return (a >= 0.f) ? powf(a, 1.f / 3) : -powf(-a, 1.f / 3);
}
inline double i_cbrt(int a) {
  return (a >= 0) ? pow((double)a, 1.0 / 3) : -pow(-(double)a, 1.0 / 3);
}
inline double i_cbrt(unsigned int a) { return pow((double)a, 1.0 / 3); }
inline double i_cbrt(double a) {
  return (a >= 0.0) ? pow(a, 1.0 / 3) : -pow(-a, 1.0 / 3);
}

/*Compute a^2, should not template this function, i_sqr(char) and i_sqr(unsigned
 * char) should return int*/
inline float i_sqr(float a) { return (a * a); }
inline int i_sqr(int a) { return (a * a); }
inline unsigned int i_sqr(unsigned int a) { return (a * a); }
inline double i_sqr(double a) { return (a * a); }
inline int i_sqr(char a) { return ((int)a * (int)a); }
inline int i_sqr(unsigned char a) { return ((int)a * (int)a); }

/*Compute a^3, should not template this function, i_cub(char) and i_cub(unsigned
 * char) should return int*/
inline float i_cub(float a) { return (a * a * a); }
inline int i_cub(int a) { return (a * a * a); }
inline unsigned int i_cub(unsigned int a) { return (a * a * a); }
inline double i_cub(double a) { return (a * a * a); }
inline int i_cub(char a) { return ((int)a * (int)a * (int)a); }
inline int i_cub(unsigned char a) { return ((int)a * (int)a * (int)a); }

/*Compute log(x)*/
inline float i_log(float x) { return ((x > 0.f) ? logf(x) : 0.f); }
inline double i_log(int x) { return ((x > 0) ? log((double)x) : 0.0); }
inline double i_log(unsigned int x) { return ((x > 0) ? log((double)x) : 0.0); }
inline double i_log(double x) { return ((x > 0.0) ? log(x) : 0.0); }

/*Compute exp(x)*/
inline float i_exp(float x) { return (expf(x)); }
inline double i_exp(int x) { return exp((double)x); }
inline double i_exp(unsigned int x) { return exp((double)x); }
inline double i_exp(double x) { return exp(x); }

/*Compute pow(a, b) => a^b*/
inline float i_pow(float a, float b) { return powf(a, b); }
inline float i_pow(float a, int b) { return powf(a, (float)b); }
inline double i_pow(int a, int b) { return pow((double)a, (double)b); }
inline double i_pow(unsigned int a, unsigned int b) {
  return pow((double)a, (double)b);
}
inline double i_pow(double a, double b) { return pow(a, b); }
inline double i_pow(double a, int b) { return pow(a, (double)b); }

/*Compute min(a,b)*/
template <typename T>
inline T i_min(T a, T b) {
  return ((a <= b) ? a : b);
}

/*Compute max(a,b)*/
template <typename T>
inline T i_max(T a, T b) {
  return ((a >= b) ? a : b);
}

/*Compute the average of a and b*/
template <typename T>
inline T i_average(T a, T b) {
  return (a + b) / 2;
}

/*Compute the sign of a, return:
 1 if a>0
 -1 if a<0
 0 if a==0*/
template <typename T>
inline T i_sign(T a) {
  if (a > T(0.0))
    return (T(1.0));
  else if (a < T(0.0))
    return (T(-1.0));
  else
    return (T(0.0));
}

/*Compute the sign of a, return:
1 if a>=0
-1 if a<0*/
template <typename T>
inline T i_sign_never_zero(T a) {
  if (a >= T(0.0))
    return (T(1.0));
  else
    return (T(-1.0));
}

/*Round a to the nearest integer*/
inline int i_round(int a) { return (a); }
inline int i_round(float a) {
  return ((a >= 0.f) ? ((int)(a + 0.5f)) : ((int)(a - 0.5f)));
}
inline int i_round(double a) {
  return ((a >= 0.0) ? ((int)(a + 0.5)) : ((int)(a - 0.5)));
}

/*Rounds a upward, returning the smallest integral value that is not less than
 * a*/
inline int i_ceil(int a) { return (a); }
inline int i_ceil(float a) { return (int)(ceilf(a)); }
inline int i_ceil(double a) { return (int)(ceil(a)); }

/*Trigonometric functions*/
inline float i_sin(float alpha) { return sinf(alpha); }
inline double i_sin(double alpha) { return sin(alpha); }
inline float i_cos(float alpha) { return cosf(alpha); }
inline double i_cos(double alpha) { return cos(alpha); }
inline float i_tan(float alpha) { return tanf(alpha); }
inline double i_tan(double alpha) { return tan(alpha); }

inline float i_asin(float alpha) {
  if (alpha >= 1.f) return Constant<float>::HALF_PI();
  if (alpha < -1.f) return -Constant<float>::HALF_PI();
  return asinf(alpha);
}
inline double i_asin(double alpha) {
  if (alpha >= 1.0) return Constant<double>::HALF_PI();
  if (alpha < -1.0) return -Constant<double>::HALF_PI();
  return asin(alpha);
}
inline float i_acos(float alpha) {
  if (alpha >= 1.f) return 0.f;
  if (alpha < -1.f) return Constant<float>::PI();
  return acosf(alpha);
}
inline double i_acos(double alpha) {
  if (alpha >= 1.0) return 0.0;
  if (alpha < -1.0) return Constant<double>::PI();
  return acos(alpha);
}
inline float i_atan2(float y, float x) { return atan2f(y, x); }
inline double i_atan2(double y, double x) { return atan2(y, x); }

inline float i_radians_to_degree(float r) {
  return (r * Constant<float>::RADIAN_TO_DEGREE());
}
inline double i_radians_to_degree(double r) {
  return (r * Constant<double>::RADIAN_TO_DEGREE());
}
inline float i_degree_to_radians(float d) {
  return (d * Constant<float>::DEGREE_TO_RADIAN());
}
inline double i_degree_to_radians(double d) {
  return (d * Constant<double>::DEGREE_TO_RADIAN());
}

/*Compute the Hamming distance between two (unsigned) integers (considered as
 * binary values, that is, as sequences of bits)*/
inline unsigned int i_hamming(unsigned int a, unsigned int b) {
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

inline unsigned int i_hamming_lut(unsigned int a, unsigned int b) {
  unsigned int distance = 0;
  unsigned int val = a ^ b;  // XOR
  unsigned char *p = (unsigned char *)&val;
  /*count the total # of "1s" in a xor b*/
  distance = i_UByteBitCountLut[p[0]] + i_UByteBitCountLut[p[1]] +
             i_UByteBitCountLut[p[2]] + i_UByteBitCountLut[p[3]];
  return distance; /*Return the number of different bits*/
}

/*Swap of numbers*/
template <typename T>
inline void i_swap(T &a, T &b) {
  T temp;
  temp = a;
  a = b;
  b = temp;
}
template <typename T>
inline void i_swap(T *a, T *b, int n) {
  for (int i = 0; i < n; i++) i_swap(a[i], b[i]);
}
template <typename T>
inline void i_swap2(T *a, T *b) {
  i_swap(a[0], b[0]);
  i_swap(a[1], b[1]);
}
template <typename T>
inline void i_swap3(T *a, T *b) {
  i_swap(a[0], b[0]);
  i_swap(a[1], b[1]);
  i_swap(a[2], b[2]);
}
template <typename T>
inline void i_swap4(T *a, T *b) {
  i_swap(a[0], b[0]);
  i_swap(a[1], b[1]);
  i_swap(a[2], b[2]);
  i_swap(a[3], b[3]);
}

/*Return:
      min_val if a <= min_val
      max_val if a >= max_val
      a       else*/
template <typename T>
inline T i_interval(T a, T min_val, T max_val) {
  if (a <= min_val) return (min_val);
  if (a >= max_val) return (max_val);
  return (a);
}

/*Return:
min_val   if a <= min_val
max_val-1 if a >= max_val
a         else*/
template <typename T>
inline T i_interval_halfopen(T a, T min_val, T max_val) {
  if (a <= min_val) return (min_val);
  if (a >= max_val) return (max_val - 1);
  return (a);
}

/*Make p[i] to be the reference of a's ith row, where a is a mxn matrix*/
template <typename T>
inline void i_make_reference(T *a, T **p, int m, int n) {
  for (int i = 0; i < m; i++) {
    p[i] = &a[i * n];
  }
}
template <typename T>
inline void i_make_reference_2x2(T a[4], T *p[2]) {
  p[0] = &a[0];
  p[1] = &a[2];
}
template <typename T>
inline void i_make_reference_3x3(T a[9], T *p[3]) {
  p[0] = &a[0];
  p[1] = &a[3];
  p[2] = &a[6];
}
template <typename T>
inline void i_make_reference_4x4(T a[16], T *p[4]) {
  p[0] = &a[0];
  p[1] = &a[4];
  p[2] = &a[8];
  p[3] = &a[12];
}
template <typename T>
inline void i_make_reference_4x9(T a[36], T *p[4]) {
  p[0] = &a[0];
  p[1] = &a[9];
  p[2] = &a[18];
  p[3] = &a[27];
}
template <typename T>
inline void i_make_reference_5x9(T a[45], T *p[5]) {
  p[0] = &a[0];
  p[1] = &a[9];
  p[2] = &a[18];
  p[3] = &a[27];
  p[4] = &a[36];
}
template <typename T>
inline void i_make_reference_9x9(T a[81], T *p[9]) {
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
inline void i_make_reference_12x12(T a[144], T *p[12]) {
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
inline void i_make_const_reference(const T *a, const T **p, int m, int n) {
  for (int i = 0; i < m; i++) {
    p[i] = &a[i * n];
  }
}
template <typename T>
inline void i_make_const_reference_2x2(const T a[4], const T *p[2]) {
  p[0] = &a[0];
  p[1] = &a[2];
}
template <typename T>
inline void i_make_const_reference_3x3(const T a[9], const T *p[3]) {
  p[0] = &a[0];
  p[1] = &a[3];
  p[2] = &a[6];
}
template <typename T>
inline void i_make_const_reference_4x4(const T a[16], const T *p[4]) {
  p[0] = &a[0];
  p[1] = &a[4];
  p[2] = &a[8];
  p[3] = &a[12];
}
template <typename T>
inline void i_make_const_reference_4x9(const T a[36], const T *p[4]) {
  p[0] = &a[0];
  p[1] = &a[9];
  p[2] = &a[18];
  p[3] = &a[27];
}
template <typename T>
inline void i_make_const_reference_5x9(const T a[45], const T *p[5]) {
  p[0] = &a[0];
  p[1] = &a[9];
  p[2] = &a[18];
  p[3] = &a[27];
  p[4] = &a[36];
}
template <typename T>
inline void i_make_const_reference_9x9(const T a[81], const T *p[9]) {
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
inline void i_make_const_reference_12x12(const T a[144], const T *p[12]) {
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

/*Create an array of pointers (in p) to the l*m*n memory area a*/
template <typename T>
inline void i_make_reference(T *a, T ***p, int l, int m, int n) {
  T *temp = a;
  for (int i = 0; i < l; i++) {
    for (int j = 0; j < m; j++) {
      p[i][j] = temp;
      temp += n;
    }
  }
}

/*Assign a[i] = i to a n-dimensional vector a*/
template <typename T>
inline void i_ramp(T *a, int n) {
  for (int i = 0; i < n; i++) {
    a[i] = (T)i;
  }
}

/*Construct the square 2D Gaussian Kernel (nxn) given Kernel width "n" and sigma
 * value "sigma"*/
inline void i_gaussian_2d(float *kernel, int n, const float sigma) {
  int r, c, i = 0;
  const float cen = ((float)(n - 1)) / 2;
  const float nf =
      i_div(0.5f, (sigma * sigma));  // normalization factor = 1/(2sigma*sigma)
  float dr, drsqr, dc, dcsqr, v, ksum = 0.f;
  // pre-compute filter
  for (r = 0; r < n; ++r) {
    dr = (float)r - cen;
    drsqr = i_sqr(dr);
    for (c = 0; c < n; ++c) {
      dc = (float)c - cen;
      dcsqr = i_sqr(dc);
      v = i_exp(-(drsqr + dcsqr) * nf);
      ksum += v;
      kernel[i++] = v;
    }
  }
  // normalize the kernel
  v = i_div(1.0f, ksum);
  for (i = 0; i < n * n; ++i) {
    kernel[i] *= v;
  }
}

inline void i_gaussian_2d(double *kernel, int n, const double sigma) {
  int r, c, i = 0;
  const double cen = ((double)(n - 1)) / 2;
  const double nf =
      i_div(0.5, (sigma * sigma));  // normalization factor = 1/(2sigma*sigma)
  double dr, drsqr, dc, dcsqr, v, ksum = 0.0;
  // pre-compute filter
  for (r = 0; r < n; ++r) {
    dr = (double)r - cen;
    drsqr = i_sqr(dr);
    for (c = 0; c < n; ++c) {
      dc = (double)c - cen;
      dcsqr = i_sqr(dc);
      v = i_exp(-(drsqr + dcsqr) * nf);
      ksum += v;
      kernel[i++] = v;
    }
  }
  // normalize the kernel
  v = i_div(1.0, ksum);
  for (i = 0; i < n * n; ++i) {
    kernel[i] *= v;
  }
}

/*Move a into b, without necessarily preserving the value of a. This function is
 * specialized for types that are expensive to copy*/
template <typename T>
inline void i_move(T &a, T &b) {
  b = a;
}

/*Check if a point x is within the 1D rectangle bounding box defined by range
 * [start, start + length)*/
template <typename T>
inline bool i_within_1d(const T x, const T start, const T length) {
  return (x >= start && x < (length + start));
}

/*Check if a point p(x,y) is within the 2D rectangle bounding box defined by
 * [x_upper_left, y_upper_left, width, height], where width and height are width
 * and height of the bounding box*/
template <typename T>
inline bool i_within_2d(const T p[2], const T x_upper_left,
                        const T y_upper_left, const T width, const T height) {
  return (p[0] >= x_upper_left && p[1] >= y_upper_left &&
          p[0] < width + x_upper_left && p[1] < height + y_upper_left);
}
/*Check if a point (x,y) is within the 2D rectangle bounding box defined by
 * [x_upper_left, y_upper_left, width, height], where width and height are width
 * and height of the bounding box*/
template <typename T>
inline bool i_within_2d(const T x, const T y, const T x_upper_left,
                        const T y_upper_left, const T width, const T height) {
  return (x >= x_upper_left && y >= y_upper_left && x < width + x_upper_left &&
          y < height + y_upper_left);
}

} /*namespace idl*/