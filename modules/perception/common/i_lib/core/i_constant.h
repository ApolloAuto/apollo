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

namespace idl {
template <typename T>
class Constant;

template <>
class Constant<char> {
 public:
  static const char MAX_VAL() {
    return (127);
  } /*the maximum value for type char*/
  static const char MIN_VAL() {
    return (-128);
  } /*the minimum value for type char*/
};

template <>
class Constant<unsigned char> {
 public:
  static const unsigned char MAX_VAL() {
    return (255);
  } /*the maximum value for type unsigned char*/
  static const unsigned char MIN_VAL() {
    return (0);
  } /*the minimum value for type unsigned char*/
};

template <>
class Constant<int> {
 public:
  static const int MAX_VAL() {
    return (2147483647);
  } /*the maximum value for type int*/
  static const int MIN_VAL() {
    return (-2147483647 - 1);
  } /*the minimum value for type int*/
};

template <>
class Constant<unsigned int> {
 public:
  static const unsigned int MAX_VAL() {
    return (4294967295);
  } /*the maximum value for type unsigned int*/
  static const unsigned int MIN_VAL() {
    return (0);
  } /*the minimum value for type unsigend int*/
};

template <>
class Constant<short> {
 public:
  static const int MAX_VAL() {
    return (32767);
  } /*the maximum value for type short*/
  static const int MIN_VAL() {
    return (-32768);
  } /*the minimum value for type short*/
};

template <>
class Constant<unsigned short> {
 public:
  static const int MAX_VAL() {
    return (65535);
  } /*the maximum value for type unsigned short*/
  static const int MIN_VAL() {
    return (0);
  } /*the minimum value for type unsigned short*/
};

template <>
class Constant<double> {
 public:
  static const double PI() { return 3.1415926535897932384626433832795; } /*pi*/
  static const double TWO_PI() {
    return 6.283185307179586476925286766559;
  } /*2pi*/
  static const double HALF_PI() {
    return 1.5707963267948966192313216916398;
  } /*pi over 2*/
  static const double REC_TWO_PI() {
    return 0.15915494309189533576888376337251;
  } /*reciprocal of 2pi*/
  static const double SQRT_2() {
    return 1.4142135623730950488016887242097;
  } /*square root of 2*/
  static const double REC_SQRT_2() {
    return 0.70710678118654752440084436210485;
  } /*reciprocal of square root of 2*/
  static const double SQRT_3() {
    return 1.7320508075688772935274463415059;
  } /*square root of 3*/
  static const double REC_SQRT_3() {
    return 0.57735026918962576450914878050196;
  } /*reciprocal of square root of 3*/
  static const double DEGREE_TO_RADIAN() {
    return 0.017453292519943295769236907684886;
  } /*2pi over 360*/
  static const double RADIAN_TO_DEGREE() {
    return 57.295779513082320876798154814105;
  } /*360 over 2pi*/
  static const double EPSILON() {
    return 1.11e-016;
  } /*machine epsilon gives an upper bound on the relative error due to rounding
       in floating point arithmetic*/
  static const double SQRT_EPSILON() {
    return 1.053e-08;
  } /*square root of epsilon*/
  static const double MAX_VAL() {
    return 1.79769e+308;
  } /*the maximum value for type double*/
  /*
  For double and floating-point types with denormalization, MIN_VAL() returns
  the minimum positive normalized value.
  MIN_VAL is only meaningful for bounded types and  for unbounded unsigned
  types, that is, types that represent an infinite set of negative values have
  no meaningful minimum.
  */
  static const double MIN_VAL() {
    return 2.22507e-308;
  } /*the minimum positive value for type double*/
  static const double LOWEST() {
    return -1.79769e+308;
  } /*the lowest finite value representable by the numeric type double*/
  static const double MIN_ABS_SAFE_DIVIDEND() {
    return 1e-8;
  } /*the minimum safe dividend when doing a divide operation, if abs(divident)
       < MIN_ABS_SAFE_DIVIDEND, it will be considered as division by zero*/
};

template <>
class Constant<float> {
 public:
  static const float PI() { return 3.1415926535897932384626433832795f; } /*pi*/
  static const float TWO_PI() {
    return 6.283185307179586476925286766559f;
  } /*2pi*/
  static const float HALF_PI() {
    return 1.5707963267948966192313216916398f;
  } /*pi over 2*/
  static const float REC_TWO_PI() {
    return 0.15915494309189533576888376337251f;
  } /*reciprocal of 2pi*/
  static const float SQRT_2() {
    return 1.4142135623730950488016887242097f;
  } /*square root of 2*/
  static const float REC_SQRT_2() {
    return 0.70710678118654752440084436210485f;
  } /*reciprocal of square root of 2*/
  static const float SQRT_3() {
    return 1.7320508075688772935274463415059f;
  } /*square root of 3*/
  static const float REC_SQRT_3() {
    return 0.57735026918962576450914878050196f;
  } /*reciprocal of square root of 3*/
  static const float DEGREE_TO_RADIAN() {
    return 0.017453292519943295769236907684886f;
  } /*2pi over 360*/
  static const float RADIAN_TO_DEGREE() {
    return 57.295779513082320876798154814105f;
  } /*360 over 2pi*/
  static const float EPSILON() {
    return 5.96e-08f;
  } /*machine epsilon gives an upper bound on the relative error due to rounding
       in floating point arithmetic*/
  static const float SQRT_EPSILON() {
    return 2.44e-04f;
  } /*square root of epsilon*/
  static const float MAX_VAL() {
    return 3.40282e+038f;
  } /*the maximum value for type float*/
  /*
  For double and floating-point types with denormalization, MIN_VAL() returns
  the minimum positive normalized value.
  MIN_VAL is only meaningful for bounded types and  for unbounded unsigned
  types, that is, types that represent an infinite set of negative values have
  no meaningful minimum.
  */
  static const float MIN_VAL() {
    return 1.17549e-038f;
  } /*the minimum positive value for type float*/
  static const float LOWEST() {
    return -3.40282e+038f;
  } /*the lowest finite value representable by the numeric type float*/
  static const float MIN_ABS_SAFE_DIVIDEND() {
    return 1e-8f;
  } /*the minimum safe dividend when doing a divide operation, if abs(divident)
       < MIN_ABS_SAFE_DIVIDEND, it will be considered as division by zero*/
};

/*the look-up table for doing fast bit count (# f 1s) in an unsigned byte*/
static const int i_UByteBitCountLut[] = {
    0, 1, 1, 2, 1, 2, 2, 3, 1, 2, 2, 3, 2, 3, 3, 4, 1, 2, 2, 3, 2, 3, 3, 4,
    2, 3, 3, 4, 3, 4, 4, 5, 1, 2, 2, 3, 2, 3, 3, 4, 2, 3, 3, 4, 3, 4, 4, 5,
    2, 3, 3, 4, 3, 4, 4, 5, 3, 4, 4, 5, 4, 5, 5, 6, 1, 2, 2, 3, 2, 3, 3, 4,
    2, 3, 3, 4, 3, 4, 4, 5, 2, 3, 3, 4, 3, 4, 4, 5, 3, 4, 4, 5, 4, 5, 5, 6,
    2, 3, 3, 4, 3, 4, 4, 5, 3, 4, 4, 5, 4, 5, 5, 6, 3, 4, 4, 5, 4, 5, 5, 6,
    4, 5, 5, 6, 5, 6, 6, 7, 1, 2, 2, 3, 2, 3, 3, 4, 2, 3, 3, 4, 3, 4, 4, 5,
    2, 3, 3, 4, 3, 4, 4, 5, 3, 4, 4, 5, 4, 5, 5, 6, 2, 3, 3, 4, 3, 4, 4, 5,
    3, 4, 4, 5, 4, 5, 5, 6, 3, 4, 4, 5, 4, 5, 5, 6, 4, 5, 5, 6, 5, 6, 6, 7,
    2, 3, 3, 4, 3, 4, 4, 5, 3, 4, 4, 5, 4, 5, 5, 6, 3, 4, 4, 5, 4, 5, 5, 6,
    4, 5, 5, 6, 5, 6, 6, 7, 3, 4, 4, 5, 4, 5, 5, 6, 4, 5, 5, 6, 5, 6, 6, 7,
    4, 5, 5, 6, 5, 6, 6, 7, 5, 6, 6, 7, 6, 7, 7, 8};

} /*namespace idl*/
