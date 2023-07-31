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

namespace apollo {
namespace perception {
namespace algorithm {
template <typename T>
class Constant;

template <>
class Constant<double> {
 public:
  static const double PI() { return 3.1415926535897932384626433832795; }  // pi
  static const double TWO_PI() {
    return 6.283185307179586476925286766559;
  }  // 2pi
  static const double HALF_PI() {
    return 1.5707963267948966192313216916398;
  }  // pi over 2
  static const double REC_TWO_PI() {
    return 0.15915494309189533576888376337251;
  }  // reciprocal of 2pi
  static const double SQRT_2() {
    return 1.4142135623730950488016887242097;
  }  // square root of 2
  static const double REC_SQRT_2() {
    return 0.70710678118654752440084436210485;
  }  // reciprocal of square root of 2
  static const double SQRT_3() {
    return 1.7320508075688772935274463415059;
  }  // square root of 3
  static const double REC_SQRT_3() {
    return 0.57735026918962576450914878050196;
  }  // reciprocal of square root of 3
  static const double DEGREE_TO_RADIAN() {
    return 0.017453292519943295769236907684886;
  }  // 2pi over 360
  static const double RADIAN_TO_DEGREE() {
    return 57.295779513082320876798154814105;
  }  // 360 over 2pi
  static const double EPSILON() { return 1.11e-016; }
  static const double SQRT_EPSILON() {
    return 1.053e-08;
  }  // square root of epsilon

  static const double MIN_ABS_SAFE_DIVIDEND() { return 1e-8; }
};

template <>
class Constant<float> {
 public:
  static const float PI() { return 3.1415926535897932384626433832795f; }  // pi
  static const float TWO_PI() {
    return 6.283185307179586476925286766559f;
  }  // 2pi
  static const float HALF_PI() {
    return 1.5707963267948966192313216916398f;
  }  // pi over 2
  static const float REC_TWO_PI() {
    return 0.15915494309189533576888376337251f;
  }  // reciprocal of 2pi
  static const float SQRT_2() {
    return 1.4142135623730950488016887242097f;
  }  // square root of 2
  static const float REC_SQRT_2() {
    return 0.70710678118654752440084436210485f;
  }  // reciprocal of square root of 2
  static const float SQRT_3() {
    return 1.7320508075688772935274463415059f;
  }  // square root of 3
  static const float REC_SQRT_3() {
    return 0.57735026918962576450914878050196f;
  }  // reciprocal of square root of 3
  static const float DEGREE_TO_RADIAN() {
    return 0.017453292519943295769236907684886f;
  }  // 2pi over 360
  static const float RADIAN_TO_DEGREE() {
    return 57.295779513082320876798154814105f;
  }  // 360 over 2pi
  static const float EPSILON() { return 5.96e-08f; }
  static const float SQRT_EPSILON() {
    return 2.44e-04f;
  }  // square root of epsilon

  static const float MIN_ABS_SAFE_DIVIDEND() { return 1e-8f; }
};

// the look-up table for doing fast bit count (# f 1s) in an unsigned byte
static const int kIUByteBitCountLut[] = {
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

}  // namespace algorithm
}  // namespace perception
}  // namespace apollo
