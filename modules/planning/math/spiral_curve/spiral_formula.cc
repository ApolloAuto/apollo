/******************************************************************************
 * Copyright 2017 The Apollo Authors. All Rights Reserved.
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

/*
 * @file: spiral_formula.cc
 */
#include "modules/planning/math/spiral_curve/spiral_formula.h"

namespace apollo {
namespace planning {

// coef transformation k3 indicates cubic spiral, k5 indicates quintic spiral
std::array<double, 4> SpiralFormula::p_to_a_k3(const double sg,
                                               const std::array<double, 4>& p) {
  std::array<double, 4> result = {
      // p params to cubic Poly params
      p[0], -(11.0 * p[0] - 18.0 * p[1] + 9.0 * p[2] - 2.0 * p[3]) / (2.0 * sg),
      (18.0 * p[0] - 45.0 * p[1] + 36.0 * p[2] - 9.0 * p[3]) / (2.0 * sg * sg),
      -(9 * p[0] - 27.0 * p[1] + 27.0 * p[2] - 9.0 * p[3]) /
          (2.0 * sg * sg * sg)};
  return result;
}

std::array<double, 6> SpiralFormula::p_to_a_k5(const double sg,
                                               const std::array<double, 6>& p) {
  double sg2 = sg * sg;
  double sg3 = sg2 * sg;

  std::array<double, 6> result = {
      // p params to quintic params
      p[0], p[1], p[2] / 2.0, -(575 * p[0] - 648 * p[3] + 81 * p[4] - 8 * p[5] +
                                170 * p[1] * sg + 22 * p[2] * sg2) /
                                  (8 * sg3),
      (333 * p[0] - 405 * p[3] + 81 * p[4] - 9 * p[5] + 90 * p[1] * sg +
       9 * p[2] * sg2) /
          (2 * sg2 * sg2),
      (-765 * p[0] + 972 * p[3] - 243 * p[4] + 36 * p[5] - 198 * p[1] * sg -
       18 * p[2] * sg2) /
          (8 * sg2 * sg3)};
  return result;
}

// kappa, theta, dkappa funcs without transformation
double SpiralFormula::kappa_func_k3_a(const double s,
                                      const std::array<double, 4>& a) {
  return ((a[3] * s + a[2]) * s + a[1]) * s + a[0];
}

double SpiralFormula::theta_func_k3_a(const double s,
                                      const std::array<double, 4>& a) {
  return (((a[3] * s / 4 + a[2] / 3) * s + a[1] / 2) * s + a[0]) * s;
}

double SpiralFormula::dkappa_func_k3_a(const double s,
                                       const std::array<double, 4>& a) {
  return (3 * a[3] * s + 2 * a[2]) * s + a[1];
}

double SpiralFormula::kappa_func_k5_a(const double s,
                                      const std::array<double, 6>& a) {
  return ((((a[5] * s + a[4]) * s + a[3]) * s + a[2]) * s + a[1]) * s + a[0];
}

double SpiralFormula::theta_func_k5_a(const double s,
                                      const std::array<double, 6>& a) {
  return (((((a[5] * s / 6 + a[4] / 5) * s + a[3] / 4) * s + a[2] / 3) * s +
           a[1] / 2) *
              s +
          a[0]) *
         s;
}

double SpiralFormula::dkappa_func_k5_a(const double s,
                                       const std::array<double, 6>& a) {
  return (((5 * a[5] * s + 4 * a[4]) * s + 3 * a[3]) * s + 2 * a[2]) * s + a[1];
}

// kappa, theta, dkappa funcs with p to a transformation
double SpiralFormula::kappa_func_k3(const double s, const double sg,
                                    const std::array<double, 4>& p) {
  std::array<double, 4> a = p_to_a_k3(sg, p);
  return ((a[3] * s + a[2]) * s + a[1]) * s + a[0];
}

double SpiralFormula::theta_func_k3(const double s, const double sg,
                                    const std::array<double, 4>& p) {
  std::array<double, 4> a = p_to_a_k3(sg, p);
  return (((a[3] * s / 4 + a[2] / 3) * s + a[1] / 2) * s + a[0]) * s;
}

double SpiralFormula::dkappa_func_k3(const double s, const double sg,
                                     const std::array<double, 4>& p) {
  std::array<double, 4> a = p_to_a_k3(sg, p);
  return (3 * a[3] * s + 2 * a[2]) * s + a[1];
}

double SpiralFormula::kappa_func_k5(const double s, const double sg,
                                    const std::array<double, 6>& p) {
  std::array<double, 6> a = p_to_a_k5(sg, p);
  return ((((a[5] * s + a[4]) * s + a[3]) * s + a[2]) * s + a[1]) * s + a[0];
}

double SpiralFormula::theta_func_k5(const double s, const double sg,
                                    const std::array<double, 6>& p) {
  std::array<double, 6> a = p_to_a_k5(sg, p);
  return (((((a[5] * s / 6 + a[4] / 5) * s + a[3] / 4) * s + a[2] / 3) * s +
           a[1] / 2) *
              s +
          a[0]) *
         s;
}

double SpiralFormula::dkappa_func_k5(const double s, const double sg,
                                     const std::array<double, 6>& p) {
  std::array<double, 6> a = p_to_a_k5(sg, p);
  return (((5 * a[5] * s + 4 * a[4]) * s + 3 * a[3]) * s + 2 * a[2]) * s + a[1];
}

double SpiralFormula::partial_theta_p1_k3(const double s, const double sg) {
  double sog = s / sg;
  return ((sog * 3.375 - 7.5) * sog + 4.5) * sog * s;
}

double SpiralFormula::partial_theta_p2_k3(const double s, const double sg) {
  double sog = s / sg;
  return ((6.0 - 3.375 * sog) * sog - 2.25) * sog * s;
}

double SpiralFormula::partial_theta_sg_k3(const double s, const double sg,
                                          const std::array<double, 4>& p) {
  double sog = s / sg;

  return ((3.375 * (p[0] - 3.0 * p[1] + 3.0 * p[2] - p[3]) * sog -
           3.0 * (2.0 * p[0] - 5.0 * p[1] + 4.0 * p[2] - p[3])) *
              sog +
          0.25 * (11.0 * p[0] - 18.0 * p[1] + 9.0 * p[2] - 2.0 * p[3])) *
         sog * sog;
}

double SpiralFormula::partial_theta_p3_k5(const double s, const double sg) {
  double sog = s / sg;
  // double ssog3 = s * sog * sog * sog;
  // double ssog4 = ssog3 * sog;
  return ((20.25 * sog - 40.5) * sog + 20.25) * sog * sog * sog * s;
  // return 20.25 * ssog3 - 40.5 * ssog4 + 20.25 * ssog4 * sog;
}

double SpiralFormula::partial_theta_p4_k5(const double s, const double sg) {
  double sog = s / sg;
  return ((-5.0625 * sog + 8.1) * sog - 2.53125) * sog * sog * sog * s;
}

double SpiralFormula::partial_theta_sg_k5(const double s, const double sg,
                                          const std::array<double, 6>& p) {
  double s2 = s * s;
  double sog = s / sg;
  double sog2 = sog * sog;
  double sog3 = sog2 * sog;
  double sog4 = sog2 * sog2;
  double sog5 = sog4 * sog;
  return (53.90625 * p[0] - 60.75 * p[3] + 7.59375 * p[4] - 0.75 * p[5]) *
             sog4 +
         10.625 * p[1] * s * sog3 + 0.6875 * p[2] * s2 * sog2 +
         (-133.2 * p[0] + 162 * p[3] - 32.4 * p[4] + 3.6 * p[5]) * sog5 +
         (-27) * p[1] * s * sog4 - 1.8 * p[2] * s2 * sog3 +
         (79.6875 * p[0] - 101.25 * p[3] + 25.3125 * p[4] - 3.75 * p[5]) *
             sog5 * sog +
         16.5 * p[1] * s * sog5 + 1.125 * p[2] * s2 * sog4;
}

}  // namespace planning
}  // namespace apollo
