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
 * @file: spiral_formula.h
 * @brief: class for polynomial spiral functions, partial derivative (analytic
 * form) with respect to parameters.
 * Every function is optimized with minimum operations plus exact analytic form.
 * @warning: all partial derivatives shall guarantee sg > 0
 */

#ifndef MODULES_PLANNING_MATH_SPIRAL_CURVE_SPIRAL_FORMULA_H_
#define MODULES_PLANNING_MATH_SPIRAL_CURVE_SPIRAL_FORMULA_H_

#include <array>

namespace apollo {
namespace planning {

class SpiralFormula {
 public:
  /* ------------------------------ transformation -------------------------- */
  /**
   * @brief Convert p parameters to a parameters for cubic spiral
   * @param sg The final length of spiral path from start to end point
   * @param p Vector of params: p0, p1, p2, p3
   *                      p0 = kappa at s = 0
   *                      p1 = kappa at s = sg / 3
   *                      p2 = kappa at s = 2 * sg / 3
   *                      p3 = kappa at s = sg
   * @return a Parameter vec of cubic kappa spiral
   **/
  static std::array<double, 4> p_to_a_k3(const double sg,
                                         const std::array<double, 4>& p);

  /**
   * @brief Convert p parameters to a parameters for quintic spiral
   * @param sg The final length of spiral path from start to end point
   * @param p Vector of params: p0, p1, p2, p3, p4, p5
   *                      p0 = kappa  at s = 0
   *                      p1 = dkappa at s = 0
   *                      p2 = ddkappa at s = 0
   *                      p3 = kappa at s = sg / 3
   *                      p4 = kappa at s = 2 * sg / 3
   *
   * @return a Parameter vec of quintic kappa spiral
   **/
  static std::array<double, 6> p_to_a_k5(const double sg,
                                         const std::array<double, 6>& p);

  /* -------------------  kappa, theta, dkappa functions -------------------- */

  // set 1: kappa, theta, dkappa with a vector (normal vector) as parameter
  // input
  /**
   * @brief Cubic kappa function with regular parameter
   * @param s Distance from start to current point on the path
   * @param a Cubic polynomial params for cubic kappa spiral
   * @return kappa Value with respect to s, kappa(s) given params vec = a
   **/
  static double kappa_func_k3_a(const double s, const std::array<double, 4>& a);

  /**
   * @brief Cubic theta function with regular parameter
   *        theta value is the integration of kappa value
   * @param s Distance from start to current point on the path
   * @param a Cubic polynomial params for cubic kappa spiral
   * @return theta Value with respect to s, theta(s) given params vec = a
   **/
  static double theta_func_k3_a(const double s, const std::array<double, 4>& a);

  /**
   * @brief Derivative of cubic kappa function (dkappa) with regular parameter
   * @param s Distance from start to current point on the path
   * @param a Cubic polynomial params for cubic kappa spiral
   * @return dkappa Value with respect to s, dkappa(s) given params vec = a
   **/
  static double dkappa_func_k3_a(const double s,
                                 const std::array<double, 4>& a);

  /**
   * @brief Quintic kappa function with regular parameter
   * @param s Distance from start to current point on the path
   * @param a Quintic polynomial params for quintic kappa spiral
   * @return kappa Value with respect to s, kappa(s) given params vec = a
   **/
  static double kappa_func_k5_a(const double s, const std::array<double, 6>& a);

  /**
   * @brief quintic theta function with regular parameter
   *        theta value is the integration of kappa value
   * @param s Distance from start to current point on the path
   * @param a Quintic polynomial params for quintic kappa spiral
   * @return theta Value with respect to s, theta(s) given params vec = a
   **/
  static double theta_func_k5_a(const double s, const std::array<double, 6>& a);

  /**
   * @brief Derivative of quintic kappa function (dkappa) with regular parameter
   * @param s Distance from start to current point on the path
   * @param a Quintic polynomial params for quintic kappa spiral
   * @return dkappa Value with respect to s, dkappa(s) given params vec = a
   **/
  static double dkappa_func_k5_a(const double s,
                                 const std::array<double, 6>& a);

  // set 2 - kappa, theta dkappa funcs with p parameter

  /**
   * @brief Cubic theta function with p parameter
   *        theta value is the integration of kappa value
   * @param s Distance from start to current point on the path
   * @param p P params for cubic kappa spiral
   * @return theta Value with respect to s, theta(s) given params vec = a
   **/
  static double kappa_func_k3(const double s, const double sg,
                              const std::array<double, 4>& p);

  /**
   * @brief Cubic theta function with p parameter
   *        theta value is the integration of kappa value
   * @param s Distance from start to current point on the path
   * @param p P params for cubic kappa spiral
   * @return theta Value with respect to s, theta(s) given p
   **/
  static double theta_func_k3(const double s, const double sg,
                              const std::array<double, 4>& p);

  /**
   * @brief Derivative of cubic kappa function (dkappa) with p parameter
   * @param s Distance from start to current point on the path
   * @param p P params for cubic kappa spiral
   * @return dkappa Value with respect to s, dkappa(s) given p
   **/
  static double dkappa_func_k3(const double s, const double sg,
                               const std::array<double, 4>& p);

  /**
   * @brief Quintic kappa function with p parameter
   * @param s Distance from start to current point on the path
   * @param p Quintic polynomial params for quintic kappa spiral
   * @return kappa Value with respect to s, kappa(s) given p
   **/
  static double kappa_func_k5(const double s, const double sg,
                              const std::array<double, 6>& p);

  /**
   * @brief Quintic theta function with p parameter
   *        theta value is the integration of kappa value
   * @param s Distance from start to current point on the path
   * @param p Quintic polynomial params for quintic kappa spiral
   * @return theta Value with respect to s, theta(s) given p
   **/
  static double theta_func_k5(const double s, const double sg,
                              const std::array<double, 6>& p);

  /**
   * @brief Derivative of quintic kappa function (dkappa) with regular parameter
   * @param s Distance from start to current point on the path
   * @param p Quintic polynomial params for quintic kappa spiral
   * @return dkappa Value with respect to s, dkappa(s) given p params
   **/
  static double dkappa_func_k5(const double s, const double sg,
                               const std::array<double, 6>& p);

  /* -------------------------- Partial Derivatives ------------------------- */
  // Partial derivatives of theta with respect to p1, p2, sg (p3, p4 sg for
  // quintic version)

  /**
   * @brief Calculate partial derivative given sg (final curve length) at s
   * location;
   * @param s S location with respect to path's own SL coordinate
   * @param sg Final length of path
   * @param p P params
   * @return partial theta / partial p1, p2 or p3
   **/

  /* ----------------------------- Cubic Version ---------------------------- */

  /**
   * @brief: partial derivative theta with respect to p1
   **/
  static double partial_theta_p1_k3(const double s, const double sg);

  /**
   * @brief: partial derivative theta with respect to p2
   **/
  static double partial_theta_p2_k3(const double s, const double sg);

  /**
   * @brief: partial derivative theta with respect to sg
   **/
  static double partial_theta_sg_k3(const double s, const double sg,
                                    const std::array<double, 4>& p);

  /* --------------------------- Quintic Version ---------------------------- */

  /**
   * @brief: partial derivative theta with respect to p3
   **/
  static double partial_theta_p3_k5(const double s, const double sg);

  /**
   * @brief: partial derivative theta with respect to p4
   **/
  static double partial_theta_p4_k5(const double s, const double sg);

  /**
   * @brief: partial derivative theta with respect to sg
   **/
  static double partial_theta_sg_k5(const double s, const double sg,
                                    const std::array<double, 6>& p);

 private:
  SpiralFormula() = default;
};

}  // namespace planning
}  // namespace apollo
#endif  // MODULES_PLANNING_MATH_SPIRAL_CURVE_SPIRAL_FORMULA_H_
