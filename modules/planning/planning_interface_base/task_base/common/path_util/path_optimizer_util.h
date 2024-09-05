/******************************************************************************
 * Copyright 2023 The Apollo Authors. All Rights Reserved.
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

#include <utility>
#include <vector>

#include "modules/planning/planning_base/proto/piecewise_jerk_path_config.pb.h"

#include "modules/planning/planning_base/common/path/path_data.h"
#include "modules/planning/planning_base/common/path_boundary.h"

namespace apollo {
namespace planning {
// SLSstate contains: (s ,s' ,s''), (l, l', l'')
using SLState = std::pair<std::array<double, 3>, std::array<double, 3>>;

class PathOptimizerUtil {
 public:
  /**
   * @brief Data format tramform from raw data to FrenetFramePath
   */
  static FrenetFramePath ToPiecewiseJerkPath(const std::vector<double>& x,
                                             const std::vector<double>& dx,
                                             const std::vector<double>& ddx,
                                             const double delta_s,
                                             const double start_s);
  /**
   * @brief Calculation of jerk boundary based on vehicle kinematics model.
   */
  static double EstimateJerkBoundary(const double vehicle_speed);

  static std::vector<common::PathPoint>
  ConvertPathPointRefFromFrontAxeToRearAxe(const PathData& path_data);
  static void FormulateExtraConstraints(
      PathBound extra_path_bound, const PathBoundary& path_boundary,
      ObsCornerConstraints* extra_constraints);
  /**
   * @brief Piecewise jerk path optimizer.
   */
  static bool OptimizePath(
      const SLState& init_state, const std::array<double, 3>& end_state,
      std::vector<double> l_ref, std::vector<double> l_ref_weight,
      const PathBoundary& path_boundary,
      const std::vector<std::pair<double, double>>& ddl_bounds,
      double dddl_bound, const PiecewiseJerkPathConfig& config,
      std::vector<double>* x, std::vector<double>* dx,
      std::vector<double>* ddx);

  static bool OptimizePathWithTowingPoints(
      const SLState& init_state, const std::array<double, 3>& end_state,
      std::vector<double> l_ref, std::vector<double> l_ref_weight,
      std::vector<double> towing_l_ref, std::vector<double> towing_l_ref_weight,
      const PathBoundary& path_boundary,
      const std::vector<std::pair<double, double>>& ddl_bounds,
      double dddl_bound, const PiecewiseJerkPathConfig& config,
      std::vector<double>* x, std::vector<double>* dx,
      std::vector<double>* ddx);

  /**
   * @brief If ref_l is below or above path boundary, will update its values and
   * weights
   */
  static void UpdatePathRefWithBound(const PathBoundary& path_boundary,
                                     double weight, std::vector<double>* ref_l,
                                     std::vector<double>* weight_ref_l);

  static void UpdatePathRefWithBound(const PathBoundary& path_boundary,
                                     double weight,
                                     const std::vector<double>& towing_ref_l,
                                     std::vector<double>* ref_l,
                                     std::vector<double>* weight_ref_l);

  static void UpdatePathRefWithBoundInSidePassDirection(
      const PathBoundary& path_boundary, double weight,
      std::vector<double>* ref_l, std::vector<double>* weight_ref_l,
      bool is_left_side_pass);

  /**
   * @brief calculate ddl bound by referenceline kappa and adc lat accleration
   */
  static void CalculateAccBound(
      const PathBoundary& path_boundary, const ReferenceLine& reference_line,
      std::vector<std::pair<double, double>>* ddl_bounds);
};

}  // namespace planning
}  // namespace apollo
