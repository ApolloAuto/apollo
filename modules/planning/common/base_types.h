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

#ifndef MODULES_PLANNING_COMMON_BASE_TYPES_H_
#define MODULES_PLANNING_COMMON_BASE_TYPES_H_

/**
 * @namespace apollo::planning
 * @brief apollo::planning
 */
namespace apollo {
namespace planning {

/**
 * @class PathPoint
 * @brief PathPoint is a data structure class
 *        for discretized path representation.
 */
class PathPoint {
 public:
  /// x coordinate
  double x = 0.0;
  /// y coordinate
  double y = 0.0;
  /// z coordinate
  double z = 0.0;
  /// derivative direction on the x-y plane
  double theta = 0.0;
  /// curvature on the x-y planning
  double kappa = 0.0;
  /// accumulated distance from beginning of the path
  double s = 0.0;
};

/**
 * @class TrajectoryPoint
 * @brief TrajectoryPoint is a data structure class
 *        for discretized trajectory representation.
 *        It inherits the variables from base class PathPoint.
 */
class TrajectoryPoint : public PathPoint {
 public:
  /// relative time from beginning of the trajectory
  double relative_time = 0.0;
  /// linear velocity
  double v = 0.0;
  /// linear acceleration
  double a = 0.0;
  /// curvature change rate w.r.t. time
  double dkappa = 0.0;
};

}  // namespace planning
}  // namespace apollo

#endif /* MODULES_PLANNING_COMMON_BASE_TYPES_H_ */
