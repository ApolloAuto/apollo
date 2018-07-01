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

/**
 * @file
 * @brief This file provides the declaration of the class "LocalPath".
 */

#ifndef MODULES_PLANNING_NAVI_LOCAL_PATH_H_
#define MODULES_PLANNING_NAVI_LOCAL_PATH_H_

#include <memory>
#include <vector>

#include "modules/common/math/vec2d.h"
#include "modules/common/proto/pnc_point.pb.h"

/**
 * @namespace apollo::planning
 * @brief apollo::planning
 */
namespace apollo {
namespace planning {
/**
 * @class LocalPath
 * @brief LocalPath is used to privide some path point operations for decider
 */
class LocalPath {
 public:
  LocalPath() = default;

  explicit LocalPath(const std::vector<common::PathPoint> &path_points);

  virtual ~LocalPath() = default;

  /**
   * @brief get the first y of points vector.
   * @param init_y the initial y coordinate are stored here.
   * @return bool true if the path is not empty, false otherwise..
   */
  bool GetInitY(double *init_y);

  /**
   * @brief get the xy coordinate vector.
   * @return const std::vector<common::math::Vec2d>& constant reference of
   * the (x,y) coordinates vector.
   */
  const std::vector<common::math::Vec2d> &GetXYPoints();

  /**
   * @brief get the PathPoints vector.
   * @return const std::vector<common::math::PathPoint>& constant reference of
   * the PathPoints vector.
   */
  const std::vector<common::PathPoint> &GetPathPoints();

  /**
   * @brief get points range.
   * @return int the valid points range.
   */
  int GetRange();

  /**
   * @brief shift the points.
   * @param dist shift distance.
   * @return void.
   */
  void Shift(const double dist);

  /**
   * @brief cut the points.
   * @param dist cut distance.
   * @return void.
   */
  void Cut(const double dist);

  /**
   * @brief resample the points.
   * @return void.
   */
  void Resample();

  /**
   * @brief merge the points with local path.
   * @param common::Path local_path the path which is used to merge with points.
   * @param weight the weight coefficient.
   * @return void.
   */
  void Merge(const common::Path &local_path, const double weight);

 private:
  std::vector<common::PathPoint> path_points_;
  std::vector<common::math::Vec2d> points_;
};

inline const std::vector<common::math::Vec2d> &LocalPath::GetXYPoints() {
  return points_;
}

inline const std::vector<common::PathPoint> &LocalPath::GetPathPoints() {
  return path_points_;
}

inline int LocalPath::GetRange() {
  return static_cast<int>(path_points_.size() - 1);
}

}  // namespace planning
}  // namespace apollo

#endif  // MODULES_PLANNING_NAVI_LOCAL_PATH_H_
