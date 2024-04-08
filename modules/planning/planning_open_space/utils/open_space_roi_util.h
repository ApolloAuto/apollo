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

/**
 * @file
 **/

#pragma once

#include <algorithm>
#include <memory>
#include <string>
#include <vector>

#include "modules/planning/planning_base/common/frame.h"

namespace apollo {
namespace planning {

using apollo::common::math::Box2d;
using apollo::common::math::Vec2d;

class OpenSpaceRoiUtil {
 public:
  /**
   * @brief main process to compute and load info needed by open space planner
   */
  static bool FormulateBoundaryConstraints(
      OpenSpaceInfo* const open_space_info);
  /**
   * @brief get XY boundary which is minmax x or y in roi boundary
   * @param XYBoundary x_min x_max y_min y_max
   * @return
   */
  static void GetRoiXYBoundary(
      const std::vector<std::vector<common::math::Vec2d>>& roi_parking_boundary,
      std::vector<double>* const XYBoundary);
  static void TransformByOriginPoint(
      const common::math::Vec2d& origin_point,
      const double heading,
      std::vector<std::vector<common::math::Vec2d>>* roi_parking_boundary);
  static void TransformByOriginPoint(
      const common::math::Vec2d& origin_point,
      const double heading,
      std::vector<common::math::Vec2d>* roi_parking_boundary);
  static void TransformByOriginPoint(
      const common::math::Vec2d& origin_point,
      const double heading,
      common::math::Vec2d* point);
  /**
   * @brief Load obstacle polygon to roi_boundary in clock wise order
   * @param filtering_distance filter out distance which is far away from
   * adc
   * @param perception_obstacle_buffer add extend buffer for obstacle box
   * @param roi_boundary the roi polygon which adc can drive
   * @return true is running success
   */
  static bool LoadObstacles(
      const double filtering_distance,
      const double perception_obstacle_buffer,
      const std::vector<double>& xy_boundary,
      Frame* const frame,
      std::vector<std::vector<common::math::Vec2d>>*
          const roi_parking_boundary);
  /**
   * @brief is obstacle can be filterd out, filtered by distance and roi XY
   * bound
   * @param xy_boundary fileter obstacle which out of xy boundary
   * @param filtering_distance filter out distance which is far away from adc
   * @return true can be filtered out
   */
  static bool FilterOutObstacle(
      const double filtering_distance,
      const Frame& frame,
      const std::vector<double>& xy_boundary,
      const Obstacle& obstacle);
  /**
   * @brief brief Transform the vertice presentation of the obstacles into
   * linear inequality as Ax>b
   * @return true  is success
   */
  static bool LoadObstacleInHyperPlanes(OpenSpaceInfo* const open_space_info);
  static bool GetHyperPlanes(
      const size_t& obstacles_num,
      const Eigen::MatrixXi& obstacles_edges_num,
      const std::vector<std::vector<Vec2d>>& obstacles_vertices_vec,
      Eigen::MatrixXd* A_all,
      Eigen::MatrixXd* b_all);
  static void TransformByOriginPoint(
      const Vec2d& origin_point,
      const double& origin_heading,
      OpenSpaceInfo* open_space_info);
  static bool IsPolygonClockwise(const std::vector<Vec2d>& polygon);
  static bool AdjustPointsOrderToClockwise(std::vector<Vec2d>* polygon);
  static bool UpdateParkingPointsOrder(const apollo::hdmap::Path& nearby_path,
                                       std::vector<Vec2d>* points);
};

}  // namespace planning
}  // namespace apollo
