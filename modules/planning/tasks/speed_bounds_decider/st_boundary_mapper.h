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

/**
 *   @file
 **/

#pragma once

#include <memory>
#include <string>
#include <vector>

#include "modules/common_msgs/config_msgs/vehicle_config.pb.h"
#include "modules/planning/tasks/speed_bounds_decider/proto/speed_bounds_decider.pb.h"
#include "modules/common/status/status.h"
#include "modules/planning/planning_base/common/dependency_injector.h"
#include "modules/planning/planning_base/common/obstacle.h"
#include "modules/planning/planning_base/common/path/path_data.h"
#include "modules/planning/planning_base/common/path_decision.h"
#include "modules/planning/planning_base/common/speed/st_boundary.h"
#include "modules/planning/planning_base/common/speed_limit.h"
#include "modules/planning/planning_base/reference_line/reference_line.h"

namespace apollo {
namespace planning {

class STBoundaryMapper {
 public:
  STBoundaryMapper(const SpeedBoundsDeciderConfig& config,
                   const ReferenceLine& reference_line,
                   const PathData& path_data, const double planning_distance,
                   const double planning_time,
                   const std::shared_ptr<DependencyInjector>& injector);

  virtual ~STBoundaryMapper() = default;

  common::Status ComputeSTBoundary(PathDecision* path_decision) const;

 private:
  FRIEND_TEST(StBoundaryMapperTest, check_overlap_test);
  FRIEND_TEST(StBoundaryMapperTest, get_overlap_boundary_points_test);

  /** @brief Calls GetOverlapBoundaryPoints to get upper and lower points
   * for a given obstacle, and then formulate STBoundary based on that.
   * It also labels boundary type based on previously documented decisions.
   */
  void ComputeSTBoundary(Obstacle* obstacle) const;

  /** @brief Map the given obstacle onto the ST-Graph. The boundary is
   * represented as upper and lower points for every s of interests.
   * Note that upper_points.size() = lower_points.size()
   */
  bool GetOverlapBoundaryPoints(
      const std::vector<common::PathPoint>& path_points,
      const Obstacle& obstacle, std::vector<STPoint>* upper_points,
      std::vector<STPoint>* lower_points) const;

  /** @brief Given a path-point and an obstacle bounding box, check if the
   *        ADC, when at that path-point, will collide with the obstacle.
   * @param The path-point of the center of rear-axis for ADC.
   * @param The bounding box of the obstacle.
   * @param The extra lateral buffer for our ADC.
   */
  bool CheckOverlap(const common::PathPoint& path_point,
                    const common::math::Box2d& obs_box,
                    const double l_buffer) const;

  /** @brief Given a path-point and an obstacle polygon, check if the
   *        ADC, when at that path-point, will collide with the obstacle.
   * @param The path-point of the center of rear-axis for ADC.
   * @param The polygon of the obstacle.
   * @param The extra lateral buffer for our ADC.
   */
  bool CheckOverlap(const common::PathPoint& path_point,
                    const common::math::Polygon2d& obs_polygon,
                    const double l_buffer,
                    common::math::Polygon2d* collision_ego_polygon = nullptr) const;

  /** @brief Maps the closest STOP decision onto the ST-graph. This STOP
   * decision can be stopping for blocking obstacles, or can be due to
   * traffic rules, etc.
   */
  bool MapStopDecision(Obstacle* stop_obstacle,
                       const ObjectDecisionType& decision) const;

  /** @brief Fine-tune the boundary for yielding or overtaking obstacles.
   * Increase boundary on the s-dimension or set the boundary type, etc.,
   * when necessary.
   */
  void ComputeSTBoundaryWithDecision(Obstacle* obstacle,
                                     const ObjectDecisionType& decision) const;

  bool CheckOverlapWithTrajectoryPoint(
      const DiscretizedPath& discretized_path,
      const common::math::Polygon2d& obstacle_shape,
      std::vector<STPoint>* upper_points, std::vector<STPoint>* lower_points,
      const double l_buffer, int default_num_point,
      const double obstacle_length, const double obstacle_width,
      const double trajectory_point_time) const;

 private:
  const SpeedBoundsDeciderConfig& speed_bounds_config_;
  const ReferenceLine& reference_line_;
  const PathData& path_data_;
  const common::VehicleParam& vehicle_param_;
  const double planning_max_distance_;
  const double planning_max_time_;
  std::shared_ptr<DependencyInjector> injector_;
};

}  // namespace planning
}  // namespace apollo
