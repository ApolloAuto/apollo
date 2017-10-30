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
 * @file obstacle.h
 **/

#ifndef MODULES_PLANNING_COMMON_OBSTACLE_H_
#define MODULES_PLANNING_COMMON_OBSTACLE_H_

#include <list>
#include <memory>
#include <string>
#include <vector>

#include "modules/perception/proto/perception_obstacle.pb.h"
#include "modules/prediction/proto/prediction_obstacle.pb.h"

#include "modules/common/math/box2d.h"
#include "modules/common/math/polygon2d.h"
#include "modules/common/math/vec2d.h"
#include "modules/planning/common/indexed_list.h"

namespace apollo {
namespace planning {

class Obstacle {
 public:
  Obstacle() = default;

  Obstacle(const std::string &id,
           const perception::PerceptionObstacle &perception_obstacle);

  Obstacle(const std::string &id,
           const perception::PerceptionObstacle &perception,
           const prediction::Trajectory &trajectory);

  const std::string &Id() const;

  std::int32_t PerceptionId() const;

  bool IsStatic() const;

  common::TrajectoryPoint GetPointAtTime(const double time) const;

  common::math::Box2d GetBoundingBox(
      const common::TrajectoryPoint &point) const;
  /**
   * @brief get the perception bounding box
   */
  const common::math::Box2d &PerceptionBoundingBox() const;

  /**
   * @brief get the perception polygon for the obstacle. It is more precise than
   * bounding box
   */
  const common::math::Polygon2d &PerceptionPolygon() const;

  const prediction::Trajectory &Trajectory() const;
  bool HasTrajectory() const { return has_trajectory_; }

  const perception::PerceptionObstacle &Perception() const;

  /**
   * @brief This is a helper function that can create obstacles from prediction
   * data.  The original prediction may have multiple trajectories for each
   * obstacle. But this function will create one obstacle for each trajectory.
   * @param predictions The prediction results
   * @return obstacles The output obstacles saved in a list of unique_ptr.
   */
  static std::list<std::unique_ptr<Obstacle>> CreateObstacles(
      const prediction::PredictionObstacles &predictions);

  static std::unique_ptr<Obstacle> CreateStaticVirtualObstacles(
      const std::string &id, const common::math::Box2d &obstacle_box);

  static bool IsStaticObstacle(
      const perception::PerceptionObstacle &perception_obstacle);

  static bool IsVirtualObstacle(
      const perception::PerceptionObstacle &perception_obstacle);

  static bool IsValidTrajectoryPoint(const common::TrajectoryPoint &point);

 private:
  std::string id_;
  std::int32_t perception_id_ = 0;
  bool is_static_ = false;
  bool is_virtual_ = false;
  bool has_trajectory_ = false;
  prediction::Trajectory trajectory_;
  perception::PerceptionObstacle perception_obstacle_;
  common::math::Box2d perception_bounding_box_;
  common::math::Polygon2d perception_polygon_;
};

typedef IndexedList<std::string, Obstacle> IndexedObstacles;
typedef ThreadSafeIndexedList<std::string, Obstacle> ThreadSafeIndexedObstacles;

}  // namespace planning
}  // namespace apollo

#endif  // MODULES_PLANNING_COMMON_OBSTACLE_H_
