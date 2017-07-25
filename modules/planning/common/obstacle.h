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

#include <vector>

#include "modules/common/math/box2d.h"
#include "modules/planning/common/planning_object.h"
#include "modules/planning/common/trajectory/prediction_trajectory.h"

namespace apollo {
namespace planning {

class Obstacle : public PlanningObject {
 public:
  enum class ObstacleType {
    UNKNOWN = 0,
    UNKNOWN_MOVABLE = 1,
    UNKNOWN_UNMOVABLE = 2,
    PEDESTRIAN = 3,
    BICYCLE = 4,
    VEHICLE = 5,
  };

  Obstacle() = default;

  const std::string &Id() const;
  void SetId(int id);
  void SetId(const std::string &id);

  const ObstacleType &Type() const;
  void SetType(const ObstacleType &type);

  double Height() const;
  void SetHeight(const double height);

  double Width() const;
  void SetWidth(const double width);

  double Length() const;
  void SetLength(const double length);

  double Heading() const;
  void SetHeading(const double heading);

  double Speed() const;
  void SetSpeed(const double speed);

  ::apollo::common::math::Box2d BoundingBox() const;

  const std::vector<PredictionTrajectory> &prediction_trajectories() const;
  void add_prediction_trajectory(
      const PredictionTrajectory &prediction_trajectory);
  std::vector<PredictionTrajectory> *mutable_prediction_trajectories();

 private:
  std::string id_ = 0;
  double height_ = 0.0;
  double width_ = 0.0;
  double length_ = 0.0;
  double heading_ = 0.0;
  // NOTICE: check is speed_ is set before usage.
  double speed_ = 0.0;

  ::apollo::common::math::Vec2d center_;
  std::vector<PredictionTrajectory> prediction_trajectories_;
  ObstacleType type_ = ObstacleType::VEHICLE;
};

}  // namespace planning
}  // namespace apollo

#endif  // MODULES_PLANNING_COMMON_OBSTACLE_H_
