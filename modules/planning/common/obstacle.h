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

#include <string>
#include <vector>

#include "modules/common/math/box2d.h"
#include "modules/perception/proto/perception_obstacle.pb.h"
#include "modules/planning/common/planning_object.h"

namespace apollo {
namespace planning {

class Obstacle : public PlanningObject {
 public:
  Obstacle() = default;

  const std::string &Id() const;

  void SetId(const std::string &id);

  const perception::PerceptionObstacle::Type &Type() const;
  void SetType(const perception::PerceptionObstacle::Type &type);

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

  apollo::common::math::Vec2d Center() const { return center_; };

  apollo::common::math::Box2d BoundingBox() const;

  const std::vector<prediction::Trajectory> &prediction_trajectories() const {
    return trajectories_;
  }
  void add_prediction_trajectory(
      const prediction::Trajectory &prediction_trajectory) {
    trajectories_.push_back(prediction_trajectory);
  }

  common::TrajectoryPoint get_point_at(const prediction::Trajectory &trajectory,
                                       const double time) const;

 private:
  std::string id_ = 0;

  // TODO: (Liangliang) set those parameters.
  double height_ = 0.0;
  double width_ = 0.0;
  double length_ = 0.0;
  double heading_ = 0.0;
  // NOTICE: check is speed_ is set before usage.
  double speed_ = 0.0;

  std::vector<prediction::Trajectory> trajectories_;
  ::apollo::common::math::Vec2d center_;
  perception::PerceptionObstacle::Type type_ =
      perception::PerceptionObstacle::VEHICLE;
};

}  // namespace planning
}  // namespace apollo

#endif  // MODULES_PLANNING_COMMON_OBSTACLE_H_
