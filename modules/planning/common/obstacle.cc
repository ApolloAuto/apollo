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
 * @file obstacle.cc
 **/

#include "modules/planning/common/obstacle.h"

#include <sstream>

namespace apollo {
namespace planning {

const std::string& Obstacle::Id() const { return id_; }
void Obstacle::SetId(int id) { id_ = std::to_string(id); }
void Obstacle::SetId(const std::string& id) { id_ = id; }

double Obstacle::Height() const { return height_; }

void Obstacle::SetHeight(const double height) { height_ = height; }

double Obstacle::Width() const { return width_; }

void Obstacle::SetWidth(const double width) { width_ = width; }

double Obstacle::Length() const { return length_; }

void Obstacle::SetLength(const double length) { length_ = length; }

double Obstacle::Heading() const { return heading_; }

void Obstacle::SetHeading(const double heading) { heading_ = heading; }

::apollo::common::math::Box2d Obstacle::BoundingBox() const {
  return ::apollo::common::math::Box2d(center_, heading_, length_, width_);
}

const PerceptionObstacle::Type& Obstacle::Type() const {
  return type_; }

void Obstacle::SetType(const PerceptionObstacle::Type& type) {
  type_ = type; }

double Obstacle::Speed() const { return speed_; }

void Obstacle::SetSpeed(const double speed) { speed_ = speed; }

const std::vector<PredictionTrajectory>& Obstacle::prediction_trajectories()
    const {
  return prediction_trajectories_;
}

void Obstacle::add_prediction_trajectory(
    const PredictionTrajectory& prediction_trajectory) {
  prediction_trajectories_.push_back(prediction_trajectory);
}

}  // namespace planning
}  // namespace apollo
