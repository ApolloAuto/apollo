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
#include "modules/l3_perception/delphi_esr/radar_obstacle.h"

namespace apollo {
namespace l3_perception {

  const Point RadarObstacle::position() {
    return position_;
  }

  const Point RadarObstacle::velocity() {
    return velocity_;
  }

  const uint32_t RadarObstacle::id() {
    return id_;
  }

  const double RadarObstacle::rcs() {
    return rcs_;
  }

  const bool RadarObstacle::movable() {
    return movable_;
  }

  const double RadarObstacle::relative_range() {
    return relative_range_;
  }

  const double RadarObstacle::relative_angle() {
    return relative_angle_;
  }

  const double RadarObstacle::relative_range_velocity() {
    return relative_range_velocity_;
  }

  const double RadarObstacle::relative_lateral_velocity() {
    return relative_lateral_velocity_;
  }
  
  void RadarObstacle::set_position(const Point& position) {
    position_.set_x(position.x());
    position_.set_y(position.y());
  }
  
  void RadarObstacle::set_position(const double x, const double y) {
    position_.set_x(x);
    position_.set_y(y);
  }

  void RadarObstacle::set_velocity(const Point& velocity) {
    velocity_.set_x(velocity.x());
    velocity_.set_y(velocity.y());
  }

  void RadarObstacle::set_velocity(const double x, const double y) {
    velocity_.set_x(x);
    velocity_.set_y(y);
  }

  void RadarObstacle::set_id(const uint32_t id) {
    id_ = id;
  }

  void RadarObstacle::set_rcs(const double rcs) {
    rcs_ = rcs;
  }

  void RadarObstacle::set_movable(const bool movable) {
    movable_ = movable;
  }

  void RadarObstacle::set_relative_range(const double relative_range) {
    relative_range_ = relative_range;
  }

  void RadarObstacle::set_relative_angle(const double relative_angle) {
    relative_angle_ = relative_angle;
  }

  void RadarObstacle::set_relative_range_velocity(const double relative_range_velocity) {
    relative_range_velocity_ = relative_range_velocity;
  }

  void RadarObstacle::set_relative_lateral_velocity(const double relative_lateral_velocity) {
    relative_lateral_velocity_ = relative_lateral_velocity;
  }

}  // namespace l3_perception
}  // namespace apollo

