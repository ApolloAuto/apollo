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
 * @file
 */

#ifndef MODEULES_L3_PERCEPTION_DELPHI_ESR_RADAR_OBSTACLE_H_
#define MODEULES_L3_PERCEPTION_DELPHI_ESR_RADAR_OBSTACLE_H_

#include "modules/perception/proto/perception_obstacle.pb.h"

/**
 * @namespace apollo::l3_perception
 * @brief apollo::l3_perception
 */
namespace apollo {
namespace l3_perception {

using ::apollo::perception::Point;

class RadarObstacle {
 public:
  const Point position();
  const Point velocity();
  const uint32_t id();
  const double rcs();
  const bool movable();
  const double relative_range();
  const double relative_angle();
  const double relative_range_velocity();
  const double relative_lateral_velocity();
  const double width();
  const double length();
  const double height();
  
  void set_position(const Point& position);
  void set_position(const double x, const double y);
  void set_velocity(const Point& velocity);
  void set_velocity(const double x, const double y);
  void set_id(const uint32_t id);
  void set_rcs(const double rcs);
  void set_movable(const bool movable);
  void set_relative_range(const double relative_range);
  void set_relative_angle(const double relative_angle);
  void set_relative_range_velocity(const double relative_range_velocity);
  void set_relative_lateral_velocity(const double relative_lateral_velocity);
  void set_width(const double width);
  void set_length(const double length);
  void set_height(const double height);

 private:
  Point position_;
  Point velocity_;
  uint32_t id_;
  double rcs_;
  bool movable_; 
  double relative_range_;
  double relative_angle_;
  double relative_range_velocity_;
  double relative_lateral_velocity_;
  double width_;
  double length_;
  double height_;
};

}  // namespace l3_perception
}  // namespace apollo

#endif  // MODULES_L3_PERCEPTION_DELPHI_ESR_RADAR_OBSTACLE_H_

