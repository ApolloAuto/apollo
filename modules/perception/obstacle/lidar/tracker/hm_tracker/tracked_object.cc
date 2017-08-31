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

#include "modules/perception/obstacle/common/geometry_util.h"
#include "modules/perception/obstacle/lidar/tracker/hm_tracker/tracked_object.h"

namespace apollo {
namespace perception {

TrackedObject::TrackedObject() {
}

TrackedObject::TrackedObject(ObjectPtr obj_ptr) : object_ptr(obj_ptr) {
  if (object_ptr != nullptr) {
    barycenter = GetCloudBarycenter<apollo::perception::pcl_util::Point>(
      object_ptr->cloud).cast<float>();
    center = object_ptr->center.cast<float>();
    size = Eigen::Vector3f(object_ptr->length, object_ptr->width,
      object_ptr->height);
    direction = object_ptr->direction.cast<float>();
    lane_direction = Eigen::Vector3f::Zero();
    anchor_point = barycenter;
    velocity = Eigen::Vector3f::Zero();
    acceleration = Eigen::Vector3f::Zero();
    type = object_ptr->type;
  }
}

TrackedObject::TrackedObject(const TrackedObject& rhs) {
  object_ptr = rhs.object_ptr;
  center = rhs.center;
  direction = rhs.direction;
  lane_direction = rhs.lane_direction;
  size = rhs.size;
  type = rhs.type;
  barycenter = rhs.barycenter;
  anchor_point = rhs.anchor_point;
  distance_to_ref = rhs.distance_to_ref;
  angle_to_ref = rhs.angle_to_ref;
  velocity = rhs.velocity;
  acceleration = rhs.acceleration;
  angle = rhs.angle;
  angular_velocity = rhs.angular_velocity;
  type = rhs.type;
  association_score = rhs.association_score;
}

TrackedObject& TrackedObject::operator = (const TrackedObject& rhs) {
  object_ptr = rhs.object_ptr;
  center = rhs.center;
  direction = rhs.direction;
  lane_direction = rhs.lane_direction;
  size = rhs.size;
  type = rhs.type;
  barycenter = rhs.barycenter;
  anchor_point = rhs.anchor_point;
  distance_to_ref = rhs.distance_to_ref;
  angle_to_ref = rhs.angle_to_ref;
  velocity = rhs.velocity;
  acceleration = rhs.acceleration;
  angle = rhs.angle;
  angular_velocity = rhs.angular_velocity;
  type = rhs.type;
  association_score = rhs.association_score;
  return (*this);
}

void TrackedObject::clone(const TrackedObject& rhs) {
  object_ptr.reset(new Object());
  object_ptr->clone(*rhs.object_ptr);

  center = rhs.center;
  direction = rhs.direction;
  lane_direction = rhs.lane_direction;
  size = rhs.size;
  type = rhs.type;
  barycenter = rhs.barycenter;
  anchor_point = rhs.anchor_point;
  distance_to_ref = rhs.distance_to_ref;
  angle_to_ref = rhs.angle_to_ref;
  velocity = rhs.velocity;
  acceleration = rhs.acceleration;
  angle = rhs.angle;
  angular_velocity = rhs.angular_velocity;
  type = rhs.type;
  association_score = rhs.association_score;
}

std::string TrackedObject::to_string() const {
  std::string txt;
  return txt;
}

}  // namespace perception
}  // namespace apollo
