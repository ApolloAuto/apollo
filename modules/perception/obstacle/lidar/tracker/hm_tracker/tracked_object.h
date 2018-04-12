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

#ifndef MODULES_PERCEPTION_OBSTACLE_LIDAR_TRACKER_HM_TRACKER_TRACKED_OBJECT_H_
#define MODULES_PERCEPTION_OBSTACLE_LIDAR_TRACKER_HM_TRACKER_TRACKED_OBJECT_H_

#include <memory>
#include <string>
#include <vector>

#include "Eigen/Core"
#include "modules/perception/obstacle/base/object.h"

namespace apollo {
namespace perception {

struct TrackedObject {
  /* NEED TO NOTICE: All the states of track would be collected mainly based on
   * the states of tracked object. Thus, update tracked object's state when you
   * update the state of track !!! */
  TrackedObject() = default;
  explicit TrackedObject(std::shared_ptr<Object> obj_ptr);

  // deep copy (copy point clonds)
  void clone(const TrackedObject& rhs);

  // cloud
  // store transformed object before tracking
  std::shared_ptr<Object> object_ptr;

  Eigen::Vector3f barycenter;

  // bbox
  Eigen::Vector3f center;
  Eigen::Vector3f size;
  Eigen::Vector3f direction;
  Eigen::Vector3f lane_direction;

  // states
  Eigen::Vector3f anchor_point;
  Eigen::Vector3f velocity;
  Eigen::Matrix3f velocity_uncertainty;
  Eigen::Vector3f acceleration;

  // class type
  ObjectType type;

  // association distance
  // range from 0 to association_score_maximum
  float association_score = 0.0f;
};  // struct TrackedObject

}  // namespace perception
}  // namespace apollo

#endif
