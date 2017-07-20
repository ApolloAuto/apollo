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
 * @file: planning_object.cc
 *
 **/

#include "modules/planning/common/planning_object.h"

namespace apollo {
namespace planning {

PlanningObject::PlanningObjectType PlanningObject::ObjectType() const {
  return object_type_;
}

PlanningObject::PlanningObjectType* PlanningObject::MutableObjectType() {
  return &object_type_;
}

const std::vector<Decision>& PlanningObject::Decisions() const {
  return decisions_;
}

std::vector<Decision>* PlanningObject::MutableDecisions() {
  return &decisions_;
}

const ::apollo::common::math::Polygon2d& PlanningObject::Polygon() const {
  return polygon_;
}

::apollo::common::math::Polygon2d* PlanningObject::MutablePolygon() {
  return &polygon_;
}

}  // namespace planning
}  // namespace apollo

