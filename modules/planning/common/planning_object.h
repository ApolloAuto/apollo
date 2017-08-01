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
 * @file: planning_object.h
 *
 **/

#ifndef MODULES_PLANNING_COMMON_PLANNING_OBJECT_H_
#define MODULES_PLANNING_COMMON_PLANNING_OBJECT_H_

#include <vector>

#include "modules/common/math/polygon2d.h"
#include "modules/planning/proto/decision.pb.h"

namespace apollo {
namespace planning {

class PlanningObject {
 public:
  enum class PlanningObjectType {
    OBSTACLE = 0,
    MAP_OBJECT = 1,
  };

 public:
  PlanningObject() = default;
  virtual PlanningObjectType ObjectType() const;
  virtual PlanningObjectType* MutableObjectType();
  virtual const apollo::common::math::Polygon2d& Polygon() const;
  virtual apollo::common::math::Polygon2d* MutablePolygon();
  virtual const std::vector<ObjectDecisionType>& Decisions() const;
  virtual std::vector<ObjectDecisionType>* MutableDecisions();

 private:
  PlanningObjectType object_type_;
  apollo::common::math::Polygon2d polygon_;
  std::vector<ObjectDecisionType> decisions_;
};

}  // namespace planning
}  // namespace apollo

#endif  // MODULES_PLANNING_COMMON_PLANNING_OBJECT_H_
