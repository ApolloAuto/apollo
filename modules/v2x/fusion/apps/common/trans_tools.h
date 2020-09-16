/******************************************************************************
 * Copyright 2020 The Apollo Authors. All Rights Reserved.
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

#pragma once

#include <cmath>
#include <memory>
#include <string>
#include <thread>
#include <vector>

#include "modules/v2x/fusion/apps/common/ft_definitions.h"

namespace apollo {
namespace v2x {
namespace ft {

base::Object Pb2Object(const PerceptionObstacle &obstacle,
                       const std::string &frame_id);
void Pb2Object(const PerceptionObstacle &obstacle, base::Object *object,
               const std::string &frame_id, double timestamp_object = 0.0);

void V2xPb2Object(const V2XObstacle &obstacle, base::Object *object,
                  const std::string &frame_id, double timestamp_object);

void CarstatusPb2Object(const LocalizationEstimate &carstatus,
                        base::Object *object, const std::string &frame_id);

double Pbs2Objects(const PerceptionObstacles &obstacles,
                   std::vector<base::Object> *objects,
                   const std::string &frame_id);

double V2xPbs2Objects(const V2XObstacles &obstacles,
                      std::vector<base::Object> *objects,
                      const std::string &frame_id);

PerceptionObstacle Object2Pb(const base::Object &object);
void FillObjectPolygonFromBBox3D(PerceptionObstacle *object_ptr);
void Object2Pb(const base::Object &object, PerceptionObstacle *obstacle);

void Object2V2xPb(const base::Object &object, V2XObstacle *obstacle);
void Objects2Pbs(const std::vector<base::Object> &objects,
                 std::shared_ptr<PerceptionObstacles> obstacles);

void Objects2V2xPbs(const std::vector<base::Object> &objects,
                    std::shared_ptr<V2XObstacles> obstacles);

}  // namespace ft
}  // namespace v2x
}  // namespace apollo
