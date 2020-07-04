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

#include <utility>

namespace apollo {
namespace planning {
namespace util {

// Helper function to convert world coordinates to relative coordinates
// around the obstacle of interest.
std::pair<double, double> WorldCoordToObjCoord(
    std::pair<double, double> input_world_coord,
    std::pair<double, double> obj_world_coord, double obj_world_angle);

double WorldAngleToObjAngle(double input_world_angle, double obj_world_angle);

}  // namespace util
}  // namespace planning
}  // namespace apollo
