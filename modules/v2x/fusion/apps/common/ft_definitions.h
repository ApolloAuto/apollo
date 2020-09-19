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

#include <memory>

#include "modules/localization/proto/localization.pb.h"
#include "modules/perception/proto/perception_obstacle.pb.h"
#include "modules/v2x/proto/v2x_obstacles.pb.h"

#include "modules/v2x/fusion/libs/common/v2x_object.h"

namespace apollo {
namespace v2x {
namespace ft {

using apollo::localization::LocalizationEstimate;
using apollo::perception::PerceptionObstacle;
using apollo::perception::PerceptionObstacles;
using apollo::v2x::V2XObstacle;
using apollo::v2x::V2XObstacles;
using apollo::v2x::ft::base::Object;

using PerceptionObstaclesPtr = std::shared_ptr<PerceptionObstacles>;
using V2XObstaclesPtr = std::shared_ptr<V2XObstacles>;
using StatusPtr = std::shared_ptr<LocalizationEstimate>;

}  // namespace ft
}  // namespace v2x
}  // namespace apollo
