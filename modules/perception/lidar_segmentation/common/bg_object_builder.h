/******************************************************************************
 * Copyright 2024 The Apollo Authors. All Rights Reserved.
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
#include <string>

#include "Eigen/Core"

#include "modules/perception/common/perception_gflags.h"
#include "modules/perception/common/base/object.h"
#include "modules/perception/common/base/point.h"
#include "modules/perception/common/base/point_cloud.h"

#include "modules/perception/common/algorithm/geometry/common.h"
#include "modules/perception/common/algorithm/geometry/convex_hull_2d.h"

#include "modules/perception/common/lidar/common/object_builder.h"

namespace apollo {
namespace perception {
namespace lidar {

/**
 * @brief background objects builder
 *
 * @param objects background objects
 * @return true
 * @return false
 */
bool BgObjectBuilder(std::vector<base::ObjectPtr>* objects, Eigen::Affine3d& lidar2novatel_pose);

}  // namespace lidar
}  // namespace perception
}  // namespace apollo
