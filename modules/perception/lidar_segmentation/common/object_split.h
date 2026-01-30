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
#include <limits>

#include "Eigen/Core"

#include "modules/perception/common/base/object.h"
#include "modules/perception/common/base/point_cloud.h"
#include "modules/perception/common/base/object_pool_types.h"

namespace apollo {
namespace perception {
namespace lidar {

/**
 * @brief split object
 *
 * @param obj
 * @param split_objs
 */
void SplitObject(const base::ObjectPtr& obj, std::vector<base::ObjectPtr>* split_objs, float split_distance);

/**
 * @brief line fitting according to max variance direction of pointcloud
 *
 * @tparam PointCloudT
 * @param cloud
 * @param params
 * @return true
 * @return false
 */
bool LineFit2D(const base::PointFCloud& cloud, Eigen::Vector3f* params);

}  // namespace lidar
}  // namespace perception
}  // namespace apollo
