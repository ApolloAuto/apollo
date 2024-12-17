/******************************************************************************
 * Copyright 2018 The Apollo Authors. All Rights Reserved.
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

#include "modules/perception/common/base/object.h"

namespace apollo {
namespace perception {
namespace lidar {

/**
 * @brief Get the Bounding Box vertices and save in polygon type
 *
 * @param object
 * @param box bounding box vertices(4 in xy plane)
 * @param expand expand valud, in meter
 */
void GetBoundingBox2d(const std::shared_ptr<base::Object>& object,
                      base::PointCloud<base::PointD>* box, double expand = 0.0);

/**
 * @brief compute object shape(center, size) from given direction and polygon
 *
 * @param object object, center and size will be updated
 * @param use_world_cloud whether use world cloud or local cloud
 */
void ComputeObjectShapeFromPolygon(std::shared_ptr<base::Object> object,
                                   bool use_world_cloud = false);

/**
 * @brief Compute polygon direction
 *
 * @param ref_center view point
 * @param object new object
 * @param direction orintation
 */
void ComputePolygonDirection(
    const Eigen::Vector3d& ref_center,
    const base::ObjectPtr& object,
    Eigen::Vector3f* direction);

/**
 * @brief Compute min max direction
 *
 * @param object new object
 * @param ref_center view point
 * @param max_point_index max point index
 * @param min_point_index min point index
 */
void ComputeMinMaxDirectionPoint(
    const base::ObjectPtr& object,
    const Eigen::Vector3d& ref_center,
    size_t* max_point_index,
    size_t* min_point_index);

}  // namespace lidar
}  // namespace perception
}  // namespace apollo
