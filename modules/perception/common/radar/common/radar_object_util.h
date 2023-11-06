/******************************************************************************
 * Copyright 2023 The Apollo Authors. All Rights Reserved.
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
namespace radar4d {

// @brief: get bounding box vertices and save in polygon type
// @param [in]: object
// @param [in]: expand valud, in meter
// @param [out]: bounding box vertices(4 in xy plane)
void GetBoundingBox2d(const std::shared_ptr<base::Object>& object,
                      base::RadarPointCloud<base::RadarPointD>* box,
                      double expand = 0.0);

// @brief: compute object shape(center, size) from given direction and polygon
// @param [in/out]: input object, center and size will be updated
// @param [in]: whether use world cloud or local cloud
void ComputeObjectShapeFromPolygon(std::shared_ptr<base::Object> object,
                                   bool use_world_cloud = false);

}  // namespace radar4d
}  // namespace perception
}  // namespace apollo
