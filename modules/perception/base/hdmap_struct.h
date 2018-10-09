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
#include <vector>

#include "modules/perception/base/point_cloud.h"

namespace apollo {
namespace perception {
namespace base {

struct alignas(16) RoadBoundary {
  PointCloud<PointD> left_boundary;
  PointCloud<PointD> right_boundary;
};

struct alignas(16) LaneBoundary {
  PointCloud<PointD> left_boundary;
  PointCloud<PointD> right_boundary;
};

struct alignas(16) HdmapStruct {
  std::vector<RoadBoundary> road_boundary;
  std::vector<PointCloud<PointD>> road_polygons;
  std::vector<PointCloud<PointD>> hole_polygons;
  std::vector<PointCloud<PointD>> junction_polygons;
};

using HdmapStructPtr = std::shared_ptr<HdmapStruct>;
using HdmapStructConstPtr = std::shared_ptr<const HdmapStruct>;

}  // namespace base
}  // namespace perception
}  // namespace apollo
