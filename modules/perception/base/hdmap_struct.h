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
#ifndef PERCEPTION_BASE_HDMAP_STRUCT_H_
#define PERCEPTION_BASE_HDMAP_STRUCT_H_
#include <memory>
#include <vector>
#include "modules/perception/base/point_cloud_types.h"

namespace apollo {
namespace perception {
namespace base {

struct alignas(16) RoadBoundary {
  PolygonDType left_boundary;
  PolygonDType right_boundary;
};

struct alignas(16) LaneBoundary {
  PolygonDType left_boundary;
  PolygonDType right_boundary;
};

struct alignas(16) HdmapStruct {
  std::vector<RoadBoundary> road_boundary;
  std::vector<PolygonDType> road_polygons;
  std::vector<PolygonDType> hole_polygons;
  std::vector<PolygonDType> junction_polygons;
};

typedef std::shared_ptr<HdmapStruct> HdmapStructPtr;
typedef std::shared_ptr<const HdmapStruct> HdmapStructConstPtr;

}  // namespace base
}  // namespace perception
}  // namespace apollo

#endif  // PERCEPTION_BASE_HDMAP_STRUCT_H_
