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
#ifndef MODULES_PERCEPTION_OBSTACLE_BASE_HDMAP_STRUCT_H_
#define MODULES_PERCEPTION_OBSTACLE_BASE_HDMAP_STRUCT_H_

#include <memory>
#include <vector>

#include "modules/perception/obstacle/base/types.h"

namespace apollo {
namespace perception {

struct alignas(16) RoadBoundary {
  PolygonDType left_boundary;
  PolygonDType right_boundary;
};

struct alignas(16) HdmapStruct {
  std::vector<RoadBoundary> road_boundary;
  std::vector<PolygonDType> junction;
};

typedef std::shared_ptr<HdmapStruct> HdmapStructPtr;
typedef std::shared_ptr<const HdmapStruct> HdmapStructConstPtr;

}  // namespace perception
}  // namespace apollo

#endif  // MODULES_PERCEPTION_OBSTACLE_BASE_HDMAP_STRUCT_H_
