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
#ifndef PERCEPTION_COMMON_GEOMETRY_ROI_FILTER_H_
#define PERCEPTION_COMMON_GEOMETRY_ROI_FILTER_H_
#include <vector>
#include "modules/perception/base/hdmap_struct.h"
#include "modules/perception/base/object.h"
#include "modules/perception/base/point_cloud_types.h"
namespace apollo {
namespace perception {
namespace common {

// @brief: whether a point is in ROI.
bool IsPtInRoi(const base::HdmapStructConstPtr roi, const base::PointD pt);

// @brief: whether a object's center is in ROI.
bool IsObjectInRoi(const base::HdmapStructConstPtr roi,
                   const base::ObjectConstPtr obj);

// @brief: whether a object's bbox is in ROI.
bool IsObjectBboxInRoi(const base::HdmapStructConstPtr roi,
                       const base::ObjectConstPtr obj);

// @brief: whether objects' center are in ROI. If return True,
//         you can get objects in ROI by param valid_objects.
bool ObjectInRoiCheck(const base::HdmapStructConstPtr roi,
                      const std::vector<base::ObjectPtr>& objects,
                      std::vector<base::ObjectPtr>* valid_objects);

// @brief: whether objects' center and bbox is in ROI. If return True,
//         you can get objects in ROI by param valid_objects.
bool ObjectInRoiSlackCheck(const base::HdmapStructConstPtr roi,
                           const std::vector<base::ObjectPtr>& objects,
                           std::vector<base::ObjectPtr>* valid_objects);

}  // namespace common
}  // namespace perception
}  // namespace apollo

#endif  // PERCEPTION_COMMON_GEOMETRY_ROI_FILTER_H_
