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
#include "modules/perception/common/algorithm/geometry/roi_filter.h"

#include <limits>

#include "Eigen/Dense"

#include "modules/perception/common/base/object_types.h"
#include "modules/perception/common/base/point_cloud.h"
#include "modules/perception/common/algorithm/geometry/common.h"

namespace apollo {
namespace perception {
namespace algorithm {

using HdmapStructConstPtr =
    std::shared_ptr<const apollo::perception::base::HdmapStruct>;
using apollo::perception::base::PointD;
using ObjectConstPtr = std::shared_ptr<const apollo::perception::base::Object>;
using ObjectPtr = std::shared_ptr<apollo::perception::base::Object>;
using apollo::perception::base::ObjectType;

bool IsPtInRoi(const HdmapStructConstPtr roi, const PointD pt) {
  for (std::size_t j = 0; j < roi->road_polygons.size(); j++) {
    if (IsPointXYInPolygon2DXY(pt, roi->road_polygons[j])) {
      return true;
    }
  }
  for (std::size_t j = 0; j < roi->junction_polygons.size(); j++) {
    if (IsPointXYInPolygon2DXY(pt, roi->junction_polygons[j])) {
      return true;
    }
  }
  return false;
}

bool IsObjectInRoi(const HdmapStructConstPtr roi, const ObjectConstPtr obj) {
  PointD ct;
  ct.x = obj->center[0];
  ct.y = obj->center[1];
  ct.z = obj->center[2];
  return IsPtInRoi(roi, ct);
}

bool IsObjectBboxInRoi(const HdmapStructConstPtr roi,
                       const ObjectConstPtr obj) {
  Eigen::Vector3d bbox_center = obj->center;
  PointD ct;
  ct.x = bbox_center[0];
  ct.y = bbox_center[1];
  ct.z = bbox_center[2];
  Eigen::Vector3d bbox_dir = obj->direction.cast<double>();
  bbox_dir(2) = 0.0;
  if (bbox_dir.norm() < std::numeric_limits<float>::epsilon()) {
    return IsPtInRoi(roi, ct);
  }
  bbox_dir.normalize();
  Eigen::Vector3d bbox_ortho_dir =
      Eigen::Vector3d(-bbox_dir(1), bbox_dir(0), 0.0f);
  double bbox_length = obj->size[0];
  double bbox_width = obj->size[1];
  Eigen::Vector3d bbox_corners[4];
  bbox_corners[0] = bbox_center + bbox_dir * bbox_length / 2 +
                    bbox_ortho_dir * bbox_width / 2;
  bbox_corners[1] = bbox_center - bbox_dir * bbox_length / 2 +
                    bbox_ortho_dir * bbox_width / 2;
  bbox_corners[2] = bbox_center + bbox_dir * bbox_length / 2 -
                    bbox_ortho_dir * bbox_width / 2;
  bbox_corners[3] = bbox_center - bbox_dir * bbox_length / 2 -
                    bbox_ortho_dir * bbox_width / 2;
  for (int i = 0; i < 4; ++i) {
    PointD corner;
    corner.x = bbox_corners[i][0];
    corner.y = bbox_corners[i][1];
    corner.z = bbox_corners[i][2];
    if (IsPtInRoi(roi, corner)) {
      return true;
    }
  }
  return false;
}

bool ObjectInRoiCheck(const HdmapStructConstPtr roi,
                      const std::vector<ObjectPtr>& objects,
                      std::vector<ObjectPtr>* valid_objects) {
  if (roi == nullptr ||
      (roi->road_polygons.empty() && roi->junction_polygons.empty())) {
    valid_objects->assign(objects.begin(), objects.end());
    return true;
  }

  valid_objects->clear();
  valid_objects->reserve(objects.size());
  for (std::size_t i = 0; i < objects.size(); i++) {
    if (IsObjectInRoi(roi, objects[i])) {
      valid_objects->push_back(objects[i]);
    }
  }

  return valid_objects->size() > 0;
}

}  // namespace algorithm
}  // namespace perception
}  // namespace apollo
