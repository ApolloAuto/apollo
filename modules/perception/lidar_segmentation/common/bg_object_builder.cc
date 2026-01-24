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

#include "modules/perception/lidar_segmentation/common/bg_object_builder.h"

#include "modules/perception/common/algorithm/geometry/common.h"
#include "modules/perception/common/algorithm/geometry/convex_hull_2d.h"
#include "modules/perception/common/lidar/common/lidar_object_util.h"

namespace apollo {
namespace perception {
namespace lidar {

bool BgObjectBuilder(std::vector<base::ObjectPtr>* objects, Eigen::Affine3d& lidar2novatel_pose) {
    if (objects == nullptr) {
        AINFO << "objects is null.";
        return false;
    }
    ObjectBuilder object_builder;
    object_builder.Init();
    for (size_t i = 0; i < objects->size(); i++) {
        if (objects->at(i)) {
            object_builder.ComputePolygon2D(objects->at(i));
            ComputePolygonDirection(Eigen::Vector3d(0, 0, 0), objects->at(i), &objects->at(i)->direction);
            object_builder.ComputePolygonSizeCenter(objects->at(i));
            object_builder.ComputeOtherObjectInformation(objects->at(i));
            object_builder.ComputeHeightAboveGround(objects->at(i));
            if (FLAGS_need_judge_front_critical) {
                objects->at(i)->lidar_supplement.is_clustered = true;
                object_builder.JudgeFrontCritical(objects->at(i), lidar2novatel_pose);
            }
        }
    }
    return true;
}

}  // namespace lidar
}  // namespace perception
}  // namespace apollo
