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

#include "modules/perception/lidar_segmentation/common/semantic_builder.h"

namespace apollo {
namespace perception {
namespace lidar {

bool SemanticBuilder::Init(const SemanticBuilderInitOptions& options) {
    return true;
}

bool SemanticBuilder::Build(const SemanticBuilderOptions& options, LidarFrame* frame) {
    if (frame == nullptr) {
        return false;
    }
    std::vector<ObjectPtr>* objects = &(frame->segmented_objects);
    for (size_t i = 0; i < objects->size(); ++i) {
        if (objects->at(i)) {
            GetObjSemanticAndMotionType(objects->at(i));
        }
    }
    return true;
}

void SemanticBuilder::GetObjSemanticAndMotionType(ObjectPtr object) {
    std::vector<int> type_count(static_cast<int>(PointSemanticLabel::MAX_LABEL), 0);
    std::vector<int> motion_type_count(static_cast<int>(PointMotionLabel::MAX_LABEL), 0);
    for (size_t i = 0; i < object->lidar_supplement.cloud.size(); i++) {
        uint8_t value = object->lidar_supplement.cloud.points_semantic_label(i);
        // get semantic label
        PointSemanticLabel label = GetSemanticLabel(value);
        int index = static_cast<int>(label);
        if (index >= 0 && index < static_cast<int>(PointSemanticLabel::MAX_LABEL)) {
            type_count[index]++;
        }
        // get motion label
        PointMotionLabel motion_label = GetMotionLabel(value);
        int motion_index = static_cast<int>(motion_label);
        if (motion_index >= 0 && motion_index < static_cast<int>(PointMotionLabel::MAX_LABEL)) {
            motion_type_count[motion_index]++;
        }
    }
    // get sematic type
    int max_value = -1;
    int max_index = 0;
    for (size_t i = 0; i < type_count.size(); i++) {
        if (type_count.at(i) > max_value) {
            max_value = type_count.at(i);
            max_index = i;
        }
    }
    // get object semantic type
    object->lidar_supplement.semantic_type = static_cast<base::ObjectSemanticType>(max_index);

    // get motion type
    int motion_max_value = -1;
    int motion_max_index = 0;
    for (size_t i = 0; i < motion_type_count.size(); i++) {
        if (motion_type_count.at(i) > motion_max_value) {
            motion_max_value = motion_type_count.at(i);
            motion_max_index = i;
        }
    }
    // get object motion type
    object->lidar_supplement.dynamic_state = static_cast<base::MotionState>(motion_max_index);
}

}  // namespace lidar
}  // namespace perception
}  // namespace apollo
