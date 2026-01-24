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

#include "modules/perception/perception_plugin/lidar_detection_filter/semantic_filter/semantic_filter.h"
#include "modules/perception/common/lidar/common/lidar_point_label.h"

namespace apollo {
namespace perception {
namespace lidar {

bool SemanticFilter::Init(const ObjectFilterInitOptions& options) {
    // clear
    filter_flag_.clear();
    semantic_type_map_.clear();
    // get config
    std::string config_file = GetConfigFile(options.config_path, options.config_file);
    SemanticFilterConfig config;
    ACHECK(cyber::common::GetProtoFromFile(config_file, &config));

    for (int i = 0; i < config.filter_type_size(); i++) {
        std::string type_name = config.filter_type(i);
        std::transform(type_name.begin(), type_name.end(), type_name.begin(), ::toupper);
        semantic_type_map_[kName2ObjectSemanticTypeMap.at(type_name)] = 1;
    }
    ground_prob_thr_ = config.ground_prob_thr();
    ground_height_thr_ = config.ground_height_thr();
    is_filter_cone_ = config.is_filter_cone();
    only_filter_cluster_ = config.only_filter_cluster();
    for (int i = 0; i < config.front_critical_filter_size(); i++) {
        std::string semantic_name = config.front_critical_filter(i).semantic_type();
        std::transform(semantic_name.begin(), semantic_name.end(), semantic_name.begin(), ::toupper);
        auto pair = std::make_pair(
                config.front_critical_filter(i).filter_strategy(), config.front_critical_filter(i).height_thres());
        front_critical_filter_map_[kName2ObjectSemanticTypeMap.at(semantic_name)] = pair;
    }
    AINFO << "Init SemanticFilter Successful.";
    return true;
}

bool SemanticFilter::Filter(const ObjectFilterOptions& options, LidarFrame* frame) {
    // check input
    if (frame == nullptr) {
        AERROR << "Input null frame ptr.";
        return false;
    }
    if (frame->cloud == nullptr) {
        AERROR << "Input null frame cloud.";
        return false;
    }
    if (frame->cloud->size() == 0) {
        AERROR << "Input none points.";
        return false;
    }
    // clear filter flags
    filter_flag_.clear();
    filter_flag_.resize(frame->segmented_objects.size(), false);
    // get filter flags
    if (!FillFilterFlags(frame)) {
        AERROR << "FillFilterFlags error.";
        return false;
    }
    if (!FillFilterFlagsOfGround(frame)) {
        AERROR << "FillFilterFlagsOfGround error.";
        return false;
    }
    if (!FillFilterFlagsOfFrontCritical(frame)) {
        AERROR << "FillFilterFlagsOfFrontCritical error.";
        return false;
    }

    // do filter
    size_t size = frame->segmented_objects.size();
    size_t valid_num = 0;
    for (size_t i = 0; i < filter_flag_.size(); ++i) {
        base::ObjectPtr obj = frame->segmented_objects.at(i);
        if (!filter_flag_.at(i)) {
            frame->segmented_objects.at(valid_num) = obj;
            valid_num++;
        }
    }
    frame->segmented_objects.resize(valid_num);
    AINFO << "SemanticFilter, filter " << size - valid_num << " objects, from " << size << " objects";

    return true;
}

bool SemanticFilter::FillFilterFlags(LidarFrame* frame) {
    // get flags according to semantic-type
    auto objects = frame->segmented_objects;
    for (size_t i = 0; i < objects.size(); i++) {
        auto obj = objects.at(i);
        if (only_filter_cluster_ && !obj->lidar_supplement.is_clustered) {
            continue;
        }
        auto iter = semantic_type_map_.find(obj->lidar_supplement.semantic_type);
        if (iter != semantic_type_map_.end()) {
            filter_flag_.at(i) = true;
        }
    }

    return true;
}

bool SemanticFilter::FillFilterFlagsOfGround(LidarFrame* frame) {
    // get flags according to semantic ground points
    auto objects = frame->segmented_objects;
    for (size_t i = 0; i < objects.size(); i++) {
        auto object = objects.at(i);
        if (filter_flag_.at(i)) {
            continue;
        }

        float min_z = std::numeric_limits<float>::max();
        float max_z = std::numeric_limits<float>::min();
        std::vector<int> type_count(static_cast<int>(PointSemanticLabel::MAX_LABEL), 0);
        for (size_t i = 0; i < object->lidar_supplement.cloud.size(); i++) {
            uint8_t value = object->lidar_supplement.cloud.points_semantic_label(i);
            // get semantic label
            PointSemanticLabel label = GetSemanticLabel(value);
            int index = static_cast<int>(label);
            if (index >= 0 && index < static_cast<int>(PointSemanticLabel::MAX_LABEL)) {
                type_count[index]++;
            }
            auto pt = object->lidar_supplement.cloud.at(i);
            min_z = std::min(min_z, pt.z);
            max_z = std::max(max_z, pt.z);
        }
        int sum = std::accumulate(type_count.begin(), type_count.end(), 0);
        float ground_prob = 0.0;
        float curb_prob = 0.0;
        if (sum > 1) {
            ground_prob = static_cast<float>(type_count.at(2)) / static_cast<float>(sum);
            curb_prob = static_cast<float>(type_count.at(4)) / static_cast<float>(sum);
        }
        // get flag
        if (!object->is_front_critical && ground_prob >= ground_prob_thr_ && (max_z - min_z) <= ground_height_thr_) {
            filter_flag_.at(i) = true;
        }
        // get flag
        if (!object->is_front_critical && curb_prob >= ground_prob_thr_ && (max_z - min_z) <= ground_height_thr_) {
            filter_flag_.at(i) = true;
        }
    }
    return true;
}

bool SemanticFilter::FillFilterFlagsOfFrontCritical(LidarFrame* frame) {
    auto objects = frame->segmented_objects;
    // front-critical objects:
    // for Ignore Noise Curb -> directly filter(1)
    // for Vegetation -> above-ground filter(2)
    // for ground -> Not filter(0)
    for (size_t i = 0; i < objects.size(); i++) {
        auto obj = objects.at(i);
        // NOT is_front_critical, SKIP
        if (!obj->is_front_critical) {
            continue;
        }
        // centerpoint-detection and TRAFFICCONE -> RESERVE
        if (!is_filter_cone_ && !obj->lidar_supplement.is_clustered
            && obj->sub_type == base::ObjectSubType::TRAFFICCONE) {
            filter_flag_.at(i) = false;
            continue;
        }
        auto iter = front_critical_filter_map_.find(obj->lidar_supplement.semantic_type);
        // not find -> default RESERVE
        if (iter == front_critical_filter_map_.end()) {
            filter_flag_.at(i) = false;
            continue;
        }
        int filter_strategy = front_critical_filter_map_.at(obj->lidar_supplement.semantic_type).first;
        float filter_height = front_critical_filter_map_.at(obj->lidar_supplement.semantic_type).second;
        // 0: no-filter; 1: filter; 2: float-filter
        if (filter_strategy == 1) {
            filter_flag_.at(i) = true;
            continue;
        } else if (filter_strategy == 2 && obj->lidar_supplement.height_above_ground > filter_height) {
            filter_flag_.at(i) = true;
            continue;
        } else {
            filter_flag_.at(i) = false;
            continue;
        }
    }
    return true;
}

}  // namespace lidar
}  // namespace perception
}  // namespace apollo
