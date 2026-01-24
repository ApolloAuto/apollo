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

#include <algorithm>

#include "modules/perception/perception_plugin/lidar_detection_filter/mark_area_filter/mark_area_filter.h"

#include "modules/perception/common/util.h"
#include "modules/perception/common/lidar/common/lidar_timer.h"
#include "modules/perception/common/algorithm/geometry/common.h"
#include "modules/perception/common/base/object_types.h"

namespace apollo {
namespace perception {
namespace lidar {

bool MarkAreaFilter::Init(const ObjectFilterInitOptions& options) {
    std::string config_file = GetConfigFile(options.config_path, options.config_file);
    ACHECK(cyber::common::GetProtoFromFile(config_file, &mark_area_filter_config_));
    AINFO << "MarkAreaFilter init config param: " << mark_area_filter_config_.DebugString();

    mark_areas_.clear();
    filter_flag_.clear();
    area_foreground_type_.clear();

    for (int i = 0; i < mark_area_filter_config_.area_size(); i++) {
        const auto& area_config = mark_area_filter_config_.area(i);
        if (!area_config.isuse()) {
            continue;
        }

        // fill Polygon point
        std::shared_ptr<base::PolygonDType> mark_area_poly;
        mark_area_poly.reset(new base::PolygonDType());
        mark_area_poly->clear();
        base::PointD pt;
        for (int i = 0; i < area_config.boundary().point_size(); ++i) {
            auto& point = area_config.boundary().point(i);
            pt.x = point.x();
            pt.y = point.y();
            pt.z = point.z();
            mark_area_poly->push_back(pt);
        }

        if (mark_area_poly->size() < 3) {
            AINFO << "Area : " << area_config.id()
                  << " polygon point size illegal, at least three points are required.";
            return false;
        }

        mark_areas_[area_config.id()] = mark_area_poly;
        mark_areas_type_[area_config.id()] = area_config.filter_type();

        // fill foreground filter type
        std::vector<base::ObjectType> ObjectTypes;
        for (int i = 0; i < area_config.filter_foreground_type_size(); i++) {
            std::string type_name = area_config.filter_foreground_type(i);
            std::transform(type_name.begin(), type_name.end(), type_name.begin(), ::toupper);
            ObjectTypes.push_back(base::kObjectName2TypeMap.at(type_name));
        }
        area_foreground_type_[area_config.id()] = ObjectTypes;
    }

    filter_ratio_ = mark_area_filter_config_.filter_ratio();

    AINFO << "Init MarkAreaFilter Successful.";
    return true;
}

bool MarkAreaFilter::ComputeAreaDistance(LidarFrame* frame) {
    // Calculate the distance between each marked area and the vehicle
    float area_distance = 0.0;
    for (const auto& mark_areas_pair : mark_areas_) {
        auto front_point = mark_areas_pair.second->front();
        Eigen::Vector3d trans_point(front_point.x, front_point.y, front_point.z);
        trans_point = frame->lidar2world_pose.inverse() * trans_point;
        area_distance = std::sqrt(trans_point[0] * trans_point[0] + trans_point[1] * trans_point[1]);
        if (area_distance <= filter_ratio_) {
            area_distance_flags_.emplace(mark_areas_pair.first, true);
        }
    }

    if (area_distance_flags_.empty()) {
        return false;
    }

    return true;
}

bool MarkAreaFilter::IsPointInPolygon(
        LidarFrame* frame,
        std::shared_ptr<base::Object> obj,
        std::shared_ptr<base::PolygonDType> area_polygen) {
    base::PointD obj_center;
    obj_center.x = obj->center(0);
    obj_center.y = obj->center(1);
    obj_center.z = obj->center(2);
    Eigen::Vector3d vec3d_lidar(obj_center.x, obj_center.y, obj_center.z);
    Eigen::Vector3d vec3d_world = frame->lidar2world_pose * vec3d_lidar;
    obj_center.x = vec3d_world[0];
    obj_center.y = vec3d_world[1];
    obj_center.z = vec3d_world[2];

    if (!algorithm::IsPointXYInPolygon2DXY(obj_center, *area_polygen)) {
        return false;
    }
    return true;
}

void MarkAreaFilter::FilterBackgroundObstacles(LidarFrame* frame, std::shared_ptr<base::PolygonDType> area_polygen) {
    auto objects = frame->segmented_objects;
    for (size_t i = 0; i < objects.size(); i++) {
        auto obj = objects.at(i);
        if (filter_flag_[i] || !obj->lidar_supplement.is_background) {
            continue;
        }
        if (IsPointInPolygon(frame, obj, area_polygen)) {
            filter_flag_[i] = true;
        }
    }
}

void MarkAreaFilter::FilterAllObstacles(LidarFrame* frame, std::shared_ptr<base::PolygonDType> area_polygen) {
    auto objects = frame->segmented_objects;
    for (size_t i = 0; i < objects.size(); i++) {
        auto obj = objects.at(i);
        if (filter_flag_[i]) {
            continue;
        }
        if (IsPointInPolygon(frame, obj, area_polygen)) {
            filter_flag_[i] = true;
        }
    }
}

void MarkAreaFilter::FilterForegroundObstacles(
        LidarFrame* frame,
        std::shared_ptr<base::PolygonDType> area_polygen,
        std::string area_id) {
    auto objects = frame->segmented_objects;
    base::ObjectType obj_type;
    for (size_t i = 0; i < objects.size(); i++) {
        auto obj = objects.at(i);
        auto type_iter = area_foreground_type_.find(area_id);
        if (filter_flag_[i] || type_iter->second.empty()) {
            continue;
        }

        obj_type = obj->type;
        bool found = std::any_of(
                type_iter->second.begin(), type_iter->second.end(), [&obj_type](const base::ObjectType s) {
                    return s == obj_type;
                });

        if (found) {
            if (IsPointInPolygon(frame, obj, area_polygen)) {
                filter_flag_[i] = true;
            }
        }
    }
}

bool MarkAreaFilter::FilterAreaObstacles(LidarFrame* frame) {
    for (auto& legal_area : area_distance_flags_) {
        auto iter = mark_areas_type_.find(legal_area.first);
        if (iter == mark_areas_type_.end()) {
            AINFO << "FilterAreaObstacles no type to filter.";
            return false;
        }
        auto area_id = legal_area.first;
        auto poly_iter = mark_areas_.find(area_id);
        if (poly_iter == mark_areas_.end()) {
            AINFO << "FilterAreaObstacles no polygen to filter.";
            return false;
        }
        auto area_polygen = poly_iter->second;
        auto type = iter->second;

        switch (type) {
        case 0:
            // filter foreground and background obstacles within the marked area
            FilterAllObstacles(frame, area_polygen);
            break;
        case 1:
            // only filter background obstacles within the marked area
            FilterBackgroundObstacles(frame, area_polygen);
            break;
        case 2:
            // only filter foreground obstacles by type (for example VEHICLE) within the marked area
            FilterForegroundObstacles(frame, area_polygen, area_id);
            break;
        default:
            AERROR << "Unsupported type to filter!";
        }
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
    AINFO << "MarkAreaFilter, filter " << size - valid_num << " objects, from " << size << " objects";

    return true;
}

bool MarkAreaFilter::Filter(const ObjectFilterOptions& options, LidarFrame* frame) {
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
    area_distance_flags_.clear();

    AINFO << "BeforeMarkAreaFilter: " << std::to_string(frame->timestamp) << " object size is "
          << frame->segmented_objects.size();

    if (!ComputeAreaDistance(frame)) {
        AINFO << "no mark area to filter";
        return false;
    }

    if (!FilterAreaObstacles(frame)) {
        AERROR << "FilterAreaObstacles error.";
        return false;
    }

    AINFO << "AfterMarkAreaFilter: object size is " << frame->segmented_objects.size();
    return true;
}

}  // namespace lidar
}  // namespace perception
}  // namespace apollo
