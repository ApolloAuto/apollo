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

#include "modules/perception/perception_plugin/lidar_detection_filter/object_split_filter/object_split_filter.h"

namespace apollo {
namespace perception {
namespace lidar {

bool ObjectSplitFilter::Init(const ObjectFilterInitOptions& options) {
    // get config
    std::string config_file = GetConfigFile(options.config_path, options.config_file);
    ACHECK(cyber::common::GetProtoFromFile(config_file, &config_));
    split_objects_.clear();
    id_ = config_.start_id();
    AINFO << "ObjectSplitFilter Configs: " << config_.DebugString() << "Init ObjectSpliterFilter Successful.";
    return true;
}

bool ObjectSplitFilter::Filter(const ObjectFilterOptions& options, LidarFrame* frame) {
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
    // clear split objects
    split_objects_.clear();
    // reset id
    id_ = config_.start_id();
    // split objects that cross with ego
    int valid_number = 0;
    for (size_t i = 0; i < frame->segmented_objects.size(); i++) {
        std::shared_ptr<base::Object> obj = frame->segmented_objects.at(i);
        if (IsCrossWithEgo(obj) && CheckForegroundBackground(obj)) {
            std::vector<base::ObjectPtr> this_split;
            SplitObject(obj, &this_split, config_.split_interval());
            BgObjectBuilder(&this_split, frame->lidar2novatel_extrinsics);
            SetSplitObjectsValue(obj, &this_split);
            split_objects_.insert(split_objects_.end(), this_split.begin(), this_split.end());
            continue;
        }
        frame->segmented_objects.at(valid_number) = obj;
        valid_number += 1;
    }
    int before_split = frame->segmented_objects.size();
    frame->segmented_objects.resize(valid_number);
    frame->segmented_objects.insert(frame->segmented_objects.end(), split_objects_.begin(), split_objects_.end());
    int after_split = frame->segmented_objects.size();

    AINFO << "ObjectSplitFilter, before split number: " << before_split << ", after split number: " << after_split;

    return true;
}

bool ObjectSplitFilter::IsCrossWithEgo(const base::ObjectPtr& object) {
    base::PointD top_left, bottom_left, bottom_right, top_right;
    // top left corner
    top_left.x = config_.forward_length();
    top_left.y = config_.left_width();
    // bottom left corner
    bottom_left.x = -config_.backard_length();
    bottom_left.y = config_.left_width();
    // bottom right corner
    bottom_right.x = -config_.backard_length();
    bottom_right.y = -config_.right_width();
    // top right corner
    top_right.x = config_.forward_length();
    top_right.y = -config_.right_width();

    // checkt if one of four corners is in object polygon
    bool is_top_left_corner_in_obj_polygon = algorithm::IsPointXYInPolygon2DXY(top_left, object->polygon);
    bool is_bottom_left_corner_in_obj_polygon = algorithm::IsPointXYInPolygon2DXY(bottom_left, object->polygon);
    bool is_bottom_right_corner_in_obj_polygon = algorithm::IsPointXYInPolygon2DXY(bottom_right, object->polygon);
    bool is_top_right_corner_in_obj_polygon = algorithm::IsPointXYInPolygon2DXY(top_right, object->polygon);
    if (is_top_left_corner_in_obj_polygon || is_bottom_left_corner_in_obj_polygon
        || is_bottom_right_corner_in_obj_polygon || is_top_right_corner_in_obj_polygon) {
        return true;
    }

    // check if ego car's polygon is cross with object polygon
    base::PointD pt_last = object->polygon.at(object->polygon.size() - 1);
    for (size_t i = 0; i < object->polygon.size(); ++i) {
        base::PointD pt_now = object->polygon.at(i);
        bool is_ego_left_edge_cross = IsLineSegmentsCross(top_left, bottom_left, pt_last, pt_now);
        bool is_ego_bottom_edge_cross = IsLineSegmentsCross(bottom_left, bottom_right, pt_last, pt_now);
        bool is_ego_right_edge_cross = IsLineSegmentsCross(bottom_right, top_right, pt_last, pt_now);
        bool is_ego_top_edge_cross = IsLineSegmentsCross(top_right, top_left, pt_last, pt_now);
        if (is_ego_left_edge_cross || is_ego_bottom_edge_cross || is_ego_right_edge_cross || is_ego_top_edge_cross) {
            return true;
        }
        pt_last.x = pt_now.x;
        pt_last.y = pt_now.y;
    }
    return false;
}

bool ObjectSplitFilter::IsLineSegmentsCross(base::PointD p1, base::PointD q1, base::PointD p2, base::PointD q2) {
    Eigen::Vector2d evp1(p1.x, p1.y);
    Eigen::Vector2d evq1(q1.x, q1.y);
    Eigen::Vector2d evp2(p2.x, p2.y);
    Eigen::Vector2d evq2(q2.x, q2.y);
    return algorithm::IsLineSegments2DIntersect(evp1, evq1, evp2, evq2);
}

bool ObjectSplitFilter::CheckForegroundBackground(const base::ObjectPtr& object) {
    if (object->lidar_supplement.is_background) {
        return true;
    }
    if ((!object->lidar_supplement.is_background) && config_.is_split_foreground_object()) {
        return true;
    }
    return false;
}

void ObjectSplitFilter::SetSplitObjectsValue(
        const base::ObjectPtr& base_object,
        std::vector<base::ObjectPtr>* split_objects) {
    for (size_t i = 0; i < split_objects->size(); i++) {
        base::ObjectPtr object = split_objects->at(i);
        object->confidence = base_object->confidence;
        object->id = NextID();
        object->lidar_supplement.is_in_roi = true;
        object->lidar_supplement.is_background = true;
        object->lidar_supplement.is_clustered = true;
        // classification
        object->type = base::ObjectType::UNKNOWN;
        object->lidar_supplement.raw_probs.push_back(
                std::vector<float>(static_cast<int>(base::ObjectType::MAX_OBJECT_TYPE), 0.f));
        object->lidar_supplement.raw_probs.back()[static_cast<int>(object->type)] = 1.0f;
        object->lidar_supplement.raw_classification_methods.push_back(
                base_object->lidar_supplement.raw_classification_methods.at(0));
        // copy to type
        object->type_probs.assign(
                object->lidar_supplement.raw_probs.back().begin(), object->lidar_supplement.raw_probs.back().end());
        // lidar supplement detections
        object->lidar_supplement.detections.resize(7);
        for (size_t jj = 0; jj < 7; jj++) {
            object->lidar_supplement.detections[jj] = 0.0;
        }
    }
}

int ObjectSplitFilter::NextID() {
    id_ += 1;
    return id_;
}

}  // namespace lidar
}  // namespace perception
}  // namespace apollo
