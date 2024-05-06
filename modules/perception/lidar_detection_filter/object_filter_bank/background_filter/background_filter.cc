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

#include "modules/perception/lidar_detection_filter/object_filter_bank/background_filter/background_filter.h"

#include "modules/perception/common/util.h"

namespace apollo {
namespace perception {
namespace lidar {

bool BackgroundFilter::Init(const ObjectFilterInitOptions& options) {
  std::string config_file =
      GetConfigFile(options.config_path, options.config_file);
  BackgroundFilterConfig config;
  ACHECK(cyber::common::GetProtoFromFile(config_file, &config));
  outside_roi_filter_distance_ = config.outside_roi_filter_distance();
  return true;
}

bool BackgroundFilter::Filter(const ObjectFilterOptions& options,
                               LidarFrame* frame) {
  if (!frame) {
    AINFO << "Lidar frame is nullptr.";
    return false;
  }
  if (!frame->hdmap_struct) {
    AINFO << "HDMap struct is nullptr.";
    for (auto& object : frame->segmented_objects) {
      object->lidar_supplement.is_in_roi = true;
      object->lidar_supplement.is_background = false;
    }
    return true;
  }
  if (frame->hdmap_struct->road_boundary.size() +
      frame->hdmap_struct->road_polygons.size() +
      frame->hdmap_struct->junction_polygons.size() ==
      0) {
    AINFO << "Donot find hdmap polygons, set object foreground.";
    for (auto& object : frame->segmented_objects) {
      object->lidar_supplement.is_in_roi = true;
      object->lidar_supplement.is_background = false;
    }
    return true;
  }

  // build world polygons for background objects
  BuildWorldPolygons(options, *frame);
  // fill roi flag for background objects
  if (!FillInRoiFlag(options, *frame)) {
    AERROR << "BackgroundFilter, FillInRoiFlag error.";
    return false;
  }
  // filter background object outside roi according to distance
  size_t valid_num = 0;
  const EigenVector<base::RoadBoundary>& road_boundary =
      frame->hdmap_struct->road_boundary;
  for (size_t i = 0; i < frame->segmented_objects.size(); ++i) {
    auto object = frame->segmented_objects[i];
    if (!object->lidar_supplement.is_background ||
        is_in_roi_flag_[i]) {
      frame->segmented_objects[valid_num] = object;
      valid_num += 1;
      continue;
    }
    // calculate min_dist to lane boundary
    double dist_to_boundary = 0.0;
    double min_dist_to_boundary = std::numeric_limits<double>::max();
    Eigen::Vector3d direction;
    Eigen::Vector3d world_point;
    for (auto& point : polygons_in_world_[i].points()) {
      world_point << point.x, point.y, point.z;
      for (const auto& boundary : road_boundary) {
        perception::algorithm::CalculateDistAndDirToBoundary(
            world_point, boundary.left_boundary, boundary.right_boundary,
            &dist_to_boundary, &direction);
        if (min_dist_to_boundary > dist_to_boundary) {
          min_dist_to_boundary = dist_to_boundary;
        }
      }
      if (min_dist_to_boundary <= outside_roi_filter_distance_) {
        frame->segmented_objects[valid_num] = object;
        valid_num += 1;
        break;
      }
    }
  }
  size_t size = frame->segmented_objects.size();
  AINFO << "BackgroundFilter, filter " << size - valid_num << " objects from "
        << size << " objects, " << "reserve " << valid_num <<" objects.";

  frame->segmented_objects.resize(valid_num);

  // set foreground for roi objects
  for (size_t i = 0; i < frame->segmented_objects.size(); ++i) {
    if (is_in_roi_flag_[i]) {
      frame->segmented_objects[i]->lidar_supplement.is_background = false;
    }
  }
  return true;
}

void BackgroundFilter::BuildWorldPolygons(const ObjectFilterOptions& options,
                                          const LidarFrame& frame) {
  const Eigen::Affine3d& pose = frame.lidar2world_pose;
  const std::vector<base::ObjectPtr>& objects = frame.segmented_objects;
  polygons_in_world_.clear();
  polygons_in_world_.resize(objects.size());
  Eigen::Vector3d local_point;
  Eigen::Vector3d world_point;
  for (size_t i = 0; i < objects.size(); ++i) {
    const base::ObjectPtr& obj = objects[i];
    if (obj->lidar_supplement.is_background) {  // only for background object
      polygons_in_world_[i].resize(obj->polygon.size());
      for (size_t j = 0; j < obj->polygon.size(); ++j) {
        local_point(0) = obj->polygon[j].x;
        local_point(1) = obj->polygon[j].y;
        local_point(2) = obj->polygon[j].z;
        world_point = pose * local_point;
        polygons_in_world_[i][j].x = world_point(0);
        polygons_in_world_[i][j].y = world_point(1);
        polygons_in_world_[i][j].z = world_point(2);
      }
    }
  }
}

bool BackgroundFilter::FillInRoiFlag(
    const ObjectFilterOptions& options, const LidarFrame& frame) {
  is_in_roi_flag_.clear();
  is_in_roi_flag_.resize(frame.segmented_objects.size(), false);
  for (size_t i = 0; i < frame.segmented_objects.size(); ++i) {
    auto object = frame.segmented_objects.at(i);
    if (!object->lidar_supplement.is_background) {
      is_in_roi_flag_[i] = true;
      continue;
    }
    for (auto& point : polygons_in_world_[i].points()) {
      if (algorithm::IsPtInRoi(frame.hdmap_struct, point)) {
        is_in_roi_flag_[i] = true;
        break;
      }
    }
  }
  return true;
}

}  // namespace lidar
}  // namespace perception
}  // namespace apollo
