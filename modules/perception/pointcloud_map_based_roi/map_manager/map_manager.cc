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

#include "modules/perception/pointcloud_map_based_roi/map_manager/map_manager.h"

#include "cyber/common/file.h"
#include "cyber/common/log.h"
#include "modules/perception/common/util.h"

namespace apollo {
namespace perception {
namespace lidar {

bool MapManager::Init(const MapManagerInitOptions& options) {
  std::string config_file =
      GetConfigFile(options.config_path, options.config_file);
  MapManagerConfig config;
  ACHECK(cyber::common::GetProtoFromFile(config_file, &config));
  update_pose_ = config.update_pose();
  roi_search_distance_ = config.roi_search_distance();
  hdmap_input_ = map::HDMapInput::Instance();
  if (!hdmap_input_->Init()) {
    AINFO << "Failed to init hdmap input.";
    return false;
  }
  return true;
}

bool MapManager::Update(const MapManagerOptions& options, LidarFrame* frame) {
  if (!frame) {
    AINFO << "Frame is nullptr.";
    return false;
  }
  if (!(frame->hdmap_struct)) {
    frame->hdmap_struct.reset(new base::HdmapStruct);
  }
  if (!hdmap_input_) {
    AINFO << "Hdmap input is nullptr";
    return false;
  }
  if (update_pose_) {
    if (!QueryPose(&(frame->lidar2world_pose))) {
      AINFO << "Failed to query updated pose.";
    }
  }
  base::PointD point;
  point.x = frame->lidar2world_pose.translation()(0);
  point.y = frame->lidar2world_pose.translation()(1);
  point.z = frame->lidar2world_pose.translation()(2);
  if (!hdmap_input_->GetRoiHDMapStruct(point, roi_search_distance_,
                                       frame->hdmap_struct)) {
    frame->hdmap_struct->road_polygons.clear();
    frame->hdmap_struct->road_boundary.clear();
    frame->hdmap_struct->hole_polygons.clear();
    frame->hdmap_struct->junction_polygons.clear();
    AINFO << "Failed to get roi from hdmap.";
  }
  return true;
}

bool MapManager::QueryPose(Eigen::Affine3d* sensor2world_pose) const {
  // TODO(...): map-based alignment to refine pose
  return false;
}

}  // namespace lidar
}  // namespace perception
}  // namespace apollo
