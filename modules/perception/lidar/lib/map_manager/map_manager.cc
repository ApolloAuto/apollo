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
#include "modules/perception/lidar/lib/map_manager/map_manager.h"
#include "modules/perception/lib/config_manager/config_manager.h"
#include "modules/perception/lib/io/file_util.h"
#include "modules/perception/lib/io/protobuf_util.h"
#include "modules/perception/lidar/common/lidar_log.h"
#include "modules/perception/lidar/lib/map_manager/proto/map_manager_config.pb.h"
namespace apollo {
namespace perception {
namespace lidar {

bool MapManager::Init(const MapManagerInitOptions& options) {
  lib::ConfigManager* config_manager =
      lib::Singleton<lib::ConfigManager>::get_instance();
  CHECK_NOTNULL(config_manager);
  const lib::ModelConfig* model_config = nullptr;
  CHECK(config_manager->GetModelConfig(Name(), &model_config));
  const std::string work_root = config_manager->work_root();
  std::string config_file;
  std::string root_path;
  CHECK(model_config->get_value("root_path", &root_path));
  config_file = lib::FileUtil::GetAbsolutePath(work_root, root_path);
  config_file = lib::FileUtil::GetAbsolutePath(config_file, "map_manager.conf");
  MapManagerConfig config;
  CHECK(lib::ParseProtobufFromFile(config_file, &config));
  update_pose_ = config.update_pose();
  roi_search_distance_ = config.roi_search_distance();
  hdmap_input_ = lib::Singleton<map::HDMapInput>::get_instance();
  if (hdmap_input_ == nullptr) {
    LOG_INFO << "Failed to load hdmap input.";
    return false;
  }
  if (!hdmap_input_->Init()) {
    LOG_INFO << "Failed to init hdmap input.";
    return false;
  }
  return true;
}

bool MapManager::Update(const MapManagerOptions& options, LidarFrame* frame) {
  if (!frame) {
    LOG_INFO << "Frame is nullptr.";
    return false;
  }
  if (!(frame->hdmap_struct)) {
    frame->hdmap_struct.reset(new base::HdmapStruct);
  }
  if (!hdmap_input_) {
    LOG_INFO << "Hdmap input is nullptr";
    return false;
  }
  if (update_pose_) {
    if (!QueryPose(&(frame->lidar2world_pose))) {
      LOG_INFO << "Failed to query updated pose.";
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
    LOG_INFO << "Failed to get roi from hdmap.";
  }
  return true;
}
bool MapManager::QueryPose(Eigen::Affine3d* sensor2world_pose) const {
  // TODO(...): map-based aligment to refine pose
  return false;
}

}  // namespace lidar
}  // namespace perception
}  // namespace apollo
