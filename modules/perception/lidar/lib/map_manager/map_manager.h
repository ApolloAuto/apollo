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
#pragma once

#include <string>

#include "gtest/gtest_prod.h"

#include "modules/perception/base/hdmap_struct.h"
#include "modules/perception/lidar/common/lidar_frame.h"
#include "modules/perception/map/hdmap/hdmap_input.h"

namespace apollo {
namespace perception {
namespace lidar {

struct MapManagerInitOptions {};

struct MapManagerOptions {};

class MapManager {
 public:
  MapManager() = default;

  ~MapManager() = default;

  bool Init(const MapManagerInitOptions& options = MapManagerInitOptions());

  // @brief: update map structure and lidar2world pose
  // @param [in]: options
  // @param [in/out]: frame
  // hdmap_struct should be filled, required,
  // lidar2world_pose can be updated, optional,
  bool Update(const MapManagerOptions& options, LidarFrame* frame);

  bool QueryPose(Eigen::Affine3d* sensor2world_pose) const;

  std::string Name() const { return "MapManager"; }

 private:
  LidarFrame* cached_frame_ = nullptr;
  map::HDMapInput* hdmap_input_ = nullptr;
  // params
  bool update_pose_ = false;
  double roi_search_distance_ = 80.0;

  FRIEND_TEST(LidarLibMapManagerTest, lidar_map_manager_test);
};  // class MapManager

}  // namespace lidar
}  // namespace perception
}  // namespace apollo
