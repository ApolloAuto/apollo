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

// #include "gtest/gtest_prod.h"

#include "modules/perception/pointcloud_map_based_roi/map_manager/proto/map_manager_config.pb.h"

#include "modules/perception/common/base/hdmap_struct.h"
#include "modules/perception/common/hdmap/hdmap_input.h"
#include "modules/perception/common/lidar/common/lidar_frame.h"
#include "modules/perception/common/lib/interface/base_init_options.h"

namespace apollo {
namespace perception {
namespace lidar {

using apollo::perception::BaseInitOptions;

struct MapManagerInitOptions : public BaseInitOptions {
};

struct MapManagerOptions {};

class MapManager {
 public:
  /**
   * @brief Construct a new Map Manager object
   * 
   */
  MapManager() = default;

  /**
   * @brief Destroy the Map Manager object
   * 
   */
  ~MapManager() = default;

  /**
   * @brief Init of Map Manager
   * 
   * @param options map manager init options
   * @return true 
   * @return false 
   */
  bool Init(const MapManagerInitOptions& options = MapManagerInitOptions());

  /**
   * @brief update map structure and lidar2world pose
   * 
   * @param options map manager options
   * @param frame lidar frame
   * @return true 
   * @return false 
   */
  bool Update(const MapManagerOptions& options, LidarFrame* frame);

  /**
   * @brief Query the pose
   * 
   * @param sensor2world_pose sensor to world pose
   * @return true 
   * @return false 
   */
  bool QueryPose(Eigen::Affine3d* sensor2world_pose) const;

  /**
   * @brief Name of Map Manager
   * 
   * @return std::string 
   */
  std::string Name() const { return "MapManager"; }

 private:
  LidarFrame* cached_frame_ = nullptr;
  map::HDMapInput* hdmap_input_ = nullptr;
  // params
  bool update_pose_ = false;
  double roi_search_distance_ = 80.0;
};  // class MapManager

}  // namespace lidar
}  // namespace perception
}  // namespace apollo
