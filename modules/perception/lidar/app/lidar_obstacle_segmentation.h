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

#include <memory>
#include <string>

#include "Eigen/Dense"

#include "modules/perception/lidar/common/lidar_error_code.h"
#include "modules/perception/lidar/lib/interface/base_classifier.h"
#include "modules/perception/lidar/lib/interface/base_segmentation.h"
#include "modules/perception/lidar/lib/map_manager/map_manager.h"
#include "modules/perception/lidar/lib/object_builder/object_builder.h"
#include "modules/perception/lidar/lib/object_filter_bank/object_filter_bank.h"
#include "modules/perception/lidar/lib/pointcloud_preprocessor/pointcloud_preprocessor.h"

namespace apollo {
namespace perception {
namespace lidar {

struct LidarObstacleSegmentationInitOptions {
  std::string sensor_name = "velodyne64";
  bool enable_hdmap_input = true;
};

struct LidarObstacleSegmentationOptions {
  std::string sensor_name;
  Eigen::Affine3d sensor2novatel_extrinsics;
};

class LidarObstacleSegmentation {
 public:
  LidarObstacleSegmentation() = default;
  ~LidarObstacleSegmentation() = default;

  bool Init(const LidarObstacleSegmentationInitOptions& options =
                LidarObstacleSegmentationInitOptions());

  LidarProcessResult Process(
      const LidarObstacleSegmentationOptions& options,
      const std::shared_ptr<apollo::drivers::PointCloud const>& message,
      LidarFrame* frame);

  LidarProcessResult Process(const LidarObstacleSegmentationOptions& options,
                             LidarFrame* frame);

  std::string Name() const { return "LidarObstacleSegmentation"; }

 private:
  LidarProcessResult ProcessCommon(
      const LidarObstacleSegmentationOptions& options, LidarFrame* frame);

 private:
  PointCloudPreprocessor cloud_preprocessor_;
  MapManager map_manager_;
  std::unique_ptr<BaseSegmentation> segmentor_;
  ObjectBuilder builder_;
  ObjectFilterBank filter_bank_;
  // params
  std::string segmentor_name_;
  bool use_map_manager_ = true;
  bool use_object_filter_bank_ = true;
};  // class LidarObstacleSegmentation

}  // namespace lidar
}  // namespace perception
}  // namespace apollo
