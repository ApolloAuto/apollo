/******************************************************************************
 * Copyright 2021 The Apollo Authors. All Rights Reserved.
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

#include "modules/perception/lidar/lib/interface/base_lidar_obstacle_detection.h"
#include "modules/perception/lidar/lib/interface/base_pointcloud_preprocessor.h"
#include "modules/perception/lidar/lib/interface/base_lidar_detector.h"

namespace apollo {
namespace perception {
namespace lidar {

class LidarObstacleDetection : public BaseLidarObstacleDetection{
 public:
  LidarObstacleDetection() = default;
  virtual ~LidarObstacleDetection() = default;

  bool Init(const LidarObstacleDetectionInitOptions& options =
                LidarObstacleDetectionInitOptions()) override;

  LidarProcessResult Process(
      const LidarObstacleDetectionOptions& options,
      const std::shared_ptr<apollo::drivers::PointCloud const>& message,
      LidarFrame* frame) override;

  LidarProcessResult Process(const LidarObstacleDetectionOptions& options,
                             LidarFrame* frame) override;

  std::string Name() const override { return "LidarObstacleDetection"; }

 private:
  LidarProcessResult ProcessCommon(const LidarObstacleDetectionOptions& options,
                                   LidarFrame* frame);

 private:
  std::shared_ptr<BasePointCloudPreprocessor> cloud_preprocessor_;
  std::shared_ptr<BaseLidarDetector> detector_;
};  // class LidarObstacleDetection

}  // namespace lidar
}  // namespace perception
}  // namespace apollo
