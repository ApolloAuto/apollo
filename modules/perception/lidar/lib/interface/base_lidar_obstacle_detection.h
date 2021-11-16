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
#include <string>

#include "Eigen/Dense"

#include "cyber/common/macros.h"
#include "modules/perception/lidar/common/lidar_error_code.h"
#include "modules/perception/lib/registerer/registerer.h"
#include "modules/drivers/proto/pointcloud.pb.h"
#include "modules/perception/lidar/common/lidar_frame.h"

namespace apollo {
namespace perception {
namespace lidar {

struct LidarObstacleDetectionInitOptions {
  std::string sensor_name = "velodyne64";
  bool enable_hdmap_input = true;
};

struct LidarObstacleDetectionOptions {
  std::string sensor_name;
  Eigen::Affine3d sensor2novatel_extrinsics;

  EIGEN_MAKE_ALIGNED_OPERATOR_NEW
} EIGEN_ALIGN16;

class BaseLidarObstacleDetection {
 public:
  BaseLidarObstacleDetection() = default;
  virtual ~BaseLidarObstacleDetection() = default;

  virtual bool Init(
      const LidarObstacleDetectionInitOptions& options =
      LidarObstacleDetectionInitOptions()) = 0;

  virtual LidarProcessResult Process(
      const LidarObstacleDetectionOptions& options,
      const std::shared_ptr<apollo::drivers::PointCloud const>& message,
      LidarFrame* frame) = 0;

  virtual LidarProcessResult Process(
      const LidarObstacleDetectionOptions& options,
      LidarFrame* frame) = 0;

  virtual std::string Name() const = 0;

 private:
  DISALLOW_COPY_AND_ASSIGN(BaseLidarObstacleDetection);
};  // class BaseLidarObstacleDetection

PERCEPTION_REGISTER_REGISTERER(BaseLidarObstacleDetection);
#define PERCEPTION_REGISTER_LIDAROBSTACLEDETECTION(name) \
  PERCEPTION_REGISTER_CLASS(BaseLidarObstacleDetection, name)

}  // namespace lidar
}  // namespace perception
}  // namespace apollo
