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

#include "cyber/common/macros.h"
#include "modules/perception/lib/registerer/registerer.h"
#include "modules/perception/lidar/common/lidar_error_code.h"
#include "modules/perception/lidar/common/lidar_frame.h"
#include "modules/perception/pipeline/pipeline.h"

namespace apollo {
namespace perception {
namespace lidar {

struct LidarObstacleTrackingInitOptions {
  std::string sensor_name = "velodyne64";
};

struct LidarObstacleTrackingOptions {
  std::string sensor_name = "velodyne64";
};

class BaseLidarObstacleTracking : public pipeline::Pipeline {
 public:
  using PipelineConfig = pipeline::PipelineConfig;
  using DataFrame = pipeline::DataFrame;

 public:
  BaseLidarObstacleTracking() = default;
  virtual ~BaseLidarObstacleTracking() = default;

  virtual bool Init(
            const LidarObstacleTrackingInitOptions& options =
            LidarObstacleTrackingInitOptions()) = 0;

  virtual LidarProcessResult Process(
            const LidarObstacleTrackingOptions& options,
            LidarFrame* frame) = 0;

  virtual bool Init(const PipelineConfig& pipeline_config) = 0;

  virtual bool Process(DataFrame* data_frame) = 0;

  virtual std::string Name() const = 0;

 private:
  DISALLOW_COPY_AND_ASSIGN(BaseLidarObstacleTracking);
};  // class BaseLidarObstacleTracking

PERCEPTION_REGISTER_REGISTERER(BaseLidarObstacleTracking);
#define PERCEPTION_REGISTER_LIDAROBSTACLETRACKING(name) \
  PERCEPTION_REGISTER_CLASS(BaseLidarObstacleTracking, name)

}  // namespace lidar
}  // namespace perception
}  // namespace apollo
