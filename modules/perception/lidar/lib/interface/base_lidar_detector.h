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

#include <string>

#include "cyber/common/macros.h"
#include "modules/perception/lib/registerer/registerer.h"
#include "modules/perception/lidar/common/lidar_frame.h"
#include "modules/perception/pipeline/stage.h"

namespace apollo {
namespace perception {
namespace lidar {

struct LidarDetectorInitOptions {
  std::string sensor_name = "velodyne64";
};

struct LidarDetectorOptions {};

class BaseLidarDetector : public pipeline::Stage {
 public:
  using DataFrame = pipeline::DataFrame;
  using Plugin = pipeline::Plugin;
  using PluginType = pipeline::PluginType;
  using StageConfig = pipeline::StageConfig;

 public:
  BaseLidarDetector() = default;

  virtual ~BaseLidarDetector() = default;

  virtual bool Init(
      const LidarDetectorInitOptions& options = LidarDetectorInitOptions()) = 0;

  // @brief: process point cloud and get objects.
  // @param [in]: options
  // @param [in/out]: frame
  // objects should be filled, required,
  // label field of point cloud can be filled, optional,
  virtual bool Detect(const LidarDetectorOptions& options,
                       LidarFrame* frame) = 0;

  virtual bool Init(const StageConfig& stage_config) = 0;

  virtual bool Process(DataFrame* data_frame) = 0;

  virtual bool IsEnabled() const = 0;;

  virtual std::string Name() const = 0;

 private:
  DISALLOW_COPY_AND_ASSIGN(BaseLidarDetector);
};  // class BaseLidarDetector

PERCEPTION_REGISTER_REGISTERER(BaseLidarDetector);
#define PERCEPTION_REGISTER_LIDARDETECTOR(name) \
  PERCEPTION_REGISTER_CLASS(BaseLidarDetector, name)

}  // namespace lidar
}  // namespace perception
}  // namespace apollo
