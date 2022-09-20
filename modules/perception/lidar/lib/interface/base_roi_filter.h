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

#include "cyber/common/macros.h"
#include "modules/perception/lib/registerer/registerer.h"
#include "modules/perception/lidar/common/lidar_frame.h"
#include "modules/perception/pipeline/stage.h"

namespace apollo {
namespace perception {
namespace lidar {

struct ROIFilterInitOptions {};

struct ROIFilterOptions {};

class BaseROIFilter : public pipeline::Stage {
 public:
  using DataFrame = pipeline::DataFrame;
  using Plugin = pipeline::Plugin;
  using PluginType = pipeline::PluginType;
  using StageConfig = pipeline::StageConfig;

 public:
  BaseROIFilter() = default;

  virtual ~BaseROIFilter() = default;

  virtual bool Init(
      const ROIFilterInitOptions& options = ROIFilterInitOptions()) = 0;

  // @brief: filter roi points from point cloud
  // @param [in]: options
  // @param [in/out]: frame
  // lidar2world_pose and hdmap_struct should be valid,
  // roi_indices should be filled, required
  // label field of point cloud can be filled, optional
  virtual bool Filter(const ROIFilterOptions& options, LidarFrame* frame) = 0;

  virtual bool Init(const StageConfig& stage_config) = 0;

  virtual bool Process(DataFrame* data_frame) = 0;

  virtual bool IsEnabled() const = 0;;

  virtual std::string Name() const = 0;

 private:
  DISALLOW_COPY_AND_ASSIGN(BaseROIFilter);
};  // class BaseROIFilter

PERCEPTION_REGISTER_REGISTERER(BaseROIFilter);
#define PERCEPTION_REGISTER_ROIFILTER(name) \
  PERCEPTION_REGISTER_CLASS(BaseROIFilter, name)

}  // namespace lidar
}  // namespace perception
}  // namespace apollo
