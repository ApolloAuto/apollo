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
#ifndef PERCEPTION_LIDAR_LIB_INTERFACE_BASE_SEGMENTATION_H_
#define PERCEPTION_LIDAR_LIB_INTERFACE_BASE_SEGMENTATION_H_

#include <string>
#include <vector>

#include "modules/perception/lib/registerer/registerer.h"
#include "modules/perception/lidar/common/lidar_frame.h"

namespace apollo {
namespace perception {
namespace lidar {

struct SegmentationInitOptions {};

struct SegmentationOptions {};

class BaseSegmentation {
 public:
  BaseSegmentation() = default;

  virtual ~BaseSegmentation() = default;

  virtual bool Init(
      const SegmentationInitOptions& options = SegmentationInitOptions()) = 0;

  // @brief: segment point cloud and get objects.
  // @param [in]: options
  // @param [in/out]: frame
  // segmented_objects should be filled, required,
  // label field of point cloud can be filled, optional,
  virtual bool Segment(const SegmentationOptions& options,
                       LidarFrame* frame) = 0;

  virtual std::string Name() const = 0;

  BaseSegmentation(const BaseSegmentation&) = delete;
  BaseSegmentation& operator=(const BaseSegmentation&) = delete;
};  // class BaseSegmentation

PERCEPTION_REGISTER_REGISTERER(BaseSegmentation);
#define PERCEPTION_REGISTER_SEGMENTATION(name) \
  PERCEPTION_REGISTER_CLASS(BaseSegmentation, name)

}  // namespace lidar
}  // namespace perception
}  // namespace apollo

#endif  // PERCEPTION_LIDAR_LIB_INTERFACE_BASE_SEGMENTATION_H_
