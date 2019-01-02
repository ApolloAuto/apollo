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

#include "modules/perception/lidar/lib/interface/base_roi_filter.h"

namespace apollo {
namespace perception {
namespace lidar {

class DummyROIFilter : public BaseROIFilter {
 public:
  DummyROIFilter() = default;

  virtual ~DummyROIFilter() = default;

  bool Init(
      const ROIFilterInitOptions& options = ROIFilterInitOptions()) override;

  // @brief: filter roi points from point cloud
  // @param [in]: options
  // @param [in/out]: frame
  // lidar2world_pose and hdmap_struct should be valid,
  // roi_indices should be filled, required
  // label field of point cloud can be filled, optional
  bool Filter(const ROIFilterOptions& options, LidarFrame* frame) override;

  std::string Name() const override { return "DummyROIFilter"; }
};  // class DummyROIFilter

}  // namespace lidar
}  // namespace perception
}  // namespace apollo
