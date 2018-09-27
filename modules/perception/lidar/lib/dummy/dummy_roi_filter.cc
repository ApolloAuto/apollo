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

#include "modules/perception/lidar/lib/dummy/dummy_roi_filter.h"

namespace apollo {
namespace perception {
namespace lidar {

bool DummyROIFilter::Init(const ROIFilterInitOptions& options) { return true; }

bool DummyROIFilter::Filter(const ROIFilterOptions& options,
                            LidarFrame* frame) {
  if (!frame) {
    return false;
  }
  if (!frame->cloud) {
    return false;
  }
  frame->roi_indices.indices.resize(frame->cloud->size());
  std::iota(frame->roi_indices.indices.begin(),
            frame->roi_indices.indices.end(), 0);
  return true;
}

PERCEPTION_REGISTER_ROIFILTER(DummyROIFilter);

}  // namespace lidar
}  // namespace perception
}  // namespace apollo
