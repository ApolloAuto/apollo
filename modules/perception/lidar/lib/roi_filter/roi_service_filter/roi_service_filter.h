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
#include "modules/perception/lidar/lib/scene_manager/roi_service/roi_service.h"

namespace apollo {
namespace perception {
namespace lidar {

class ROIServiceFilter : public BaseROIFilter {
 public:
  ROIServiceFilter() : BaseROIFilter() {}
  ~ROIServiceFilter() = default;

  bool Init(const ROIFilterInitOptions& options) override;

  std::string Name() const override { return "ROIServiceFilter"; }

  bool Filter(const ROIFilterOptions& options, LidarFrame* frame) override;

 private:
  ROIServicePtr roi_service_ = nullptr;
  ROIServiceContent roi_service_content_;
};

}  // namespace lidar
}  // namespace perception
}  // namespace apollo
