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

#include "modules/perception/pointcloud_map_based_roi/roi_filter/roi_service_filter/roi_service_filter.h"

#include "modules/perception/common/lidar/common/lidar_point_label.h"
#include "modules/perception/common/lidar/scene_manager/scene_manager.h"

namespace apollo {
namespace perception {
namespace lidar {

bool ROIServiceFilter::Init(const ROIFilterInitOptions& options) {
  roi_service_ = std::dynamic_pointer_cast<ROIService>(
      SceneManager::Instance().Service("ROIService"));
  if (roi_service_ == nullptr) {
    AERROR << "ROi service is nullptr, Init scene manager first !";
    return false;
  }
  return true;
}

bool ROIServiceFilter::Filter(const ROIFilterOptions& options,
                              LidarFrame* frame) {
  if (frame == nullptr || frame->world_cloud == nullptr) {
    AERROR << "Frame is nullptr.";
    return false;
  }
  roi_service_->GetServiceContentCopy(&roi_service_content_);
  if (!roi_service_content_.IsServiceReady()) {
    AERROR << "service is not ready.";
    return false;
  }

  frame->roi_indices.indices.clear();
  frame->roi_indices.indices.reserve(frame->world_cloud->size());
  for (size_t i = 0; i < frame->world_cloud->size(); ++i) {
    auto& pt = frame->world_cloud->at(i);
    Eigen::Vector3d world_point(pt.x, pt.y, pt.z);
    if (roi_service_->QueryIsPointInROI(world_point, roi_service_content_)) {
      frame->roi_indices.indices.push_back(static_cast<int>(i));
      frame->cloud->mutable_points_label()->at(i) =
          static_cast<uint8_t>(LidarPointLabel::ROI);
    }
  }
  return true;
}

}  // namespace lidar
}  // namespace perception
}  // namespace apollo
