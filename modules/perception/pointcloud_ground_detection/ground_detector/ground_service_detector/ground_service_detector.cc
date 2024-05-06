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

#include "modules/perception/pointcloud_ground_detection/ground_detector/ground_service_detector/ground_service_detector.h"

#include "cyber/common/file.h"

#include "modules/perception/common/util.h"
#include "modules/perception/common/lidar/common/lidar_point_label.h"
#include "modules/perception/pointcloud_ground_detection/ground_detector/proto/ground_service_detector_config.pb.h"

namespace apollo {
namespace perception {
namespace lidar {

bool GroundServiceDetector::Init(const GroundDetectorInitOptions& options) {
  std::string config_file =
      GetConfigFile(options.config_path, options.config_file);
  GroundServiceDetectorConfig config;
  ACHECK(cyber::common::GetProtoFromFile(config_file, &config));
  ground_threshold_ = config.ground_threshold();

  ground_service_ = std::dynamic_pointer_cast<GroundService>(
      SceneManager::Instance().Service("GroundService"));
  if (ground_service_ == nullptr) {
    AERROR << "Ground service is nullptr, Init scene manager first !";
    return false;
  }
  return true;
}

bool GroundServiceDetector::Detect(const GroundDetectorOptions& options,
                                   LidarFrame* frame) {
  if (frame == nullptr || frame->world_cloud == nullptr) {
    AERROR << "Frame is nullptr.";
    return false;
  }
  ground_service_->GetServiceContentCopy(&ground_service_content_);
  if (!ground_service_content_.IsServiceReady()) {
    AERROR << "service is not ready.";
    return false;
  }
  auto& cloud = frame->world_cloud;
  auto& non_ground_indices = frame->non_ground_indices;
  non_ground_indices.indices.clear();
  non_ground_indices.indices.reserve(cloud->size());
  for (size_t i = 0; i < cloud->size(); ++i) {
    auto& pt = cloud->at(i);
    Eigen::Vector3d world_point(pt.x, pt.y, pt.z);
    float dist = ground_service_->QueryPointToGroundDistance(
        world_point, ground_service_content_);
    frame->cloud->mutable_points_height()->at(i) = dist;
    frame->world_cloud->mutable_points_height()->at(i) = dist;
    if (dist > ground_threshold_) {
      non_ground_indices.indices.push_back(static_cast<int>(i));
    } else {
      frame->cloud->mutable_points_label()->at(i) =
          static_cast<uint8_t>(LidarPointLabel::GROUND);
    }
  }
  return true;
}

PERCEPTION_REGISTER_GROUNDDETECTOR(GroundServiceDetector);

}  // namespace lidar
}  // namespace perception
}  // namespace apollo
