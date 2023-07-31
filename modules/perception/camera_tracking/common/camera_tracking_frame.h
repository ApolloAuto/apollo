/******************************************************************************
 * Copyright 2023 The Apollo Authors. All Rights Reserved.
 *
 * Licensed under the Apache License, Version 2.0 (the License);
 * you may not use this file except in compliance with the License.
 * You may obtain a copy of the License at
 *
 * http://www.apache.org/licenses/LICENSE-2.0
 *
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an AS IS BASIS,
 * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 * See the License for the specific language governing permissions and
 * limitations under the License.
 *****************************************************************************/
#pragma once

#include <memory>
#include <vector>

#include "modules/common/util/eigen_defs.h"
#include "modules/perception/common/base/blob.h"
#include "modules/perception/common/base/object_pool_types.h"
#include "modules/perception/common/camera/common/data_provider.h"
#include "modules/perception/common/lib/interface/base_calibration_service.h"

namespace apollo {
namespace perception {
namespace camera {

struct CameraTrackingFrame {
  // Frame sequence id
  std::uint64_t frame_id;
  // Timestamp
  double timestamp;
  // Image
  std::shared_ptr<camera::DataProvider> data_provider;
  // Detection result
  std::vector<base::ObjectPtr> detected_objects;
  // proposed_objects
  std::vector<base::ObjectPtr> proposed_objects;
  // tracked objects
  std::vector<base::ObjectPtr> tracked_objects;
  // Camera Intrinstic
  Eigen::Matrix3f camera_k_matrix = Eigen::Matrix3f::Identity();
  // Tracking feature blob
  std::shared_ptr<base::Blob<float>> track_feature_blob = nullptr;
  // appearance features for tracking
  std::shared_ptr<base::Blob<float>> feature_blob = nullptr;
  // calibration service
  BaseCalibrationService *calibration_service = nullptr;
  // narrow to obstacle projected_matrix
  Eigen::Matrix3d project_matrix = Eigen::Matrix3d::Identity();
  // camera to world pose
  Eigen::Affine3d camera2world_pose = Eigen::Affine3d::Identity();

  EIGEN_MAKE_ALIGNED_OPERATOR_NEW
};

}  // namespace camera
}  // namespace perception
}  // namespace apollo
