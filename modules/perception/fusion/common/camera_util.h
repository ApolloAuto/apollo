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

#include <memory>
#include <vector>

#include "modules/perception/base/camera.h"
#include "modules/perception/base/object.h"
#include "modules/perception/common/geometry/common.h"
#include "modules/perception/fusion/base/sensor_object.h"

namespace apollo {
namespace perception {
namespace fusion {

// @brief: get object eight vertices
// @param [in]: object
// @param [in/out]: vertices
void GetObjectEightVertices(std::shared_ptr<const base::Object> obj,
                            std::vector<Eigen::Vector3d>* vertices);

template <typename VectorType>
bool IsPtInFrustum(const VectorType& pt, double width, double height) {
  if (pt[0] < 0 || pt[0] > width || pt[1] < 0 || pt[1] > height) {
    return false;
  }
  return true;
}

template <typename Type>
Type CalculateAugmentedIOUBBox(const base::BBox2D<Type>& box1,
                               const base::BBox2D<Type>& box2,
                               const Type& augmented_buffer) {
  base::BBox2D<Type> augmented_box1 = box1;
  augmented_box1.xmin -= augmented_buffer;
  augmented_box1.ymin -= augmented_buffer;
  augmented_box1.xmax += augmented_buffer;
  augmented_box1.ymax += augmented_buffer;
  base::BBox2D<Type> augmented_box2 = box2;
  augmented_box2.xmin -= augmented_buffer;
  augmented_box2.ymin -= augmented_buffer;
  augmented_box2.xmax += augmented_buffer;
  augmented_box2.ymax += augmented_buffer;
  Type augmented_iou = common::CalculateIOUBBox(augmented_box1, augmented_box2);
  return augmented_iou;
}

// @brief: project 3d point to 2d point in camera
// @param [in]: pt3d
// @param [in]: world2camera_pose
// @param [in]: camera_model
// @param [in/out]: pt2d
// @return: true if local 3d point z > 0, else false
bool Pt3dToCamera2d(const Eigen::Vector3d& pt3d,
                    const Eigen::Matrix4d& world2camera_pose,
                    base::BaseCameraModelPtr camera_model,
                    Eigen::Vector2d* pt2d);

// @brief: whether object eight vertices' local 3d point z  < 0
// @param [in]: obj
// @param [in]: world2camera_pose
// @param [in]: camera_model
bool IsObjectEightVerticesAllBehindCamera(
    const std::shared_ptr<const base::Object>& obj,
    const Eigen::Matrix4d& world2camera_pose,
    base::BaseCameraModelPtr camera_model);

// @brief: compute the ratio of object in camera view
// @param [in]: sensor_object
// @param [in]: camera_model
// @param [in]: camera_sensor2world_pose
// @param [in]: camera_timestamp
// @param [in]: camera_max_dist, the max distace can be detected by camera
// @param [in]: motion_compenseation
// @param [in]: all_in, if all_in is true, just return 0.0 or 1.0
float ObjectInCameraView(SensorObjectConstPtr sensor_object,
                         base::BaseCameraModelPtr camera_model,
                         const Eigen::Affine3d& camera_sensor2world_pose,
                         double camera_ts, double camera_max_dist,
                         bool motion_compensation, bool all_in);

}  // namespace fusion
}  // namespace perception
}  // namespace apollo
