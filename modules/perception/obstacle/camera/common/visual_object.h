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

#ifndef MODULES_PERCEPTION_OBSTACLE_CAMERA_COMMON_VISUAL_OBJECT_H
#define MODULES_PERCEPTION_OBSTACLE_CAMERA_COMMON_VISUAL_OBJECT_H

#include <Eigen/Core>
#include <memory>
#include <vector>

#include "modules/perception/obstacle/base/types.h"

namespace apollo {
namespace perception {

struct alignas(16) VisualObject {
  // Per-frame object id, assigned from detection
  int id = 0;
  // Confidence of objectness, ranging as [0, 1]
  float score = 0.0f;

  // [pixel] 2D bounding box
  // upper-left corner: x1, y1
  Eigen::Vector2f upper_left;
  // lower-right corner: x2, y2
  Eigen::Vector2f lower_right;

  // 2D bounding box truncation ratio, for out-of-image objects
  float trunc_width = 0.0f;
  float trunc_height = 0.0f;

  // Object type from detection
  ObjectType type = UNKNOWN;
  // Probability of each object type
  std::vector<float> type_probs;

  // ROI pooling feature from layers of deep learning detection model
  std::vector<float> dl_roi_feature;

  // [meter] physical size of 3D oriented bounding box
  // length is the size in the main direction
  float length = 0.0f;
  float width = 0.0f;
  float height = 0.0f;

  // [radian] observation angle of object, ranging as [-pi, pi]
  float alpha = 0.0f;
  // [radian] Rotation around the vertical axis, ranging as [-pi, pi]
  // the yaw angle, theta = 0.0f means direction = (1, 0, 0)
  float theta = 0.0f;
  // main direction
  Eigen::Vector3f direction = Eigen::Vector3f(1.0f, 0.0f, 0.0f);

  // [meter] physical center of the object, (cx, cy, cz)
  Eigen::Vector3f center = Eigen::Vector3f::Zero();
  // [meter] distance to object physical center from camera origin
  float distance = 0.0f;
  // [meter / second] physical velocity of the object, (vx, vy, vz)
  Eigen::Vector3f velocity = Eigen::Vector3f::Zero();

  // globally unique tracking id for camera visual objects
  int track_id = 0;
  // [second] age of the tracked object
  float track_age = 0.0f;
  // [second] the last observed timestamp
  float last_track_timestamp = 0.0f;
};

typedef std::shared_ptr<VisualObject> VisualObjectPtr;
typedef std::shared_ptr<const VisualObject> VisualObjectConstPtr;

}  // namespace perception
}  // namespace apollo

#endif  // MODULES_PERCEPTION_OBSTACLE_CAMERA_COMMON_VISUAL_OBJECT_H
