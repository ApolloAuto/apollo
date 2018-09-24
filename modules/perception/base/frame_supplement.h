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
#ifndef PERCEPTION_BASE_FRAME_SUPPLEMENT_H_
#define PERCEPTION_BASE_FRAME_SUPPLEMENT_H_
#include <memory>
#include "modules/perception/base/blob.h"
#include "modules/perception/base/image.h"
#include "modules/perception/base/impending_collision_edge.h"
#include "modules/perception/base/point_cloud_types.h"

namespace apollo {
namespace perception {
namespace base {
// sensor-specific frame supplements: Lidar, Radar, Camera
struct alignas(16) LidarFrameSupplement {
  // @brief valid only when on_use = true
  bool on_use = false;

  // @brief only reference of the original cloud in lidar coordinate system
  PointFCloudPtr cloud_ptr = nullptr;

  void Reset() {
    on_use = false;
    cloud_ptr = nullptr;
  }
};

typedef std::shared_ptr<LidarFrameSupplement> LidarFrameSupplementPtr;
typedef std::shared_ptr<const LidarFrameSupplement>
    LidarFrameSupplementConstPtr;

struct alignas(16) RadarFrameSupplement {
  // @brief valid only when on_use = true
  bool on_use = false;
  void Reset() { on_use = false; }
};
typedef std::shared_ptr<RadarFrameSupplement> RadarFrameSupplementPtr;
typedef std::shared_ptr<const RadarFrameSupplement>
    RadarFrameSupplementConstPtr;

struct alignas(16) CameraFrameSupplement {
  // @brief valid only when on_use = true
  bool on_use = false;

  // @brief only refrence of the image data
  Image8UPtr image_ptr = nullptr;

  // TODO(guiyilin): modify interfaces of visualizer, use Image8U
  std::shared_ptr<Blob<uint8_t>> image_blob = nullptr;

  void Reset() {
    on_use = false;
    image_ptr = nullptr;
    image_blob = nullptr;
  }
};

typedef std::shared_ptr<CameraFrameSupplement> CameraFrameSupplementPtr;
typedef std::shared_ptr<const CameraFrameSupplement>
    CameraFrameSupplementConstPtr;

struct alignas(16) UltrasonicFrameSupplement {
  // @brief valid only when on_use = true
  bool on_use = false;

  // @brief only refrence of the image data
  ImpendingCollisionEdgesPtr impending_collision_edges_ptr = nullptr;

  void Reset() {
    on_use = false;
    impending_collision_edges_ptr = nullptr;
  }
};

typedef std::shared_ptr<UltrasonicFrameSupplement> UltrasonicFrameSupplementPtr;
typedef std::shared_ptr<const UltrasonicFrameSupplement>
    UltrasonicFrameSupplementConstPtr;

}  // namespace base
}  // namespace perception
}  // namespace apollo

#endif  // PERCEPTION_BASE_FRAME_SUPPLEMENT_H_
