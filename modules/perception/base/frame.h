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

#include "Eigen/Dense"

#include "modules/perception/base/frame_supplement.h"
#include "modules/perception/base/object.h"
#include "modules/perception/base/object_types.h"
#include "modules/perception/base/sensor_meta.h"

namespace apollo {
namespace perception {
namespace base {

struct alignas(16) Frame {
  Frame() { sensor2world_pose.setIdentity(); }

  void Reset() {
    timestamp = 0.0;
    objects.clear();
    sensor2world_pose.setIdentity();
    sensor_info.Reset();
    lidar_frame_supplement.Reset();
    radar_frame_supplement.Reset();
    camera_frame_supplement.Reset();
  }
  // @brief sensor information
  SensorInfo sensor_info;

  double timestamp = 0.0;
  std::vector<std::shared_ptr<Object>> objects;
  Eigen::Affine3d sensor2world_pose;

  // sensor-specific frame supplements
  LidarFrameSupplement lidar_frame_supplement;
  RadarFrameSupplement radar_frame_supplement;
  CameraFrameSupplement camera_frame_supplement;
  UltrasonicFrameSupplement ultrasonic_frame_supplement;
};

typedef std::shared_ptr<Frame> FramePtr;
typedef std::shared_ptr<const Frame> FrameConstPtr;

}  // namespace base
}  // namespace perception
}  // namespace apollo
