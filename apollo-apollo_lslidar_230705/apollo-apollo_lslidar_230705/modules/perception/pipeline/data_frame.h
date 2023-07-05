/******************************************************************************
 * Copyright 2022 The Apollo Authors. All Rights Reserved.
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

#include "modules/perception/camera/common/camera_frame.h"
#include "modules/perception/fusion/base/fusion_frame.h"
#include "modules/perception/lidar/common/lidar_frame.h"

namespace apollo {
namespace perception {
namespace pipeline {

struct DataFrame {
  camera::CameraFrame* camera_frame;
  lidar::LidarFrame* lidar_frame;
  // RadarFrame* radar_frame;

  fusion::FusionFrame* fusion_frame;
};

}  // namespace pipeline
}  // namespace perception
}  // namespace apollo
