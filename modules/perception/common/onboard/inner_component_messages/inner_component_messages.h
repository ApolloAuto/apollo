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

#include "cyber/cyber.h"
#include "modules/perception/common/base/frame.h"
#include "modules/perception/common/base/hdmap_struct.h"
#include "modules/perception/common/base/impending_collision_edge.h"
#include "modules/common_msgs/perception_msgs/perception_obstacle.pb.h"

namespace apollo {
namespace perception {
namespace onboard {

enum class ProcessStage {
  LIDAR_PREPROCESS = 0,
  LIDAR_DETECTION = 1,
  LIDAR_RECOGNITION = 2,
  STEREO_CAMERA_DETECTION = 3,
  MONOCULAR_CAMERA_DETECTION = 4,
  LONG_RANGE_RADAR_DETECTION = 5,
  SHORT_RANGE_RADAR_DETECTION = 6,
  ULTRASONIC_DETECTION = 7,
  SENSOR_FUSION = 8,
  UNKNOWN_STAGE = 9,
  PROCESSSTAGE_COUNT = 10
};

class Descriptor {
 public:
  std::string full_name() { return "name"; }
};

class SensorFrameMessage {
 public:
  SensorFrameMessage() { type_name_ = "SensorFrameMessage"; }
  ~SensorFrameMessage() = default;
  std::string GetTypeName() { return type_name_; }
  SensorFrameMessage* New() const { return new SensorFrameMessage; }

 public:
  apollo::common::ErrorCode error_code_ = apollo::common::ErrorCode::OK;

  std::string sensor_id_;
  double timestamp_ = 0.0;
  uint64_t lidar_timestamp_ = 0;
  uint32_t seq_num_ = 0;
  std::string type_name_;
  base::HdmapStructConstPtr hdmap_;

  base::FramePtr frame_;

  ProcessStage process_stage_ = ProcessStage::UNKNOWN_STAGE;
};

}  // namespace onboard
}  // namespace perception
}  // namespace apollo
