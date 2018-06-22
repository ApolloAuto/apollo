/******************************************************************************
 * Copyright 2017 The Apollo Authors. All Rights Reserved.
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

#ifndef MODULES_PERCEPTION_OBSTACLE_ONBOARD_SENSOR_RAW_FRAME_H_
#define MODULES_PERCEPTION_OBSTACLE_ONBOARD_SENSOR_RAW_FRAME_H_

#include "Eigen/Core"
#include "modules/drivers/proto/conti_radar.pb.h"
#include "modules/perception/obstacle/base/types.h"
#include "modules/perception/obstacle/radar/interface/base_radar_detector.h"

namespace apollo {
namespace perception {

class SensorRawFrame {
 public:
  SensorRawFrame()
      : sensor_type_(SensorType::UNKNOWN_SENSOR_TYPE),
        timestamp_(0.0),
        pose_(Eigen::Matrix4d::Identity()) {}
  virtual ~SensorRawFrame() {}

 public:
  SensorType sensor_type_;
  double timestamp_;
  Eigen::Matrix4d pose_;
};

class VelodyneRawFrame : public SensorRawFrame {
 public:
  VelodyneRawFrame() {}
  ~VelodyneRawFrame() {}

 public:
  pcl_util::PointCloudPtr cloud_;
};

class RadarRawFrame : public SensorRawFrame {
 public:
  RadarRawFrame() {}
  ~RadarRawFrame() {}

 public:
  ContiRadar raw_obstacles_;
  Eigen::Vector3f car_linear_speed_;
};

}  // namespace perception
}  // namespace apollo

#endif  // MODULES_PERCEPTION_OBSTACLE_ONBOARD_SENSOR_RAW_FRAME_H_
