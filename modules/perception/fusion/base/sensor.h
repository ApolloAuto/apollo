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

#include <deque>
#include <memory>
#include <string>
#include <vector>

#include "gtest/gtest_prod.h"

#include "modules/perception/base/sensor_meta.h"
#include "modules/perception/fusion/base/base_forward_declaration.h"
#include "modules/perception/fusion/base/sensor_frame.h"

namespace apollo {
namespace perception {
namespace fusion {

class Sensor {
 public:
  Sensor() = delete;

  explicit Sensor(const base::SensorInfo& sensor_info)
      : sensor_info_(sensor_info) {}

  // query frames whose time stamp is in range
  // (_latest_fused_time_stamp, time_stamp]
  void QueryLatestFrames(double timestamp, std::vector<SensorFramePtr>* frames);

  // query latest frame whose time stamp is in range
  // (_latest_fused_time_stamp, time_stamp]
  SensorFramePtr QueryLatestFrame(double timestamp);

  bool GetPose(double timestamp, Eigen::Affine3d* pose) const;

  // Getter
  inline std::string GetSensorId() const { return sensor_info_.name; }

  // Getter
  inline base::SensorType GetSensorType() const { return sensor_info_.type; }

  void AddFrame(const base::FrameConstPtr& frame_ptr);

  inline static void SetMaxCachedFrameNumber(size_t number) {
    kMaxCachedFrameNum = number;
  }

  void SetLatestQueryTimestamp(const double latest_query_timestamp) {
    latest_query_timestamp_ = latest_query_timestamp;
  }

 private:
  FRIEND_TEST(SensorTest, test);

  base::SensorInfo sensor_info_;

  double latest_query_timestamp_ = 0.0;

  std::deque<SensorFramePtr> frames_;

  static size_t kMaxCachedFrameNum;
};

}  // namespace fusion
}  // namespace perception
}  // namespace apollo
