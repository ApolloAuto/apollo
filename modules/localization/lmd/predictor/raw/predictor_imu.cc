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

#include <iomanip>

#include "modules/localization/lmd/predictor/raw/predictor_imu.h"

#include "modules/common/log.h"

namespace apollo {
namespace localization {

using apollo::common::Status;

PredictorImu::PredictorImu(double memory_cycle_sec)
    : Predictor(memory_cycle_sec), raw_imu_(memory_cycle_sec) {
  name_ = kPredictorImuName;
  on_adapter_thread_ = true;
}

PredictorImu::~PredictorImu() {}

bool PredictorImu::UpdateImu(const CorrectedImu& imu) {
  if (!imu.has_header() || !imu.header().has_timestamp_sec() ||
      !imu.has_imu()) {
    AERROR << "Message has not some feilds";
    return false;
  }

  auto timestamp_sec = imu.header().timestamp_sec();
  if (!raw_imu_.Push(timestamp_sec, imu.imu())) {
    AWARN << std::setprecision(15)
          << "Failed push pose to list, with timestamp[" << timestamp_sec
          << "]";
    return false;
  }

  return WindowFilter(timestamp_sec);
}

bool PredictorImu::Updateable() const {
  return !predicted_.empty() && predicted_.Newer(latest_timestamp_sec_);
}

Status PredictorImu::Update() {
  if (!predicted_.empty()) {
    auto latest = predicted_.Latest();
    latest_timestamp_sec_ = latest->first;
  }
  return Status::OK();
}

bool PredictorImu::WindowFilter(double timestamp_sec) {
  Pose pose;
  pose.mutable_linear_acceleration()->set_x(0.0);
  pose.mutable_linear_acceleration()->set_y(0.0);
  pose.mutable_linear_acceleration()->set_z(0.0);
  pose.mutable_angular_velocity()->set_x(0.0);
  pose.mutable_angular_velocity()->set_y(0.0);
  pose.mutable_angular_velocity()->set_z(0.0);

  constexpr double kWindowSize = 0.20;
  constexpr double kSamplingInterval = 0.01;
  std::size_t count = 0;
  for (auto t = timestamp_sec; t >= timestamp_sec - kWindowSize;
       t -= kSamplingInterval) {
    Pose sample;
    raw_imu_.FindNearestPose(t, &sample);
    pose.mutable_linear_acceleration()->set_x(pose.linear_acceleration().x() +
                                              sample.linear_acceleration().x());
    pose.mutable_linear_acceleration()->set_y(pose.linear_acceleration().y() +
                                              sample.linear_acceleration().y());
    pose.mutable_linear_acceleration()->set_z(pose.linear_acceleration().z() +
                                              sample.linear_acceleration().z());
    pose.mutable_angular_velocity()->set_x(pose.angular_velocity().x() +
                                           sample.angular_velocity().x());
    pose.mutable_angular_velocity()->set_y(pose.angular_velocity().y() +
                                           sample.angular_velocity().y());
    pose.mutable_angular_velocity()->set_z(pose.angular_velocity().z() +
                                           sample.angular_velocity().z());

    count++;
  }

  pose.mutable_linear_acceleration()->set_x(pose.linear_acceleration().x() /
                                            count);
  pose.mutable_linear_acceleration()->set_y(pose.linear_acceleration().y() /
                                            count);
  pose.mutable_linear_acceleration()->set_z(pose.linear_acceleration().z() /
                                            count);
  pose.mutable_angular_velocity()->set_x(pose.angular_velocity().x() / count);
  pose.mutable_angular_velocity()->set_y(pose.angular_velocity().y() / count);
  pose.mutable_angular_velocity()->set_z(pose.angular_velocity().z() / count);

  if (!predicted_.Push(timestamp_sec, pose)) {
    AWARN << std::setprecision(15)
          << "Failed push pose to list, with timestamp[" << timestamp_sec
          << "]";
    return false;
  }

  return true;
}

}  // namespace localization
}  // namespace apollo
