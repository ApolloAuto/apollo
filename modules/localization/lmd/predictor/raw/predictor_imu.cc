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

using apollo::common::Point3D;
using apollo::common::Status;

namespace {
constexpr double kSamplingInterval = 0.01;
}  // namespace

PredictorImu::PredictorImu(double memory_cycle_sec)
    : Predictor(memory_cycle_sec), raw_imu_(memory_cycle_sec) {
  name_ = kPredictorImuName;
  on_adapter_thread_ = true;
}

PredictorImu::~PredictorImu() {}

bool PredictorImu::UpdateImu(const CorrectedImu& imu) {
  if (!imu.has_header() || !imu.header().has_timestamp_sec() ||
      !imu.has_imu() || !imu.imu().has_linear_acceleration() ||
      !imu.imu().has_angular_velocity()) {
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

  return true;
}

bool PredictorImu::Updateable() const {
  if (predicted_.empty()) {
    return !raw_imu_.empty();
  } else {
    return !predicted_.Newer(raw_imu_.Latest()->first - kSamplingInterval);
  }
}

Status PredictorImu::Update() {
  ResamplingFilter();
  return Status::OK();
}

void PredictorImu::ResamplingFilter() {
  auto latest_it = predicted_.Latest();
  if (latest_it == predicted_.end()) {
    predicted_.Push(raw_imu_.begin()->first, raw_imu_.begin()->second);
  } else {
    auto timestamp_sec = latest_it->first + kSamplingInterval;
    Pose pose;
    raw_imu_.FindNearestPose(timestamp_sec, &pose);
    predicted_.Push(timestamp_sec, pose);
  }
}

void PredictorImu::LPFilter() {
  double timestamp_sec;
  Point3D x_0, x_1, x_2;
  Point3D y_0, y_1, y_2;
  Pose pose;

  auto latest_it = predicted_.Latest();
  if (latest_it == predicted_.end()) {
    timestamp_sec = raw_imu_.begin()->first;

    x_0 = x_1 = x_2 = raw_imu_.begin()->second.linear_acceleration();
    y_1 = y_2 = x_2;
    pose = raw_imu_.begin()->second;
  } else {
    timestamp_sec = latest_it->first + kSamplingInterval;

    raw_imu_.FindNearestPose(timestamp_sec - 2.0 * kSamplingInterval, &pose);
    x_2 = pose.linear_acceleration();
    raw_imu_.FindNearestPose(timestamp_sec - kSamplingInterval, &pose);
    x_1 = pose.linear_acceleration();
    raw_imu_.FindNearestPose(timestamp_sec, &pose);
    x_0 = pose.linear_acceleration();

    if (predicted_.size() == 1) {
      y_2 = x_2;
    } else {
      auto it_2 = latest_it;
      it_2--;
      y_2 = it_2->second.linear_acceleration();
    }
    y_1 = latest_it->second.linear_acceleration();
  }

  //  two-order Butterworth filter
  auto but_filter = [](double x_0, double x_1, double x_2, double y_1,
                       double y_2) {
    constexpr double kBZ[3] = {0.024472, 0.048943, 0.024472};
    constexpr double kAZ[3] = {1.2185, -1.9021, 0.78149};
    return (kBZ[0] * x_0 + kBZ[1] * x_1 + kBZ[2] * x_2 - kAZ[1] * y_1 -
            kAZ[2] * y_2) /
           kAZ[0];
  };

  y_0.set_x(but_filter(x_0.x(), x_1.x(), x_2.x(), y_1.x(), y_2.x()));
  y_0.set_y(but_filter(x_0.y(), x_1.y(), x_2.y(), y_1.y(), y_2.y()));
  y_0.set_z(but_filter(x_0.z(), x_1.z(), x_2.z(), y_1.z(), y_2.z()));
  pose.mutable_linear_acceleration()->CopyFrom(y_0);
  predicted_.Push(timestamp_sec, pose);
}

}  // namespace localization
}  // namespace apollo
