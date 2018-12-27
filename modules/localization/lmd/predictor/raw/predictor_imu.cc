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
#include <vector>

#include "modules/common/time/time_util.h"
#include "modules/localization/common/localization_gflags.h"
#include "modules/localization/lmd/predictor/raw/predictor_imu.h"

#include "modules/common/log.h"

namespace apollo {
namespace localization {

using apollo::common::Point3D;
using apollo::common::Status;
using apollo::common::time::TimeUtil;
using apollo::drivers::gnss::Imu;

namespace {
constexpr double kSamplingInterval = 0.01;
}  // namespace

PredictorImu::PredictorImu(double memory_cycle_sec)
    : Predictor(memory_cycle_sec), raw_imu_(memory_cycle_sec) {
  name_ = kPredictorImuName;
  on_adapter_thread_ = true;
  constexpr double kCutoffFreq = 20.0;
  InitLPFilter(kCutoffFreq);

  constexpr double kLCutoffFreq = 0.6;
  InitializeFilters(kSamplingInterval, kLCutoffFreq);
}

PredictorImu::~PredictorImu() {}

bool PredictorImu::UpdateImu(const CorrectedImu& imu) {
  if (!imu.has_header() || !imu.header().has_timestamp_sec() ||
      !imu.has_imu() || !imu.imu().has_linear_acceleration() ||
      !imu.imu().has_angular_velocity()) {
    AERROR << "Message has not some feilds";
    return false;
  }

  auto pose = imu.imu();
  pose.mutable_linear_acceleration()->CopyFrom(pose.linear_acceleration());

  auto timestamp_sec = imu.header().timestamp_sec();
  if (!raw_imu_.Push(timestamp_sec, pose)) {
    AWARN << std::setprecision(15)
          << "Failed push pose to list, with timestamp[" << timestamp_sec
          << "]";
    return false;
  }

  return true;
}

bool PredictorImu::UpdateRawImu(const Imu& imu) {
  // TODO(all): skipped
  if (true) {
    return true;
  }

  if (!imu.has_measurement_time() || !imu.has_linear_acceleration() ||
      !imu.has_angular_velocity()) {
    AERROR << "Message has not some feilds";
    return false;
  }

  auto timestamp_sec = TimeUtil::Gps2unix(imu.measurement_time());
  Pose pose;
  pose.mutable_linear_acceleration()->set_x(imu.linear_acceleration().x() *
                                            FLAGS_imu_rate);
  pose.mutable_linear_acceleration()->set_y(imu.linear_acceleration().y() *
                                            FLAGS_imu_rate);
  pose.mutable_linear_acceleration()->set_z(imu.linear_acceleration().z() *
                                            FLAGS_imu_rate);
  pose.mutable_linear_acceleration()->set_z(0.0);
  pose.mutable_angular_velocity()->set_x(imu.angular_velocity().x() *
                                         FLAGS_imu_rate);
  pose.mutable_angular_velocity()->set_y(imu.angular_velocity().y() *
                                         FLAGS_imu_rate);
  pose.mutable_angular_velocity()->set_z(imu.angular_velocity().z() *
                                         FLAGS_imu_rate);
  if (!raw_imu_.Push(timestamp_sec, pose)) {
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
  // LPFilter();
  DigitalFilter();
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

void PredictorImu::InitLPFilter(double cutoff_freq) {
  auto omega = 2.0 * M_PI * cutoff_freq * kSamplingInterval;
  auto sin_o = std::sin(omega);
  auto cos_o = std::cos(omega);
  auto alpha = sin_o / (2.0 / std::sqrt(2));
  iir_filter_bz_[0] = (1.0 - cos_o) / 2.0;
  iir_filter_bz_[1] = 1.0 - cos_o;
  iir_filter_bz_[2] = (1.0 - cos_o) / 2.0;
  iir_filter_az_[0] = 1.0 + alpha;
  iir_filter_az_[1] = -2.0 * cos_o;
  iir_filter_az_[2] = 1.0 - alpha;
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
  auto but_filter = [&](double x_0, double x_1, double x_2, double y_1,
                        double y_2) {
    const auto& bz = iir_filter_bz_;
    const auto& az = iir_filter_az_;
    return (bz[0] * x_0 + bz[1] * x_1 + bz[2] * x_2 - az[1] * y_1 -
            az[2] * y_2) /
           az[0];
  };

  y_0.set_x(but_filter(x_0.x(), x_1.x(), x_2.x(), y_1.x(), y_2.x()));
  y_0.set_y(but_filter(x_0.y(), x_1.y(), x_2.y(), y_1.y(), y_2.y()));
  y_0.set_z(but_filter(x_0.z(), x_1.z(), x_2.z(), y_1.z(), y_2.z()));
  pose.mutable_linear_acceleration()->CopyFrom(y_0);
  predicted_.Push(timestamp_sec, pose);
}

void PredictorImu::InitializeFilters(const double ts,
                                     const double cutoff_freq) {
  // Low pass filter
  std::vector<double> den(3, 0.0);
  std::vector<double> num(3, 0.0);
  common::LpfCoefficients(ts, cutoff_freq, &den, &num);
  digital_filter_x_.set_coefficients(den, num);
  digital_filter_y_.set_coefficients(den, num);
}

void PredictorImu::DigitalFilter() {
  auto latest_it = predicted_.Latest();
  if (latest_it != predicted_.end()) {
    Pose pose;
    pose.CopyFrom(latest_it->second);
    auto timestamp_sec = latest_it->first;
    auto linear_acc = pose.linear_acceleration();

    linear_acc.set_x(digital_filter_x_.Filter(linear_acc.x()));
    linear_acc.set_y(digital_filter_y_.Filter(linear_acc.y()));

    pose.mutable_linear_acceleration()->CopyFrom(linear_acc);

    predicted_.Pop();
    predicted_.Push(timestamp_sec, pose);
  }
}

}  // namespace localization
}  // namespace apollo
