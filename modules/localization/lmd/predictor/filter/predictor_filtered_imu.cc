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

#include <algorithm>
#include <iomanip>
#include <string>

#include "modules/localization/lmd/predictor/filter/predictor_filtered_imu.h"

#include "modules/common/log.h"

namespace apollo {
namespace localization {

using apollo::common::Point3D;
using apollo::common::Status;

namespace {
constexpr double kSamplingInterval = 0.01;

const std::string& PredictorImuName() {
  static const std::string name(kPredictorImuName);
  return name;
}
}  // namespace

PredictorFilteredImu::PredictorFilteredImu(double memory_cycle_sec)
    : Predictor(memory_cycle_sec), chassis_speed_(memory_cycle_sec) {
  name_ = kPredictorFilteredImuName;
  dep_predicteds_.emplace(PredictorImuName(), PoseList(memory_cycle_sec));
  on_adapter_thread_ = true;
}

PredictorFilteredImu::~PredictorFilteredImu() {}

bool PredictorFilteredImu::UpdateChassis(
    const apollo::canbus::Chassis& chassis) {
  if (!chassis.has_header() || !chassis.header().has_timestamp_sec() ||
      !chassis.has_speed_mps()) {
    AERROR << "Message has not some feilds";
    return false;
  }

  auto timestamp_sec = chassis.header().timestamp_sec();
  Pose pose;
  pose.mutable_linear_velocity()->set_x(chassis.speed_mps());
  if (!chassis_speed_.Push(timestamp_sec, pose)) {
    AWARN << std::setprecision(15)
          << "Failed push speed_mps to list, with timestamp[" << timestamp_sec
          << "]";
    return false;
  }

  return true;
}

bool PredictorFilteredImu::Updateable() const {
  const auto& imu = dep_predicteds_.find(PredictorImuName())->second;
  if (predicted_.empty()) {
    return !imu.empty();
  } else {
    return !predicted_.Newer(imu.Latest()->first - kSamplingInterval) &&
           !predicted_.Newer(chassis_speed_.Latest()->first -
                             kSamplingInterval);
  }
}

Status PredictorFilteredImu::Update() {
  double timestamp_sec;
  Pose pose;

  const auto& imu = dep_predicteds_.find(PredictorImuName())->second;
  auto latest_it = predicted_.Latest();
  if (latest_it == predicted_.end()) {
    timestamp_sec =
        std::min(chassis_speed_.Latest()->first, imu.Latest()->first);
    imu.FindNearestPose(timestamp_sec, &pose);
    AccelKalmanFilterInit(pose.linear_acceleration().y());
  } else {
    timestamp_sec = latest_it->first + kSamplingInterval;
    imu.FindNearestPose(timestamp_sec, &pose);

    auto a_control = pose.linear_acceleration().y();
    Pose chassis_pose0, chassis_pose1;
    chassis_speed_.FindNearestPose(timestamp_sec - kSamplingInterval,
                                   &chassis_pose0);
    chassis_speed_.FindNearestPose(timestamp_sec, &chassis_pose1);
    auto a_observation = (chassis_pose1.linear_velocity().x() -
                          chassis_pose0.linear_velocity().x()) /
                         kSamplingInterval;

    auto a_estimate = AccelKalmanFilterProcess(a_control, a_observation);
    pose.mutable_linear_acceleration()->set_y(a_estimate);
  }

  predicted_.Push(timestamp_sec, pose);
  return Status::OK();
}

void PredictorFilteredImu::AccelKalmanFilterInit(double a_estimate) {
  Eigen::Matrix<double, 1, 1> x;
  x(0, 0) = a_estimate;
  Eigen::Matrix<double, 1, 1> P;
  constexpr double kP = 0.25;
  P(0, 0) = kP;
  Eigen::Matrix<double, 1, 1> F;
  F(0, 0) = 0.0;
  Eigen::Matrix<double, 1, 1> Q;
  constexpr double kQ = 0.25;
  Q(0, 0) = kQ;
  Eigen::Matrix<double, 1, 1> H;
  H(0, 0) = 1.0;
  Eigen::Matrix<double, 1, 1> R;
  constexpr double kR = 0.01;
  R(0, 0) = kR;
  Eigen::Matrix<double, 1, 1> B;
  B(0, 0) = 1.0;

  accel_kf_.SetStateEstimate(x, P);
  accel_kf_.SetTransitionMatrix(F);
  accel_kf_.SetTransitionNoise(Q);
  accel_kf_.SetObservationMatrix(H);
  accel_kf_.SetObservationNoise(R);
  accel_kf_.SetControlMatrix(B);
}

double PredictorFilteredImu::AccelKalmanFilterProcess(double a_control,
                                                      double a_observation) {
  Eigen::Matrix<double, 1, 1> u;
  u(0, 0) = a_control;
  accel_kf_.Predict(u);
  Eigen::Matrix<double, 1, 1> z;
  z(0, 0) = a_observation;
  accel_kf_.Correct(z);
  auto state = accel_kf_.GetStateEstimate();
  return state(0, 0);
}

}  // namespace localization
}  // namespace apollo
