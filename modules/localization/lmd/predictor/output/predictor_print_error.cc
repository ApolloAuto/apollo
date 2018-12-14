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

#include "modules/localization/lmd/predictor/output/predictor_print_error.h"

#include <algorithm>
#include <iomanip>

#include "modules/common/math/math_utils.h"
#include "modules/common/math/quaternion.h"

namespace apollo {
namespace localization {

using apollo::common::Status;
using apollo::common::math::QuaternionToHeading;
using apollo::common::math::RotateAxis;

namespace {
constexpr double kPrintErrorInterval = 0.01;
}  // namespace

PredictorPrintError::PredictorPrintError(double memory_cycle_sec)
    : Predictor(memory_cycle_sec) {
  name_ = kPredictorPrintErrorName;
  dep_predicteds_.emplace(kPredictorGpsName, PoseList(memory_cycle_sec));
  dep_predicteds_.emplace(kPredictorOutputName, PoseList(memory_cycle_sec));
  on_adapter_thread_ = true;
}

PredictorPrintError::~PredictorPrintError() {}

bool PredictorPrintError::Updateable() const {
  const auto& gps = dep_predicteds_.find(kPredictorGpsName)->second;
  const auto& output = dep_predicteds_.find(kPredictorOutputName)->second;
  return !gps.empty() && !output.empty() &&
         (!has_latest_timestamp_sec_ ||
          latest_timestamp_sec_ + kPrintErrorInterval < DepsTimestamp());
}

Status PredictorPrintError::Update() {
  auto deps_timestamp_sec = DepsTimestamp();
  if (!has_latest_timestamp_sec_) {
    has_latest_timestamp_sec_ = true;
    latest_timestamp_sec_ = deps_timestamp_sec;
    return Status::OK();
  }

  for (auto timestamp_sec = latest_timestamp_sec_;
       timestamp_sec + kPrintErrorInterval < deps_timestamp_sec;
       timestamp_sec += kPrintErrorInterval) {
    const auto& output = dep_predicteds_[kPredictorOutputName];
    Pose pose;
    output.FindMatchingPose(timestamp_sec, &pose);
    PrintPoseError(timestamp_sec, pose);
  }

  latest_timestamp_sec_ = deps_timestamp_sec;
  return Status::OK();
}

double PredictorPrintError::DepsTimestamp() const {
  auto g_it = dep_predicteds_.find(kPredictorGpsName)->second.Latest();
  auto o_it = dep_predicteds_.find(kPredictorOutputName)->second.Latest();
  return std::min(o_it->first, g_it->first);
}

void PredictorPrintError::PrintPoseError(double timestamp_sec,
                                         const Pose& pose) {
  const auto& gps = dep_predicteds_[kPredictorGpsName];
  Pose gps_pose;
  if (!gps.FindMatchingPose(timestamp_sec, &gps_pose)) {
    return;
  }

  if (!pose.has_position() || !gps_pose.has_position() ||
      !pose.has_orientation() || !gps_pose.has_orientation() ||
      !pose.has_linear_velocity() || !gps_pose.has_linear_velocity()) {
    AERROR << "Poses has not some feilds";
    return;
  }

  auto heading =
      QuaternionToHeading(pose.orientation().qw(), pose.orientation().qx(),
                          pose.orientation().qy(), pose.orientation().qz());
  auto gps_heading = QuaternionToHeading(
      gps_pose.orientation().qw(), gps_pose.orientation().qx(),
      gps_pose.orientation().qy(), gps_pose.orientation().qz());

  double flu_vx, flu_vy;
  RotateAxis(gps_heading, gps_pose.linear_velocity().x(),
             gps_pose.linear_velocity().y(), &flu_vx, &flu_vy);

  // TODO(all): estimate acceleration
  double flu_ax = 0.0, flu_ay = 0.0;
  Pose gps_pose0;
  constexpr double kTimeDiff = 0.01;
  if (gps.FindMatchingPose(timestamp_sec - kTimeDiff, &gps_pose0) &&
      gps_pose0.has_orientation() && gps_pose0.has_linear_velocity()) {
    auto gps_heading0 = QuaternionToHeading(
        gps_pose0.orientation().qw(), gps_pose0.orientation().qx(),
        gps_pose0.orientation().qy(), gps_pose0.orientation().qz());
    double flu_vx0, flu_vy0;
    RotateAxis(gps_heading0, gps_pose0.linear_velocity().x(),
               gps_pose0.linear_velocity().y(), &flu_vx0, &flu_vy0);
    flu_ax = (flu_vx - flu_vx0) / kTimeDiff;
    flu_ay = (flu_vy - flu_vy0) / kTimeDiff;
  }

  double flu_dx, flu_dy;
  RotateAxis(gps_heading, pose.position().x() - gps_pose.position().x(),
             pose.position().y() - gps_pose.position().y(), &flu_dx, &flu_dy);

  double flu_dvx, flu_dvy;
  RotateAxis(gps_heading, pose.linear_velocity().x(),
             pose.linear_velocity().y(), &flu_dvx, &flu_dvy);
  flu_dvx -= flu_vx;
  flu_dvy -= flu_vy;

  double flu_dax, flu_day;
  RotateAxis(gps_heading, pose.linear_acceleration().x(),
             pose.linear_acceleration().y(), &flu_dax, &flu_day);
  flu_dax -= flu_ax;
  flu_day -= flu_ay;

  ADEBUG << std::setprecision(15) << "Timestamp[" << timestamp_sec << "]";
  ADEBUG << std::setprecision(15) << "True heading[" << gps_heading
         << "], heading error[" << heading - gps_heading << "]";
  ADEBUG << std::setprecision(15) << "True position, x["
         << gps_pose.position().x() << "], y[" << gps_pose.position().y()
         << "], z[" << gps_pose.position().z() << "], position error, station["
         << flu_dx << "], lateral[" << flu_dy << "]";
  ADEBUG << std::setprecision(15) << "True velocity, station[" << flu_vx
         << "], lateral[" << flu_vy << "], velocity error, station[" << flu_dvx
         << "], lateral[" << flu_dvy << "]";
  ADEBUG << std::setprecision(15) << "True acceleration, station[" << flu_ax
         << "], lateral[" << flu_ay << "], acceleration error, station["
         << flu_dax << "], lateral[" << flu_day << "]";
}

}  // namespace localization
}  // namespace apollo
