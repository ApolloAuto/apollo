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

#include "modules/localization/lmd/predictor/perception/predictor_perception.h"

#include <algorithm>
#include <string>

#include "modules/common/log.h"
#include "modules/common/math/quaternion.h"
#include "modules/localization/common/localization_gflags.h"

namespace apollo {
namespace localization {

using apollo::common::ErrorCode;
using apollo::common::PointENU;
using apollo::common::Status;
using apollo::common::math::HeadingToQuaternion;
using apollo::perception::LaneMarkers;

namespace {
constexpr double kPCMapSearchRadius = 10.0;

const std::string& PredictorOutputName() {
  static const std::string name(kPredictorOutputName);
  return name;
}
}  // namespace

PredictorPerception::PredictorPerception(double memory_cycle_sec)
    : Predictor(memory_cycle_sec),
      pc_map_(FLAGS_enable_lmd_premapping ? &lm_provider_ : nullptr),
      pc_registrator_(&pc_map_),
      lane_markers_samples_(memory_cycle_sec) {
  name_ = kPredictorPerceptionName;
  dep_predicteds_.emplace(PredictorOutputName(), PoseList(memory_cycle_sec));
}

PredictorPerception::~PredictorPerception() {}

bool PredictorPerception::UpdateLaneMarkers(double timestamp_sec,
                                            const LaneMarkers& lane_markers) {
  // sampling lane markers
  auto sample = lm_sampler_.Sampling(lane_markers);
  if (!lane_markers_samples_.Push(timestamp_sec, sample)) {
    AWARN << "Failed push lane_markers_samples to list, with timestamp["
          << timestamp_sec << "]";
    return false;
  }

  return true;
}

bool PredictorPerception::Updateable() const {
  const auto& output = dep_predicteds_.find(PredictorOutputName())->second;
  return lane_markers_samples_.Older(output) &&
         (predicted_.Older(lane_markers_samples_) || predicted_.empty()) &&
         DepsTimestamp() > deps_timestamp_sec_;
}

Status PredictorPerception::Update() {
  deps_timestamp_sec_ = DepsTimestamp();

  auto latest_sample_it = lane_markers_samples_.Latest();
  auto timestamp_sec = latest_sample_it->first;
  const auto& latest_sample = latest_sample_it->second;

  // get estimated pose for timestamp_sec
  const auto& output = dep_predicteds_[PredictorOutputName()];
  Pose pose_estimated;
  output.FindNearestPose(timestamp_sec, &pose_estimated);
  if (!pose_estimated.has_position() || !pose_estimated.position().has_x() ||
      !pose_estimated.position().has_y() ||
      !pose_estimated.position().has_z() || !pose_estimated.has_heading()) {
    AERROR << "Pose has not some field";
    return Status(ErrorCode::LOCALIZATION_ERROR, "Pose has not some field");
  }
  const auto& position_estimated = pose_estimated.position();
  auto heading_estimated = pose_estimated.heading();

  // update pc_map
  auto ret = pc_map_.UpdateRange(position_estimated, kPCMapSearchRadius);
  if (ret != Status::OK()) {
    AERROR << "Update pc_map failed";
    return ret;
  }

  // position and heading registration
  PointENU position;
  double heading;
  pc_registrator_.Register(latest_sample, position_estimated, heading_estimated,
                           &position, &heading);

  // fill pose
  Pose pose;
  pose.CopyFrom(pose_estimated);
  pose.mutable_position()->CopyFrom(position);
  pose.set_heading(heading);
  // TODO(all): orientation Need to be more precise
  auto orientation = HeadingToQuaternion(heading);
  pose.mutable_orientation()->set_qx(orientation.x());
  pose.mutable_orientation()->set_qy(orientation.y());
  pose.mutable_orientation()->set_qz(orientation.z());
  pose.mutable_orientation()->set_qw(orientation.w());

  // Add pose to predicted list
  predicted_.Push(timestamp_sec, pose);

  return Status::OK();
}

double PredictorPerception::DepsTimestamp() const {
  double timestamp_sec = std::numeric_limits<double>::min();
  auto sample_it = lane_markers_samples_.Latest();
  if (sample_it != lane_markers_samples_.end()) {
    timestamp_sec = std::max(sample_it->first, timestamp_sec);
  }
  const auto& output = dep_predicteds_.find(PredictorOutputName())->second;
  auto o_it = output.Latest();
  if (o_it != output.end()) {
    timestamp_sec = std::max(o_it->first, timestamp_sec);
  }
  return timestamp_sec;
}

}  // namespace localization
}  // namespace apollo
