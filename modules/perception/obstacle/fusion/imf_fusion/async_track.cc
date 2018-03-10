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

#include "modules/perception/obstacle/fusion/imf_fusion/async_track.h"
#include "modules/common/log.h"
#include "modules/common/macro.h"
#include "modules/perception/obstacle/base/types.h"
#include "modules/perception/obstacle/common/geometry_util.h"
#include "modules/perception/obstacle/fusion/imf_fusion/imf_fusion.h"
#include "modules/perception/obstacle/fusion/probabilistic_fusion/pbf_base_track_object_matcher.h"
#include "modules/perception/obstacle/fusion/probabilistic_fusion/pbf_kalman_motion_fusion.h"
#include "modules/perception/obstacle/fusion/probabilistic_fusion/pbf_sensor_manager.h"

namespace apollo {
namespace perception {

/*class AsyncTrack*/
int AsyncTrack::s_track_idx_ = 0;

AsyncTrack::AsyncTrack(PbfSensorObjectPtr obj)
    : s_motion_fusion_method("PbfKalmanMotionFusion") {
  idx_ = GetNextTrackId();
  // SensorType sensor_type = obj->sensor_type;
  std::string sensor_id = obj->sensor_id;
  motion_fusion_.reset(new PbfKalmanMotionFusion());
  fused_timestamp_ = obj->timestamp;
  fused_object_.reset(new PbfSensorObject());
  fused_object_->clone(*obj);
  age_ = 0;
  invisible_period_ = 0;
  tracking_period_ = 0.0;
  is_dead_ = false;
}

void AsyncTrack::SetMotionFusionMethod(const std::string motion_fusion_method) {
  s_motion_fusion_method = motion_fusion_method;
  if (motion_fusion_method == "PbfKalmanMotionFusion") {
    motion_fusion_.reset(new PbfKalmanMotionFusion());
  } else if (motion_fusion_method == "IMFMotionFusion") {
    motion_fusion_.reset(new PbfIMFFusion());
  }
}

AsyncTrack::~AsyncTrack() {}

void AsyncTrack::UpdateWithSensorObject(PbfSensorObjectPtr obj,
                                        double match_dist) {
  // const SensorType &sensor_type = obj->sensor_type;
  const std::string sensor_id = obj->sensor_id;
  PerformMotionFusion(obj);
  // double timestamp = obj->timestamp;
  invisible_period_ = 0;
  tracking_period_ += obj->timestamp - fused_timestamp_;
  fused_timestamp_ = obj->timestamp;
  fused_object_->timestamp = obj->timestamp;
}

void AsyncTrack::UpdateWithoutSensorObject(const SensorType &sensor_type,
                                           const std::string &sensor_id,
                                           double min_match_dist,
                                           double timestamp) {
  double time_diff = timestamp - fused_timestamp_;
  motion_fusion_->UpdateWithoutObject(time_diff);
}

int AsyncTrack::GetTrackId() const {
  return idx_;
}

PbfSensorObjectPtr AsyncTrack::GetFusedObject() {
  return fused_object_;
}

double AsyncTrack::GetFusedTimestamp() const {
  return fused_timestamp_;
}

void AsyncTrack::PerformMotionFusion(PbfSensorObjectPtr obj) {
  if (motion_fusion_ == nullptr) {
    AERROR << "Skip motion fusion becuase motion_fusion_ is nullptr.";
    return;
  }

  const SensorType &sensor_type = obj->sensor_type;
  double time_diff = obj->timestamp - fused_object_->timestamp;

  if (is_camera(sensor_type) || is_radar(sensor_type)) {
    if (motion_fusion_->Initialized()) {
      motion_fusion_->UpdateWithObject(obj, time_diff);
      Eigen::Vector3d anchor_point;
      Eigen::Vector3d velocity;
      Eigen::Vector3d pre_anchor_point;
      Eigen::Vector3d pre_velocity;
      motion_fusion_->GetState(&pre_anchor_point, &pre_velocity);
      motion_fusion_->UpdateWithObject(obj, time_diff);
      motion_fusion_->GetState(&anchor_point, &velocity);
      fused_object_->object->velocity = velocity;
      Eigen::Vector3d translation = anchor_point - pre_anchor_point;
      fused_object_->object->anchor_point = anchor_point;
      fused_object_->object->center += translation;
      fused_object_->object->velocity = velocity;
    } else {
      motion_fusion_->Initialize(obj);
    }
  }
}

int AsyncTrack::GetNextTrackId() {
  int ret_track_id = s_track_idx_;
  if (s_track_idx_ == INT_MAX) {
    s_track_idx_ = 0;
  } else {
    s_track_idx_++;
  }
  return ret_track_id;
}

AsyncTrackManager *AsyncTrackManager::instance() {
  static AsyncTrackManager track_manager;
  return &track_manager;
}

AsyncTrackManager::AsyncTrackManager() {}

AsyncTrackManager::~AsyncTrackManager() {}

int AsyncTrackManager::RemoveLostTracks() {
  size_t track_count = 0;
  for (size_t i = 0; i < tracks_.size(); i++) {
    if (!tracks_[i]->IsDead()) {
      if (i != track_count) {
        tracks_[track_count] = tracks_[i];
      }
      track_count++;
    }
  }
  AINFO << "Remove " << tracks_.size() - track_count << " tracks";
  tracks_.resize(track_count);
  return track_count;
}

}  // namespace perception
}  // namespace apollo
