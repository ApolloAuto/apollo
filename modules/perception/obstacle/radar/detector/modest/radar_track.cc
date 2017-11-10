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

#include "modules/perception/obstacle/radar/detector/modest/radar_track.h"

namespace apollo {
namespace perception {

// Static member variable
int RadarTrack::s_current_idx_ = 0;
std::string RadarTrack::s_chosen_filter_ = "";
int RadarTrack::s_tracked_times_threshold_ = 4;

RadarTrack::RadarTrack() {
  s_current_idx_ %= MAX_RADAR_IDX;
  obs_id_ = s_current_idx_++;

  obs_radar_ = nullptr;
  obs_ = nullptr;
  tracked_times_ = 1;
  tracking_time_ = 0.0;
}

RadarTrack::RadarTrack(Object &obs, const double &timestamp) {
  s_current_idx_ %= MAX_RADAR_IDX;
  obs_id_ = s_current_idx_++;

  obs_radar_ = ObjectPtr(new Object);
  obs_ = ObjectPtr(new Object);

  *obs_radar_ = obs;
  *obs_ = obs;
  if (s_chosen_filter_ == "Adaptive Kalman Filter") {
    tracker_ = boost::shared_ptr<BaseFilter>(new AdaptiveKalmanFilter());
  } else {
    AERROR << "Failed to initialize RadarTrack.";
    return;
  }
  tracker_->Initialize(obs);

  timestamp_ = timestamp;
  tracked_times_ = 1;
  tracking_time_ = 0.0;
  id_tracked_ = false;
}

RadarTrack::RadarTrack(const RadarTrack &track) {
  obs_id_ = track.obs_id_;
  obs_radar_ = track.obs_radar_;
  obs_ = track.obs_;
  tracked_times_ = track.tracked_times_;
  tracking_time_ = track.tracking_time_;
  tracker_ = track.tracker_;
  timestamp_ = track.timestamp_;
  id_tracked_ = track.id_tracked_;
}

RadarTrack &RadarTrack::operator=(const RadarTrack &track) {
  obs_id_ = track.obs_id_;
  obs_radar_ = track.obs_radar_;
  obs_ = track.obs_;
  tracker_ = track.tracker_;
  tracked_times_ = track.tracked_times_;
  tracking_time_ = track.tracking_time_;
  timestamp_ = track.timestamp_;
  id_tracked_ = track.id_tracked_;
  return *this;
}

void RadarTrack::UpdataObsRadar(ObjectPtr obs_radar, const double timestamp) {
  obs_radar_ = obs_radar;
  Eigen::Vector4d result;
  double time_diff = timestamp - timestamp_;
  if (time_diff < 0) {
      AWARN << "New objects should younger than the track";
      return;
  }
  result = tracker_->UpdateWithObject(*obs_radar_, time_diff);
  (*obs_).clone(*obs_radar_);
  (*obs_).center[0] = result[0];
  (*obs_).center[1] = result[1];
  (*obs_).velocity[0] = result[2];
  (*obs_).velocity[1] = result[3];

  Eigen::Matrix4d covariance_matrix = tracker_->GetCovarianceMatrix();
  (*obs_).position_uncertainty.topLeftCorner(2, 2) =
      covariance_matrix.topLeftCorner(2, 2);
  (*obs_).velocity_uncertainty.topLeftCorner(2, 2) =
      covariance_matrix.block<2, 2>(2, 2);

  tracking_time_ += timestamp - timestamp_;
  timestamp_ = timestamp;
}

void RadarTrack::SetObsRadar(ObjectPtr obs_radar) {
  obs_radar_ = obs_radar;
}

void RadarTrack::IncreaseTrackedTimes() {
  tracked_times_++;
}

int RadarTrack::GetObsId() const {
  return obs_id_;
}

ObjectPtr RadarTrack::GetObsRadar() {
  return obs_radar_;
}

ObjectPtr RadarTrack::GetObs() {
  return obs_;
}

double RadarTrack::GetTimestamp() {
  return timestamp_;
}

double RadarTrack::GetTrackingTime() {
  return tracking_time_;
}

} // namespace perception
} // namespace apollo