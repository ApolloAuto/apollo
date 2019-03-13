/******************************************************************************
 * Copyright 2018 The Apollo Authors. All Rights Reserved.
 *
 * Licensed under the Apache License, Version 2.0 (the License);
 * you may not use this file except in compliance with the License.
 * You may obtain a copy of the License at
 *
 * http://www.apache.org/licenses/LICENSE-2.0
 *
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an AS IS BASIS,
 * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 * See the License for the specific language governing permissions and
 * limitations under the License.
 *****************************************************************************/
#include "modules/perception/radar/lib/tracker/common/radar_track.h"
#include "modules/perception/radar/lib/tracker/filter/adaptive_kalman_filter.h"

namespace apollo {
namespace perception {
namespace radar {

// Static member variable
int RadarTrack::s_current_idx_ = 0;
int RadarTrack::s_tracked_times_threshold_ = 3;
bool RadarTrack::s_use_filter_ = false;
std::string RadarTrack::s_chosen_filter_ =  // NOLINT
    "AdaptiveKalmanFilter";

RadarTrack::RadarTrack(const base::ObjectPtr& obs, const double timestamp) {
  s_current_idx_ %= MAX_RADAR_IDX;
  obs_id_ = s_current_idx_++;
  obs_radar_ = base::ObjectPool::Instance().Get();
  *obs_radar_ = *obs;
  obs_ = base::ObjectPool::Instance().Get();
  *obs_ = *obs;
  timestamp_ = timestamp;
  tracked_times_ = 1;
  tracking_time_ = 0.0;
  is_dead_ = false;

  // Or use register class instead.
  if (s_chosen_filter_ == "AdaptiveKalmanFilter") {
    filter_ = std::shared_ptr<BaseFilter>(new AdaptiveKalmanFilter());
  } else {
    // default
    filter_ = std::shared_ptr<BaseFilter>(new AdaptiveKalmanFilter());
  }

  filter_->Init(*obs);
}

void RadarTrack::UpdataObsRadar(const base::ObjectPtr& obs_radar,
                                const double timestamp) {
  *obs_radar_ = *obs_radar;
  *obs_ = *obs_radar;
  double time_diff = timestamp - timestamp_;
  if (s_use_filter_) {
    Eigen::VectorXd state;
    state = filter_->UpdateWithObject(*obs_radar_, time_diff);
    obs_->center(0) = static_cast<float>(state(0));
    obs_->center(1) = static_cast<float>(state(1));
    obs_->velocity(0) = static_cast<float>(state(2));
    obs_->velocity(1) = static_cast<float>(state(3));
    Eigen::Matrix4d covariance_matrix = filter_->GetCovarianceMatrix();
    obs_->center_uncertainty(0) = static_cast<float>(covariance_matrix(0, 0));
    obs_->center_uncertainty(1) = static_cast<float>(covariance_matrix(1, 1));
    obs_->velocity_uncertainty(1) = static_cast<float>(covariance_matrix(2, 2));
    obs_->velocity_uncertainty(1) = static_cast<float>(covariance_matrix(3, 3));
  }
  tracking_time_ += time_diff;
  timestamp_ = timestamp;
  ++tracked_times_;
}

void RadarTrack::SetObsRadarNullptr() {
  obs_radar_ = nullptr;
  obs_ = nullptr;
}

base::ObjectPtr RadarTrack::GetObsRadar() { return obs_radar_; }

base::ObjectPtr RadarTrack::GetObs() { return obs_; }

int RadarTrack::GetObsId() const { return obs_id_; }

double RadarTrack::GetTimestamp() { return timestamp_; }

double RadarTrack::GetTrackingTime() { return tracking_time_; }
}  // namespace radar
}  // namespace perception
}  // namespace apollo
