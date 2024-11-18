/******************************************************************************
 * Copyright 2024 The Apollo Authors. All Rights Reserved.
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
#include "modules/perception/camera_detection_occupancy/tracker/common/camera_track.h"

#include "modules/perception/camera_detection_occupancy/tracker/filter/adaptive_kalman_filter.h"

namespace apollo {
namespace perception {
namespace camera {

// Static member variable
int CameraTrack::s_current_idx_ = 0;
int CameraTrack::s_tracked_times_threshold_ = 3;
const int MAX_CAMERA_IDX = 2147483647;
bool CameraTrack::s_use_filter_ = false;
std::string CameraTrack::s_chosen_filter_ =  // NOLINT
    "AdaptiveKalmanFilter";

CameraTrack::CameraTrack(const base::ObjectPtr& obs, const double timestamp) {
  s_current_idx_ %= MAX_CAMERA_IDX;
  obs_id_ = s_current_idx_++;
  obs_camera_ = base::ObjectPool::Instance().Get();
  *obs_camera_ = *obs;
  obs_ = base::ObjectPool::Instance().Get();
  *obs_ = *obs;
  timestamp_ = timestamp;
  tracked_times_ = 1;
  tracking_time_ = 0.0;
  is_dead_ = false;
  is_assigned_ = false;

  // Or use register class instead.
  filter_.reset(new AdaptiveKalmanFilter);
  filter_->Init(*obs);
}

void CameraTrack::UpdataObsCamera(const base::ObjectPtr& obs_camera,
                                  const double timestamp) {
  *obs_camera_ = *obs_camera;
  *obs_ = *obs_camera;
  double time_diff = timestamp - timestamp_;
  if (s_use_filter_) {
    Eigen::VectorXd state;
    state = filter_->UpdateWithObject(*obs_camera_, time_diff);
    obs_->center(0) = static_cast<float>(state(0));
    obs_->center(1) = static_cast<float>(state(1));
    obs_->velocity(0) = static_cast<float>(state(2));
    obs_->velocity(1) = static_cast<float>(state(3));
    Eigen::Matrix4d covariance_matrix = filter_->GetCovarianceMatrix();
    obs_->center_uncertainty(0) = static_cast<float>(covariance_matrix(0, 0));
    obs_->center_uncertainty(1) = static_cast<float>(covariance_matrix(1, 1));
    obs_->velocity_uncertainty(0) = static_cast<float>(covariance_matrix(2, 2));
    obs_->velocity_uncertainty(1) = static_cast<float>(covariance_matrix(3, 3));
  }
  tracking_time_ += time_diff;
  timestamp_ = timestamp;
  ++tracked_times_;
  is_assigned_ = true;
}

void CameraTrack::SetObsCameraNullptr() {
  obs_camera_ = nullptr;
  obs_ = nullptr;
}

base::ObjectPtr CameraTrack::GetObsCamera() { return obs_camera_; }

base::ObjectPtr CameraTrack::GetObs() { return obs_; }

int CameraTrack::GetObsId() const { return obs_id_; }

double CameraTrack::GetTimestamp() { return timestamp_; }

double CameraTrack::GetTrackingTime() { return tracking_time_; }
}  // namespace camera
}  // namespace perception
}  // namespace apollo
