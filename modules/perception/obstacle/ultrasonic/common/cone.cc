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

#include "modules/perception/obstacle/ultrasonic/common/cone.h"
#include "modules/common/log.h"

namespace apollo {
namespace perception {

RawCone::RawCone(const int id,
    const std::vector<ExtrinIdent> &systmextrinsics_) {
  id_ = id;
  rho_ = 0;
  meas_flag_ = false;
  extrinsics_ = systmextrinsics_[id_];
  for (size_t i = 0; i < extrinsics_.upoint_arc().size(); ++i) {
    Eigen::Vector3d localpoint = rho_ * extrinsics_.upoint_arc().at(i);
    localpoint = extrinsics_.transform() * localpoint;
    meas_arc_.push_back(localpoint);
  }
}

RawCone::RawCone(const int id, const ExtrinIdent &extrinsics) {
  id_ = id;
  rho_ = 0;
  meas_flag_ = false;
  extrinsics_ = extrinsics;
  for (size_t i = 0; i < extrinsics_.upoint_arc().size(); ++i) {
    Eigen::Vector3d localpoint = rho_ * extrinsics_.upoint_arc().at(i);
    localpoint = extrinsics_.transform() * localpoint;
    meas_arc_.push_back(localpoint);
  }
}

void RawCone::cone_update(const float rho, const float time_diff) {
  rho_ = rho;
  meas_flag_ = (rho_ > 0)? 1 : 0;
  for (size_t i = 0; i < meas_arc_.size(); ++i) {
    Eigen::Vector3d localpoint = rho_ * extrinsics_.upoint_arc().at(i);
    meas_arc_[i] = extrinsics_.transform() * localpoint;
  }
}

FilteredCone::FilteredCone(const int id, const ExtrinIdent& systm_extrinsic) {
  id_ = id;
  rho_ = 0;
  meas_flag_ = false;
  extrinsics_ = systm_extrinsic;
  timestamp_ = 0.0;
  tracking_time_ = 0.0;
  for (size_t i = 0; i < extrinsics_.upoint_arc().size(); ++i) {
    Eigen::Vector3d localpoint = rho_ * extrinsics_.upoint_arc().at(i);
    localpoint = extrinsics_.transform() * localpoint;
    meas_arc_.push_back(localpoint);
  }
}

void FilteredCone::cone_update(const float rho, const double timestamp) {
  meas_flag_ = (rho > 0)? true:false;
  if (meas_flag_) {
    rho_ = rho;
    if (kf_.GetTrackCount() > 0) {
      double time_diff = timestamp - timestamp_;
      if (time_diff < 0) {
        AERROR << "error, new objects younger than track age";
      } else {
        kf_.UpdateWithObject(rho_, time_diff);
        rho_ = kf_.GetState();
        tracking_time_ += time_diff;
      }
    } else {
      kf_.Initialize(rho_);
    }
  } else {
    if (kf_.GetLostCount() < kUltrasonicMaxLostCount &&
        kf_.GetTrackCount() > 0) {
      double time_diff = timestamp - timestamp_;
      if (time_diff < 0) {
        AERROR << "error, new objects younger than track age";
      } else {
        kf_.UpdateWithoutObject(time_diff);
        meas_flag_ = true;
        tracking_time_ += time_diff;
      }
    } else {
      kf_.Reset();
    }
    rho_ = kf_.GetState();
  }

  timestamp_ = timestamp;
  for (size_t i = 0; i < meas_arc_.size(); ++i) {
    Eigen::Vector3d localpoint = rho_ * extrinsics_.upoint_arc().at(i);
    meas_arc_[i] = extrinsics_.transform() * localpoint;
  }
}

}  // namespace perception
}  // namespace apollo
