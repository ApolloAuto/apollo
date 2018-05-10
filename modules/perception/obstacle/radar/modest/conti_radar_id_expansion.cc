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
#include "modules/perception/obstacle/radar/modest/conti_radar_id_expansion.h"

namespace apollo {
namespace perception {
ContiRadarIDExpansion::ContiRadarIDExpansion() {
  current_idx_ = 0;
  need_restart_ = true;
  need_inner_restart_ = false;
  timestamp_ = 0.0;
}

ContiRadarIDExpansion::~ContiRadarIDExpansion() {}

void ContiRadarIDExpansion::ExpandIds(ContiRadar *radar_obs) {
  if (radar_obs == nullptr) {
    return;
  }
  // SkipOutdatedObjects(radar_obs);
  for (int i = 0; i < radar_obs->contiobs_size(); ++i) {
    ContiRadarObs &contiobs = *(radar_obs->mutable_contiobs(i));
    int id = contiobs.obstacle_id();
    int meas_state = contiobs.meas_state();
    if (/*need_restart_ || */ need_inner_restart_ ||
        meas_state == static_cast<int>(ContiMeasState::CONTI_NEW)) {
      int next_id = GetNextId();
      local2global_[id] = next_id;
    } else {
      auto it = local2global_.find(id);
      if (it == local2global_.end()) {
        int next_id = GetNextId();
        local2global_[id] = next_id;
      }
    }
    contiobs.set_obstacle_id(local2global_[id]);
  }
}

void ContiRadarIDExpansion::SkipOutdatedObjects(ContiRadar *radar_obs) {
  ContiRadar out_obs;
  double timestamp = radar_obs->header().timestamp_sec() - 1e-6;
  need_inner_restart_ = false;
  for (int i = 0; i < radar_obs->contiobs_size(); ++i) {
    ContiRadarObs &contiobs = *(radar_obs->mutable_contiobs(i));
    double object_timestamp =
        static_cast<double>(contiobs.header().timestamp_sec());
    if (object_timestamp > timestamp) {
      ContiRadarObs *obs = out_obs.add_contiobs();
      *obs = contiobs;
    } else {
      need_inner_restart_ = true;
    }
  }
  if (need_inner_restart_) {
    AINFO << "skip outdated objects: " << radar_obs->contiobs_size() << " -> "
          << out_obs.contiobs_size();
    *radar_obs = out_obs;
  }
}

void ContiRadarIDExpansion::SetNeedRestart(const bool need_restart) {
  need_restart_ = need_restart;
}

void ContiRadarIDExpansion::UpdateTimestamp(const double &timestamp) {
  need_restart_ = false;
  if (timestamp - timestamp_ > 0.1) {
    need_restart_ = true;
  }
  timestamp_ = timestamp;
}

int ContiRadarIDExpansion::GetNextId() {
  ++current_idx_;
  current_idx_ %= MAX_RADAR_IDX;
  return current_idx_;
}

}  // namespace perception
}  // namespace apollo
