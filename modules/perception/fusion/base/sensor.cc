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
#include "modules/perception/fusion/base/sensor.h"

#include <assert.h>

#include "modules/perception/base/log.h"

namespace apollo {
namespace perception {
namespace fusion {

int Sensor::kMaxCachedFrameNum = 10;

void Sensor::QueryLatestFrames(double timestamp,
                               std::vector<SensorFramePtr>* frames) {
  assert(frames != nullptr);

  frames->clear();
  for (size_t i = 0; i < frames_.size(); i++) {
    if (frames_[i]->GetTimestamp() > latest_query_timestamp_ &&
        frames_[i]->GetTimestamp() <= timestamp) {
      frames->push_back(frames_[i]);
    }
  }
  latest_query_timestamp_ = timestamp;
}

SensorFramePtr Sensor::QueryLatestFrame(double timestamp) {
  SensorFramePtr latest_frame = nullptr;
  for (size_t i = 0; i < frames_.size(); i++) {
    if (frames_[i]->GetTimestamp() > latest_query_timestamp_ &&
        frames_[i]->GetTimestamp() <= timestamp) {
      latest_frame = frames_[i];
      latest_query_timestamp_ = frames_[i]->GetTimestamp();
    }
  }
  return latest_frame;
}

bool Sensor::GetPose(double timestamp, Eigen::Affine3d* pose) const {
  assert(pose != nullptr);

  for (int i = static_cast<int>(frames_.size()) - 1; i >= 0; --i) {
    double time_diff = timestamp - frames_[i]->GetTimestamp();
    if (fabs(time_diff) < 1.0e-3) {
      return frames_[i]->GetPose(pose);
    }
  }

  LOG_ERROR << "Failed to find pose for timestamp: "
            << GLOG_TIMESTAMP(timestamp);
  return false;
}

void Sensor::AddFrame(const base::FrameConstPtr& frame_ptr) {
  SensorPtr sensor_ptr = this->GetPtr();
  SensorFramePtr frame(new SensorFrame());
  frame->Initialize(frame_ptr, sensor_ptr);
  if (frames_.size() == kMaxCachedFrameNum) {
    frames_.pop_front();
  }
  frames_.emplace_back(frame);
}

}  // namespace fusion
}  // namespace perception
}  // namespace apollo
