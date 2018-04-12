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

#include "modules/perception/obstacle/camera/filter/object_camera_filter.h"

namespace apollo {
namespace perception {

bool ObjectCameraFilter::Init() { return true; }

bool ObjectCameraFilter::Filter(
    const double &timestamp,
    std::vector<std::shared_ptr<VisualObject>> *objects) {
  if (!objects) return false;

  // update lost_frame_count
  for (auto p : tracked_filters_) {
    p.second.lost_frame_cnt_ += 1;
  }

  for (auto obj_ptr : *objects) {
    int track_id = obj_ptr->track_id;

    if (track_id != -1 && tracked_filters_.count(track_id)) {
      Predict(track_id, timestamp);
      Update(track_id, obj_ptr);
      GetState(track_id, obj_ptr);
    } else {
      Create(track_id, timestamp, obj_ptr);
    }
  }

  Destroy();
  return true;
}

std::string ObjectCameraFilter::Name() const { return "ObjectCameraFilter"; }

void ObjectCameraFilter::Create(const int &track_id, const double &timestamp,
                                const std::shared_ptr<VisualObject> &obj_ptr) {
  tracked_filters_[track_id] = ObjectFilter();
  tracked_filters_[track_id].track_id_ = track_id;
  tracked_filters_[track_id].last_timestamp_ = timestamp;
  tracked_filters_[track_id].x_.Init(obj_ptr->center.x());
  tracked_filters_[track_id].y_.Init(obj_ptr->center.y());
  tracked_filters_[track_id].theta_.Init(obj_ptr->theta);
}

void ObjectCameraFilter::Predict(const int &track_id, const double &timestamp) {
  double time_diff = timestamp - tracked_filters_[track_id].last_timestamp_;
  float diff = static_cast<float>(time_diff);

  tracked_filters_[track_id].x_.Predict(diff);
  tracked_filters_[track_id].y_.Predict(diff);
  tracked_filters_[track_id].theta_.Predict(diff);

  tracked_filters_[track_id].lost_frame_cnt_ = 0;
  tracked_filters_[track_id].last_timestamp_ = timestamp;
}

void ObjectCameraFilter::Update(const int &track_id,
                                const std::shared_ptr<VisualObject> &obj_ptr) {
  tracked_filters_[track_id].x_.Update(obj_ptr->center.x());
  tracked_filters_[track_id].y_.Update(obj_ptr->center.y());
  tracked_filters_[track_id].theta_.Update(obj_ptr->theta);
}

void ObjectCameraFilter::GetState(const int &track_id,
                                  std::shared_ptr<VisualObject> obj_ptr) {
  auto x_state = tracked_filters_[track_id].x_.GetState();
  auto y_state = tracked_filters_[track_id].y_.GetState();
  auto x_state_cov = tracked_filters_[track_id].x_.GetCov();
  auto y_state_cov = tracked_filters_[track_id].y_.GetCov();

  obj_ptr->center.x() = x_state.x();
  obj_ptr->velocity.x() = x_state.y();

  obj_ptr->center.y() = y_state.x();
  obj_ptr->velocity.y() = y_state.y();

  obj_ptr->center.z() = 0.0f;
  obj_ptr->velocity.z() = 0.0f;

  obj_ptr->state_uncertainty.block(0, 0, 2, 2) << x_state_cov(0, 0), 0, 0,
      y_state_cov(0, 0);
  obj_ptr->state_uncertainty.block(2, 2, 2, 2) << x_state_cov(1, 1), 0, 0,
      y_state_cov(1, 1);
  obj_ptr->state_uncertainty.block(0, 2, 2, 2) << x_state_cov(0, 1), 0, 0,
      y_state_cov(0, 1);
  obj_ptr->state_uncertainty.block(2, 0, 2, 2) << x_state_cov(1, 0), 0, 0,
      y_state_cov(1, 0);

  obj_ptr->theta = tracked_filters_[track_id].theta_.GetState().x();
  obj_ptr->direction =
      Eigen::Vector3f(cos(obj_ptr->theta), 0.0f, -sin(obj_ptr->theta));
}

void ObjectCameraFilter::Destroy() {
  std::vector<int> id_to_destroy;
  for (const auto &p : tracked_filters_) {
    if (p.second.lost_frame_cnt_ > kMaxKeptFrameCnt) {
      id_to_destroy.emplace_back(p.first);
    }
  }

  for (const auto &id : id_to_destroy) {
    tracked_filters_.erase(id);
  }
}

}  // namespace perception
}  // namespace apollo
