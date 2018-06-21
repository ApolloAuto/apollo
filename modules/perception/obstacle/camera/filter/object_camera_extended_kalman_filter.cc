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

#include "modules/perception/obstacle/camera/filter/object_camera_extended_kalman_filter.h"

#include <cmath>
#include <string>
#include <vector>

namespace apollo {
namespace perception {

using common::math::ExtendedKalmanFilter;
using ObjectFilter = ObjectCameraExtendedKalmanFilter::ObjectFilter;

void ObjectCameraExtendedKalmanFilter::GetState(const int track_id,
    const std::shared_ptr<VisualObject>& obj_ptr) {

  const auto& filter = tracked_filters_[track_id].object_config_filter_;
  Eigen::Vector4f mean = filter.GetStateMean();
  Eigen::Matrix4f covariance = filter.GetStateCovariance();

  obj_ptr->center.x() = mean[0];
  obj_ptr->velocity.x() = mean[3] * std::cos(mean[2]);

  obj_ptr->center.y() = mean[1];
  obj_ptr->velocity.y() = mean[3] * std::sin(mean[2]);

  obj_ptr->center.z() = 0.0f;
  obj_ptr->velocity.z() = 0.0f;

  obj_ptr->state_uncertainty.block(0, 0, 2, 2) << covariance(0, 0),
      covariance(0, 1), covariance(1, 0), covariance(1, 1);
  // TODO(zhangyajia): set the covariance for other variables

  obj_ptr->theta = mean[2];
  obj_ptr->direction = Eigen::Vector3f(std::cos(obj_ptr->theta), 0.0f,
      -std::sin(obj_ptr->theta));
}

bool ObjectCameraExtendedKalmanFilter::Filter(const double timestamp,
    std::vector<std::shared_ptr<VisualObject> >* objects) {
  if (!objects) {
    return false;
  }

  // update lost_frame_count
  for (auto& p : tracked_filters_) {
    p.second.lost_frame_cnt_ += 1;
  }

  for (auto& obj_ptr : *objects) {
    int track_id = obj_ptr->track_id;

    if (track_id != -1 && tracked_filters_.count(track_id) > 0) {
      Predict(track_id, timestamp);
      Update(track_id, obj_ptr);
      GetState(track_id, obj_ptr);
    } else {
      tracked_filters_[track_id] = CreateObjectFilter(track_id, timestamp,
          obj_ptr);
    }
  }

  Destroy();
  return true;
}

void ObjectCameraExtendedKalmanFilter::Predict(const int track_id,
    const double timestamp) {
  double time_diff = timestamp - tracked_filters_[track_id].last_timestamp_;

  // update transition model
  auto f = [&time_diff](const Eigen::Vector4f& x,
                        const Eigen::Matrix<float, 1, 1>& u) {
        Eigen::Vector4f x_next;
        x_next[0] = x[0] + x[3] * time_diff * std::cos(x[2]);
        x_next[1] = x[1] + x[3] * time_diff * std::sin(x[2]);
        x_next[2] = x[2];
        x_next[3] = x[3];
        return x_next;
      };

  Eigen::Vector4f state =
      tracked_filters_[track_id].object_config_filter_.GetStateMean();
  Eigen::Matrix4f F = UpdateTransitionMatrix(state[2], state[3], time_diff);
  tracked_filters_[track_id].object_config_filter_.SetTransitionModel(f, F);

  tracked_filters_[track_id].object_config_filter_.Predict();
  tracked_filters_[track_id].lost_frame_cnt_ = 0;
  tracked_filters_[track_id].last_timestamp_ = timestamp;
}

void ObjectCameraExtendedKalmanFilter::Update(const int track_id,
    const std::shared_ptr<VisualObject> &obj_ptr) {
  auto x = obj_ptr->center.x();
  auto y = obj_ptr->center.y();
  auto theta = obj_ptr->theta;
  tracked_filters_[track_id].object_config_filter_.Correct({x, y, theta });
}

std::string ObjectCameraExtendedKalmanFilter::Name() const {
  return "ObjectCameraExtendedKalmanFilter";
}

ObjectFilter ObjectCameraExtendedKalmanFilter::CreateObjectFilter(
    const int track_id, const float timestamp,
    const std::shared_ptr<VisualObject>& ptr_object) const {
  ObjectFilter object_filter(track_id, timestamp);

  auto x = ptr_object->center.x();
  auto y = ptr_object->center.y();
  auto theta = ptr_object->theta;
  auto v = ptr_object->velocity.norm();
  object_filter.object_config_filter_ = InitObjectFilter(x, y, theta, v);

  return object_filter;
}

ExtendedKalmanFilter<float, 4, 3, 1>
ObjectCameraExtendedKalmanFilter::InitObjectFilter(const float x,
    const float y, const float theta, const float v) const {

  ExtendedKalmanFilter<float, 4, 3, 1> filter;
  Eigen::Vector4f state = { x, y, theta, v };

  // initial guess of the believe space
  Eigen::Matrix4f P;
  P.setIdentity();
  P(0, 0) = P(1, 1) = 20.0f;
  P(2, 2) = (M_PI / 3) * (M_PI / 3);
  P(3, 3) = 20.0f;
  filter.SetStateEstimate(state, P);

  // transition model will be set dynamically.

  // observation model is fixed
  // observation function
  auto h = [](const Eigen::Vector4f& x) {
    Eigen::Vector3f o;
    o[0] = x[0];
    o[1] = x[1];
    o[2] = x[2];
    return o;
  };
  // observation matrix
  Eigen::Matrix<float, 3, 4> H;
  H.setIdentity();
  filter.SetObservationModel(h, H);

  // observation noise
  Eigen::Matrix<float, 3, 3> R;
  R.setIdentity();
  R(0, 0) = R(1, 1) = 9.0f;
  R(2, 2) = (M_PI / 6) * (M_PI / 6);
  filter.SetObservationNoise(R);

  return filter;
}

Eigen::Matrix4f ObjectCameraExtendedKalmanFilter::UpdateTransitionMatrix(
    const double theta, const double v, const double dt) const {
  Eigen::Matrix4f F;
  F << 1.0, 0.0, v * dt * (-1.0) * std::sin(theta), dt * std::cos(theta),
      0.0, 1.0, v * dt * std::cos(theta), dt * std::sin(theta),
      0.0, 0.0, 1.0, 0.0,
      0.0, 0.0, 0.0, 1.0;
  return F;
}

void ObjectCameraExtendedKalmanFilter::Destroy() {
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
