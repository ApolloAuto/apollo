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
#include "modules/perception/common/perception_gflags.h"

namespace apollo {
namespace perception {

using KalmanFilter1D = common::math::KalmanFilter<float, 2, 1, 1>;

bool ObjectCameraFilter::Init() { return true; }

bool ObjectCameraFilter::Filter(
    const double timestamp, std::vector<std::shared_ptr<VisualObject>>* objects,
    const FilterOptions& options) {
  if (!objects) return false;

  // update lost_frame_count
  for (auto p : tracked_filters_) {
    p.second.lost_frame_cnt_ += 1;
  }

  for (auto obj_ptr : *objects) {
    int track_id = obj_ptr->track_id;

    if (track_id != -1 && tracked_filters_.count(track_id)) {
      Predict(track_id, timestamp);
      Update(track_id, obj_ptr, options);
      GetState(track_id, obj_ptr);
    } else {
      Create(track_id, timestamp, obj_ptr, options);
    }
  }

  Destroy();
  return true;
}

std::string ObjectCameraFilter::Name() const { return "ObjectCameraFilter"; }

void ObjectCameraFilter::InitFilter(const float x, KalmanFilter1D* filter) {
  CHECK_NOTNULL(filter);

  Eigen::Matrix<float, 2, 1> state_x;
  state_x << x, 0.0f;

  Eigen::Matrix<float, 2, 2> p;
  p.setIdentity();
  p *= 20.0f;

  filter->SetStateEstimate(state_x, p);

  Eigen::Matrix<float, 2, 2> f;
  f << 1.0f, 0.033f, 0.0f, 1.0f;
  filter->SetTransitionMatrix(f);

  Eigen::Matrix<float, 1, 2> h;
  h << 1.0f, 0.0f;
  filter->SetObservationMatrix(h);

  Eigen::Matrix<float, 2, 2> q;
  q.setIdentity();
  q *= 10.0f;
  filter->SetTransitionNoise(q);

  Eigen::Matrix<float, 1, 1> r;
  r.setIdentity();
  r *= 20.0f;
  filter->SetObservationNoise(r);

  Eigen::Matrix<float, 2, 1> b;
  b.setZero();
  filter->SetControlMatrix(b);
}

void ObjectCameraFilter::Create(const int track_id, const double timestamp,
                                const std::shared_ptr<VisualObject>& obj_ptr,
                                const FilterOptions& options) {
  tracked_filters_[track_id] = ObjectFilter();
  tracked_filters_[track_id].track_id_ = track_id;
  tracked_filters_[track_id].last_timestamp_ = timestamp;

  if (!FLAGS_use_navigation_mode) {
    Eigen::Matrix4d cam2world_pose = Eigen::Matrix4d::Identity();
    if (options.camera_trans != nullptr) {
      cam2world_pose = *(options.camera_trans);
    } else {
      AERROR << "Input camera_trans is null";
      return;
    }
    tracked_filters_[track_id].global_to_local_offset_ = Eigen::Vector3d(
        -cam2world_pose(0, 3), -cam2world_pose(1, 3), -cam2world_pose(2, 3));

    TransformPoseGlobal2Local(track_id, &cam2world_pose);
    ADEBUG << "cam2world_pose\n" << cam2world_pose;
    Eigen::Matrix4f cam2world_posef = cam2world_pose.cast<float>();
    TransformObject(obj_ptr, cam2world_posef);
  }

  InitFilter(obj_ptr->center.x(), &(tracked_filters_[track_id].x_));
  InitFilter(obj_ptr->center.y(), &(tracked_filters_[track_id].y_));
  InitFilter(obj_ptr->theta, &(tracked_filters_[track_id].theta_));

  auto x_state_cov = tracked_filters_[track_id].x_.GetStateCovariance();
  auto y_state_cov = tracked_filters_[track_id].y_.GetStateCovariance();
  obj_ptr->state_uncertainty.block(0, 0, 2, 2) << x_state_cov(0, 0), 0, 0,
      y_state_cov(0, 0);
  obj_ptr->state_uncertainty.block(2, 2, 2, 2) << x_state_cov(1, 1), 0, 0,
      y_state_cov(1, 1);
  obj_ptr->state_uncertainty.block(0, 2, 2, 2) << x_state_cov(0, 1), 0, 0,
      y_state_cov(0, 1);
  obj_ptr->state_uncertainty.block(2, 0, 2, 2) << x_state_cov(1, 0), 0, 0,
      y_state_cov(1, 0);
}

void ObjectCameraFilter::TransformPoseGlobal2Local(const int track_id,
                                                   Eigen::Matrix4d* pose) {
  (*pose)(0, 3) += tracked_filters_[track_id].global_to_local_offset_(0);
  (*pose)(1, 3) += tracked_filters_[track_id].global_to_local_offset_(1);
  (*pose)(2, 3) += tracked_filters_[track_id].global_to_local_offset_(2);
}

void ObjectCameraFilter::TransformObject(std::shared_ptr<VisualObject> obj,
                                         const Eigen::Matrix4f& pose) {
  // Transform object with given pose
  Eigen::Vector3f& dir = obj->direction;
  dir = (pose * Eigen::Vector4f(dir[0], dir[1], dir[2], 0)).head(3);
  // transform center
  Eigen::Vector3f& center = obj->center;
  center = (pose * Eigen::Vector4f(center[0], center[1], center[2], 1)).head(3);

  if (fabs(obj->direction[0]) < DBL_MIN) {
    obj->theta = obj->direction(1) > 0 ? M_PI / 2 : -M_PI / 2;
  } else {
    obj->theta = atan2(obj->direction[1], obj->direction[0]);
  }
}

void ObjectCameraFilter::Predict(const int track_id, const double timestamp) {
  double time_diff = timestamp - tracked_filters_[track_id].last_timestamp_;
  float diff = static_cast<float>(time_diff);

  Eigen::Matrix<float, 2, 2> fx =
      tracked_filters_[track_id].x_.GetTransitionMatrix();
  fx(0, 1) = diff;
  tracked_filters_[track_id].x_.SetTransitionMatrix(fx);

  Eigen::Matrix<float, 2, 2> fy =
      tracked_filters_[track_id].y_.GetTransitionMatrix();
  fy(0, 1) = diff;
  tracked_filters_[track_id].y_.SetTransitionMatrix(fy);

  Eigen::Matrix<float, 2, 2> ft =
      tracked_filters_[track_id].theta_.GetTransitionMatrix();
  ft(0, 1) = diff;
  tracked_filters_[track_id].theta_.SetTransitionMatrix(ft);

  tracked_filters_[track_id].x_.Predict();
  tracked_filters_[track_id].y_.Predict();
  tracked_filters_[track_id].theta_.Predict();

  tracked_filters_[track_id].lost_frame_cnt_ = 0;
  tracked_filters_[track_id].last_timestamp_ = timestamp;
}

void ObjectCameraFilter::Update(const int track_id,
                                const std::shared_ptr<VisualObject>& obj_ptr,
                                const FilterOptions& options) {
  if (!FLAGS_use_navigation_mode) {
    Eigen::Matrix4d cam2world_pose = Eigen::Matrix4d::Identity();

    if (options.camera_trans != nullptr) {
      cam2world_pose = *(options.camera_trans);
    } else {
      AERROR << "Input camera_trans is null";
      return;
    }

    TransformPoseGlobal2Local(track_id, &cam2world_pose);
    ADEBUG << "cam2world_pose\n" << cam2world_pose;
    Eigen::Matrix4f cam2world_posef = cam2world_pose.cast<float>();
    TransformObject(obj_ptr, cam2world_posef);
  }

  Eigen::Matrix<float, 1, 1> z_x;
  z_x << obj_ptr->center.x();
  tracked_filters_[track_id].x_.Correct(z_x);

  Eigen::Matrix<float, 1, 1> z_y;
  z_y << obj_ptr->center.y();
  tracked_filters_[track_id].y_.Correct(z_y);

  Eigen::Matrix<float, 1, 1> z_theta;
  z_theta << obj_ptr->theta;
  tracked_filters_[track_id].theta_.Correct(z_theta);
}

void ObjectCameraFilter::GetState(const int track_id,
                                  std::shared_ptr<VisualObject> obj_ptr) {
  auto x_state = tracked_filters_[track_id].x_.GetStateEstimate();
  auto y_state = tracked_filters_[track_id].y_.GetStateEstimate();
  auto x_state_cov = tracked_filters_[track_id].x_.GetStateCovariance();
  auto y_state_cov = tracked_filters_[track_id].y_.GetStateCovariance();

  obj_ptr->center.x() =
      x_state.x() - tracked_filters_[track_id].global_to_local_offset_(0);
  obj_ptr->velocity.x() = x_state.y();

  obj_ptr->center.y() =
      y_state.x() - tracked_filters_[track_id].global_to_local_offset_(1);

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

  obj_ptr->theta = tracked_filters_[track_id].theta_.GetStateEstimate().x();

  if (FLAGS_use_navigation_mode) {
    obj_ptr->direction =
        Eigen::Vector3f(cos(obj_ptr->theta), 0.0f, -sin(obj_ptr->theta));
  } else {
    // obtain direction in world coordinates
    obj_ptr->direction =
        Eigen::Vector3f(cos(obj_ptr->theta), sin(obj_ptr->theta), 0);
  }
}

void ObjectCameraFilter::Destroy() {
  std::vector<int> id_to_destroy;
  for (const auto& p : tracked_filters_) {
    if (p.second.lost_frame_cnt_ > kMaxKeptFrameCnt) {
      id_to_destroy.emplace_back(p.first);
    }
  }

  for (const auto& id : id_to_destroy) {
    tracked_filters_.erase(id);
  }
}

}  // namespace perception
}  // namespace apollo
