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

#include "modules/perception/obstacle/lidar/tracker/hm_tracker/object_track.h"

#include <algorithm>

#include "modules/common/log.h"
#include "modules/common/util/util.h"
#include "modules/map/hdmap/hdmap.h"
#include "modules/perception/common/geometry_util.h"
#include "modules/perception/obstacle/lidar/tracker/hm_tracker/kalman_filter.h"

namespace apollo {
namespace perception {

int ObjectTrack::s_track_idx_ = 0;
FilterType ObjectTrack::s_filter_method_ = KALMAN_FILTER;
int ObjectTrackSet::s_track_consecutive_invisible_maximum_ = 1;
float ObjectTrackSet::s_track_visible_ratio_minimum_ = 0.6;
int ObjectTrack::s_track_cached_history_size_maximum_ = 5;
double ObjectTrack::s_acceleration_noise_maximum_ = 5;
double ObjectTrack::s_speed_noise_maximum_ = 0.4;

bool ObjectTrack::SetFilterMethod(const std::string& filter_method_name) {
  if (filter_method_name == "kalman_filter") {
    s_filter_method_ = KALMAN_FILTER;
    AINFO << "filter method of object track is " << filter_method_name;
    return true;
  }
  AERROR << "invalid filter method name of object track!";
  return false;
}

bool ObjectTrack::SetTrackCachedHistorySizeMaximum(
    const int& track_cached_history_size_maximum) {
  if (track_cached_history_size_maximum > 0) {
    s_track_cached_history_size_maximum_ = track_cached_history_size_maximum;
    AINFO << "track cached history size maximum of object track is "
          << s_track_cached_history_size_maximum_;
    return true;
  }
  AERROR << "invalid track cached history size maximum of object track!";
  return false;
}

bool ObjectTrack::SetSpeedNoiseMaximum(const double& speed_noise_maximum) {
  if (speed_noise_maximum > 0) {
    s_speed_noise_maximum_ = speed_noise_maximum;
    AINFO << "speed noise maximum of object track is "
          << s_speed_noise_maximum_;
    return true;
  }
  AERROR << "invalid speed noise maximum of object track!";
  return false;
}

bool ObjectTrack::SetAccelerationNoiseMaximum(
    const double& acceleration_noise_maximum) {
  if (acceleration_noise_maximum > 0) {
    s_acceleration_noise_maximum_ = acceleration_noise_maximum;
    AINFO << "acceleration noise maximum of object track is "
          << s_acceleration_noise_maximum_;
    return true;
  }
  AERROR << "invalid acceleration noise maximum of object track!";
  return false;
}

int ObjectTrack::GetNextTrackId() {
  // Get next avaiable track id
  int ret_track_id = s_track_idx_;
  if (s_track_idx_ == INT_MAX) {
    s_track_idx_ = 0;
  } else {
    s_track_idx_++;
  }
  return ret_track_id;
}

ObjectTrack::ObjectTrack(std::shared_ptr<TrackedObject> obj) {
  // Initialize filter
  Eigen::Vector3f initial_anchor_point = obj->anchor_point;
  Eigen::Vector3f initial_velocity = Eigen::Vector3f::Zero();
  if (s_filter_method_ == KALMAN_FILTER) {
    filter_ = new KalmanFilter();
  } else {
    filter_ = new KalmanFilter();
    AWARN << "invalid filter method! default filter (KalmanFilter) in use!";
  }
  filter_->Initialize(initial_anchor_point, initial_velocity);

  // Initialize track info
  idx_ = ObjectTrack::GetNextTrackId();
  age_ = 1;
  total_visible_count_ = 1;
  consecutive_invisible_count_ = 0;
  period_ = 0.0;
  current_object_ = obj;

  // Initialize track states
  is_static_hypothesis_ = false;
  belief_anchor_point_ = initial_anchor_point;
  belief_velocity_ = initial_velocity;
  const double uncertainty_factor = 5.0;
  belief_velocity_uncertainty_ =
      Eigen::Matrix3f::Identity() * uncertainty_factor;
  belief_velocity_accelaration_ = Eigen::Vector3f::Zero();
  // NEED TO NOTICE: All the states would be collected mainly based on states
  // of tracked object. Thus, update tracked object when you update the state
  // of track !!!!!
  obj->velocity = belief_velocity_;
  obj->velocity_uncertainty = belief_velocity_uncertainty_;

  // Initialize object direction with its lane direction
  obj->direction = obj->lane_direction;
}

ObjectTrack::~ObjectTrack() {
  if (filter_) {
    delete filter_;
    filter_ = nullptr;
  }
}

Eigen::VectorXf ObjectTrack::Predict(const double& time_diff) {
  // Get the predict of filter
  Eigen::VectorXf filter_predict = filter_->Predict(time_diff);
  // Get the predict of track
  Eigen::VectorXf track_predict = filter_predict;
  track_predict(0) = belief_anchor_point_(0) + belief_velocity_(0) * time_diff;
  track_predict(1) = belief_anchor_point_(1) + belief_velocity_(1) * time_diff;
  track_predict(2) = belief_anchor_point_(2) + belief_velocity_(2) * time_diff;
  track_predict(3) = belief_velocity_(0);
  track_predict(4) = belief_velocity_(1);
  track_predict(5) = belief_velocity_(2);
  return track_predict;
}

void ObjectTrack::UpdateWithObject(std::shared_ptr<TrackedObject>* new_object,
                                   const double& time_diff) {
  ACHECK(new_object != nullptr) << "Update object with nullptr object";
  // A. update object track
  // A.1 update filter
  filter_->UpdateWithObject((*new_object), current_object_, time_diff);
  filter_->GetState(&belief_anchor_point_, &belief_velocity_);
  filter_->GetOnlineCovariance(&belief_velocity_uncertainty_);
  // NEED TO NOTICE: All the states would be collected mainly based on states
  // of tracked object. Thus, update tracked object when you update the state
  // of track !!!!!
  (*new_object)->anchor_point = belief_anchor_point_;
  (*new_object)->velocity = belief_velocity_;
  (*new_object)->velocity_uncertainty = belief_velocity_uncertainty_;

  belief_velocity_accelaration_ =
      ((*new_object)->velocity - current_object_->velocity) / time_diff;
  // A.2 update track info
  ++age_;
  total_visible_count_++;
  consecutive_invisible_count_ = 0;
  period_ += time_diff;
  // A.3 update history
  int history_size = history_objects_.size();
  if (history_size >= s_track_cached_history_size_maximum_) {
    history_objects_.pop_front();
  }
  history_objects_.push_back(current_object_);
  current_object_ = *new_object;

  // B. smooth object track
  // B.1 smooth velocity
  SmoothTrackVelocity((*new_object), time_diff);
  // B.2 smooth orientation
  SmoothTrackOrientation();
}

void ObjectTrack::UpdateWithoutObject(const double& time_diff) {
  // A. update object of track
  std::shared_ptr<TrackedObject> new_obj(new TrackedObject());
  new_obj->clone(*current_object_);
  Eigen::Vector3f predicted_shift = belief_velocity_ * time_diff;
  new_obj->anchor_point = current_object_->anchor_point + predicted_shift;
  new_obj->barycenter = current_object_->barycenter + predicted_shift;
  new_obj->center = current_object_->center + predicted_shift;

  // B. update cloud & polygon
  pcl_util::PointCloudPtr pc = new_obj->object_ptr->cloud;
  for (size_t j = 0; j < pc->points.size(); ++j) {
    pc->points[j].x += predicted_shift[0];
    pc->points[j].y += predicted_shift[1];
    pc->points[j].z += predicted_shift[2];
  }
  PolygonDType& polygon = new_obj->object_ptr->polygon;
  for (size_t j = 0; j < polygon.points.size(); ++j) {
    polygon.points[j].x += predicted_shift[0];
    polygon.points[j].y += predicted_shift[1];
    polygon.points[j].z += predicted_shift[2];
  }

  // C. update filter
  filter_->UpdateWithoutObject(time_diff);

  // D. update states of track
  belief_anchor_point_ = new_obj->anchor_point;
  // NEED TO NOTICE: All the states would be collected mainly based on states
  // of tracked object. Thus, update tracked object when you update the state
  // of track !!!!
  new_obj->velocity = belief_velocity_;
  new_obj->velocity_uncertainty = belief_velocity_uncertainty_;

  // E. update track info
  ++age_;
  consecutive_invisible_count_++;
  period_ += time_diff;

  // F. update history
  int history_size = history_objects_.size();
  if (history_size >= s_track_cached_history_size_maximum_) {
    history_objects_.pop_front();
  }
  history_objects_.push_back(current_object_);
  current_object_ = new_obj;
}

void ObjectTrack::UpdateWithoutObject(const Eigen::VectorXf& predict_state,
                                      const double& time_diff) {
  // A. update object of track
  std::shared_ptr<TrackedObject> new_obj(new TrackedObject());
  new_obj->clone(*current_object_);
  Eigen::Vector3f predicted_shift = predict_state.tail(3) * time_diff;
  new_obj->anchor_point = current_object_->anchor_point + predicted_shift;
  new_obj->barycenter = current_object_->barycenter + predicted_shift;
  new_obj->center = current_object_->center + predicted_shift;

  // B. update cloud & polygon
  pcl_util::PointCloudPtr pc = new_obj->object_ptr->cloud;
  for (size_t j = 0; j < pc->points.size(); ++j) {
    pc->points[j].x += predicted_shift[0];
    pc->points[j].y += predicted_shift[1];
    pc->points[j].z += predicted_shift[2];
  }
  PolygonDType& polygon = new_obj->object_ptr->polygon;
  for (size_t j = 0; j < polygon.points.size(); ++j) {
    polygon.points[j].x += predicted_shift[0];
    polygon.points[j].y += predicted_shift[1];
    polygon.points[j].z += predicted_shift[2];
  }

  // C. update filter without object
  filter_->UpdateWithoutObject(time_diff);

  // D. update states of track
  belief_anchor_point_ = new_obj->anchor_point;
  // NEED TO NOTICE: All the states would be collected mainly based on states
  // of tracked object. Thus, update tracked object when you update the state
  // of track !!!!
  new_obj->velocity = belief_velocity_;
  new_obj->velocity_uncertainty = belief_velocity_uncertainty_;

  // E. update track info
  ++age_;
  consecutive_invisible_count_++;
  period_ += time_diff;

  // F. update history
  int history_size = history_objects_.size();
  if (history_size >= s_track_cached_history_size_maximum_) {
    history_objects_.pop_front();
  }
  history_objects_.push_back(current_object_);
  current_object_ = new_obj;
}

void ObjectTrack::SmoothTrackVelocity(
    const std::shared_ptr<TrackedObject>& new_object, const double& time_diff) {
  // A. keep motion if accelaration of filter is greater than a threshold
  Eigen::Vector3f filter_acceleration_gain = Eigen::Vector3f::Zero();
  filter_->GetAccelerationGain(&filter_acceleration_gain);
  double filter_accelaration = filter_acceleration_gain.norm();
  bool need_keep_motion = filter_accelaration > s_acceleration_noise_maximum_;
  if (need_keep_motion) {
    Eigen::Vector3f last_velocity = Eigen::Vector3f::Zero();
    if (history_objects_.size() > 0) {
      last_velocity = history_objects_[history_objects_.size() - 1]->velocity;
    }
    belief_velocity_ = last_velocity;
    belief_velocity_accelaration_ = Eigen::Vector3f::Zero();
    // NEED TO NOTICE: All the states would be collected mainly based on states
    // of tracked object. Thus, update tracked object when you update the state
    // of track !!!!
    current_object_->velocity = belief_velocity_;
    // keep static hypothesis
    return;
  }
  // B. static hypothesis check & claping noise
  is_static_hypothesis_ =
      CheckTrackStaticHypothesis(new_object->object_ptr, time_diff);
  if (is_static_hypothesis_) {
    belief_velocity_ = Eigen::Vector3f::Zero();
    belief_velocity_accelaration_ = Eigen::Vector3f::Zero();
    // NEED TO NOTICE: All the states would be collected mainly based on states
    // of tracked object. Thus, update tracked object when you update the state
    // of track !!!!
    current_object_->velocity = belief_velocity_;
  }
}

void ObjectTrack::SmoothTrackOrientation() {
  Eigen::Vector3f current_dir = current_object_->direction;
  float current_speed = current_object_->velocity.head(2).norm();
  bool velocity_is_obvious = current_speed > (s_speed_noise_maximum_ * 2);
  if (velocity_is_obvious) {
    current_dir = current_object_->velocity;
  } else {
    current_dir = current_object_->lane_direction;
  }
  current_dir(2) = 0;
  current_dir.normalize();
  Eigen::Vector3d new_size;
  Eigen::Vector3d new_center;
  ComputeBboxSizeCenter<pcl_util::Point>(current_object_->object_ptr->cloud,
                                         current_dir.cast<double>(), &new_size,
                                         &new_center);
  current_object_->direction = current_dir;
  current_object_->center = new_center.cast<float>();
  current_object_->size = new_size.cast<float>();
}

bool ObjectTrack::CheckTrackStaticHypothesis(
    const std::shared_ptr<Object>& new_object, const double& time_diff) {
  // A. check whether track velocity angle changed obviously
  bool is_velocity_angle_change =
      CheckTrackStaticHypothesisByVelocityAngleChange(new_object, time_diff);

  // B. evaluate velocity level
  double speed = belief_velocity_.head(2).norm();
  bool velocity_is_noise = speed < (s_speed_noise_maximum_ / 2);
  bool velocity_is_small = speed < (s_speed_noise_maximum_ / 1);
  if (velocity_is_noise) {
    return true;
  }
  // NEED TO NOTICE: claping small velocity may not reasonable when the true
  // velocity of target object is really small. e.g. a moving out vehicle in
  // a parking lot. Thus, instead of clapping all the small velocity, we clap
  // those whose history trajectory or performance is close to a static one.
  if (velocity_is_small && is_velocity_angle_change) {
    return true;
  }
  return false;
}

bool ObjectTrack::CheckTrackStaticHypothesisByVelocityAngleChange(
    const std::shared_ptr<Object>& new_object, const double& time_diff) {
  Eigen::Vector3f previous_velocity =
      history_objects_[history_objects_.size() - 1]->velocity;
  Eigen::Vector3f current_velocity = current_object_->velocity;
  double velocity_angle_change =
      VectorTheta2dXy(previous_velocity, current_velocity);
  if (fabs(velocity_angle_change) > M_PI / 4.0) {
    return true;
  }
  return false;
}

/*class ObjectTrackSet*/
ObjectTrackSet::ObjectTrackSet() { tracks_.reserve(1000); }

ObjectTrackSet::~ObjectTrackSet() { Clear(); }

bool ObjectTrackSet::SetTrackConsecutiveInvisibleMaximum(
    const int& track_consecutive_invisible_maximum) {
  if (track_consecutive_invisible_maximum >= 0) {
    s_track_consecutive_invisible_maximum_ =
        track_consecutive_invisible_maximum;
    AINFO << "track consecutive invisible maximum of object track set is "
          << s_track_consecutive_invisible_maximum_;
    return true;
  }
  AERROR << "invalid track consecutive invisible maximum of object track! ";
  return false;
}

bool ObjectTrackSet::SetTrackVisibleRatioMinimum(
    const float& track_visible_ratio_minimum) {
  if (track_visible_ratio_minimum >= 0 && track_visible_ratio_minimum <= 1) {
    s_track_visible_ratio_minimum_ = track_visible_ratio_minimum;
    AINFO << "track visible ratio minimum of object track set is "
          << s_track_visible_ratio_minimum_;
    return true;
  }
  AERROR << "invalid track visible ratio minimum of object track! ";
  return false;
}

void ObjectTrackSet::Clear() {
  for (size_t i = 0; i < tracks_.size(); i++) {
    if (tracks_[i]) {
      delete (tracks_[i]);
      tracks_[i] = nullptr;
    }
  }
  tracks_.clear();
}

int ObjectTrackSet::RemoveLostTracks() {
  size_t track_num = 0;
  for (size_t i = 0; i < tracks_.size(); ++i) {
    // A. remove tracks invisible ratio less than given minimum
    float track_visible_ratio =
        tracks_[i]->total_visible_count_ * 1.0f / tracks_[i]->age_;
    if (track_visible_ratio < s_track_visible_ratio_minimum_) continue;
    // B. remove tracks consecutive invisible count greater than given maximum
    int track_consecutive_invisible_count =
        tracks_[i]->consecutive_invisible_count_;
    if (track_consecutive_invisible_count >
        s_track_consecutive_invisible_maximum_)
      continue;
    // C. update
    if (i == track_num) {
      track_num++;
    } else {
      ObjectTrackPtr tmp = tracks_[i];
      tracks_[i] = tracks_[track_num];
      tracks_[track_num] = tmp;
      track_num++;
    }
  }
  // remove lost tracks
  int no_removed = tracks_.size() - track_num;
  for (size_t i = track_num; i < tracks_.size(); ++i) {
    if (tracks_[i] != nullptr) {
      delete (tracks_[i]);
      tracks_[i] = nullptr;
    }
  }
  tracks_.resize(track_num);
  return no_removed;
}

}  // namespace perception
}  // namespace apollo
