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

#include <algorithm>
#include <string>
#include <vector>
#include "modules/common/log.h"
#include "modules/perception/obstacle/common/geometry_util.h"
#include "modules/perception/obstacle/lidar/tracker/hm_tracker/kalman_filter.h"
#include "modules/perception/obstacle/lidar/tracker/hm_tracker/object_track.h"

namespace apollo {
namespace perception {

int ObjectTrack::s_track_idx_ = 0;
std::string ObjectTrack::s_filter_method_ = "kalman_filter";

void ObjectTrack::SetFilterMethod(const std::string& filter_method) {
  // Set filter method for all the track objects
  if (filter_method == "kalman_filter") {
    s_filter_method_ = filter_method;
  } else {
    AINFO << "invalid filter_method!";
  }
  AINFO << "track filter algorithm is " << s_filter_method_;
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

ObjectTrack::ObjectTrack(TrackedObjectPtr obj) {
  // Setup filter
  Eigen::Vector3f initial_anchor_point = obj->anchor_point;
  Eigen::Vector3f initial_velocity = Eigen::Vector3f::Zero();

  if (s_filter_method_ == "kalman_filter") {
    filter_ = new KalmanFilter();
  }

  filter_->Initialize(initial_anchor_point, initial_velocity);

  // Initialize track info
  idx_ = ObjectTrack::GetNextTrackId();
  age_ = 1;
  total_visible_count_ = 1;
  consecutive_invisible_count_ = 0;
  period_ = 0.0;
  current_object_ = obj;
  accumulated_type_probs_ = std::vector<float>(MAX_OBJECT_TYPE, 0.0);
  accumulated_type_probs_ = obj->object_ptr->type_probs;
  type_life_time_ = 0;

  // Initialize track states
  is_static_hypothesis_ = false;
  belief_anchor_point_ = initial_anchor_point;
  belief_velocity_ = initial_velocity;
  belief_velocity_accelaration_ = Eigen::Vector3f::Zero();

  // NEED TO NOTICE: All the states would be collected mainly based on states
  // of tracked object. Thus, update tracked object when you update the state
  // of track !!!!!
  obj->velocity = initial_velocity;
}

ObjectTrack::~ObjectTrack() {
  if (filter_) {
    delete filter_;
    filter_ = NULL;
  }
}

Eigen::VectorXf ObjectTrack::Predict(const double time_diff) {
  // Predict the state of track

  /* Previously, we use the predict of filtering algorithm directly. However, it is hard to
   * ensure that measurements are perfect all the time. To avoid bad estimation generated 
   * from imperfect detection, we use filtering result as prior guidance, while some other
   * post-processing may correct the state of track after filtering. 
   * Thus, aftering predicting of filtering algorithm, we return predicts of states of track.
   * NEED TO NOTICE: predicting is a very necessary part of filtering algorithm !!!!!
   * */

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

void ObjectTrack::UpdateWithObject(TrackedObjectPtr* new_object,
  const double time_diff) {
  // Update track with object
  if ((*new_object) == nullptr) {
    UpdateWithoutObject(time_diff);
  }

  /* 1. update object track */
  // 1.1 check measurement outliers
  bool updating_new_object_is_outlier = false;

  // 1.2 update with object if observation is not outlier
  if (!updating_new_object_is_outlier) {
    filter_->UpdateWithObject((*new_object), current_object_, time_diff);
    filter_->GetState(&belief_anchor_point_, &belief_velocity_);
  } else {
    /* Here, we only update belief anchor point with anchor point of new detected object. In 
     * the future, new method that handle outlier could be designed to handle more complicated
     * strategies. */
    belief_anchor_point_ = (*new_object)->anchor_point;
  }

  // NEED TO NOTICE: All the states would be collected mainly based on states
  // of tracked object. Thus, update tracked object when you update the state
  // of track !!!!!
  (*new_object)->anchor_point = belief_anchor_point_;
  (*new_object)->velocity = belief_velocity_;

  belief_velocity_accelaration_ = ((*new_object)->velocity -
    current_object_->velocity) / time_diff;

  /* Currently, we only considered outliers' influence on motion estimation. Track level 
   * smoothness of orientation & class idx may also take into acount it in the future. */

  // 1.4 update track info
  age_++;
  total_visible_count_++;
  consecutive_invisible_count_ = 0;
  period_ += time_diff;

  // 1.5 update history
  if (history_objects_.size() >= s_max_cached_object_size_) {
    history_objects_.pop_front();
  }
  history_objects_.push_back(current_object_);
  current_object_ = *new_object;

  /* Previously, velocity of object track is smoothed after the smoothing of orientation & type.
   * However, compared to the output of original filter, smoothed velocity, which is a posteriori,
   * is more reasonable to be used in the smoothing of orientation & type. */

  /* Previously, track static hypothesis is checked before smoothing strategies. Now, it has been
   * moved in the method of smooth track velocity. This is more reasonable, as track static 
   * hypothesis is decided by velocity more directly when velocity estimation improved. */

  /* 2. smooth object track */
  // 2.1 smooth velocity
  SmoothTrackVelocity((*new_object), time_diff);

  // 2.2 smooth orientation
  SmoothTrackOrientation();

  // 2.3 smooth class idx
  SmoothTrackClassIdx();
}

void ObjectTrack::UpdateWithoutObject(const double time_diff) {
  // Update track's obstacle object
  TrackedObjectPtr new_obj(new TrackedObject());
  new_obj->clone(*current_object_);

  Eigen::Vector3f predicted_shift = belief_velocity_ * time_diff;
  new_obj->anchor_point = current_object_->anchor_point + predicted_shift;
  new_obj->barycenter = current_object_->barycenter + predicted_shift;
  new_obj->center = current_object_->center + predicted_shift;

  // Update obstacle cloud
  pcl_util::PointCloudPtr pc = new_obj->object_ptr->cloud;
  for (size_t j = 0; j < pc->points.size(); ++j) {
    pc->points[j].x += predicted_shift[0];
    pc->points[j].y += predicted_shift[1];
    pc->points[j].z += predicted_shift[2];
  }

  // Update obstacle polygon
  PolygonDType& polygon = new_obj->object_ptr->polygon;
  for (size_t j = 0; j < polygon.points.size(); ++j) {
    polygon.points[j].x += predicted_shift[0];
    polygon.points[j].y += predicted_shift[1];
    polygon.points[j].z += predicted_shift[2];
  }

  // Update filter
  filter_->UpdateWithoutObject(time_diff);

  // Update states of track
  belief_anchor_point_ = new_obj->anchor_point;

  // NEED TO NOTICE: All the states would be collected mainly based on states
  // of tracked object included temporaryly occluded ones. Thus, update
  // tracked object when you update the state of track !!!!
  new_obj->velocity = belief_velocity_;

  // Update track info
  age_++;
  consecutive_invisible_count_++;
  period_ += time_diff;

  // Update history
  if (history_objects_.size() >= s_max_cached_object_size_) {
    history_objects_.pop_front();
  }
  history_objects_.push_back(current_object_);
  current_object_ = new_obj;
}

void ObjectTrack::UpdateWithoutObject(const Eigen::VectorXf& predict_state,
  const double time_diff) {
  // Update track's obstacle object
  TrackedObjectPtr new_obj(new TrackedObject());
  new_obj->clone(*current_object_);

  Eigen::Vector3f predicted_shift = predict_state.tail(3) * time_diff;
  new_obj->anchor_point = current_object_->anchor_point + predicted_shift;
  new_obj->barycenter = current_object_->barycenter + predicted_shift;
  new_obj->center = current_object_->center + predicted_shift;

  // Update obstacle cloud
  pcl_util::PointCloudPtr pc = new_obj->object_ptr->cloud;
  for (size_t j = 0; j < pc->points.size(); ++j) {
    pc->points[j].x += predicted_shift[0];
    pc->points[j].y += predicted_shift[1];
    pc->points[j].z += predicted_shift[2];
  }

  // Update obstacle polygon
  PolygonDType& polygon = new_obj->object_ptr->polygon;
  for (size_t j = 0; j < polygon.points.size(); ++j) {
    polygon.points[j].x += predicted_shift[0];
    polygon.points[j].y += predicted_shift[1];
    polygon.points[j].z += predicted_shift[2];
  }

  /* No need to update filter*/

  // Update filter
  filter_->UpdateWithoutObject(time_diff);

  // Update states of track
  belief_anchor_point_ = new_obj->anchor_point;

  // NEED TO NOTICE: All the states would be collected mainly based on states
  // of tracked object included temporaryly occluded ones. Thus, update
  // tracked object when you update the state of track !!!!
  new_obj->velocity = belief_velocity_;

  // Update track info
  age_++;
  consecutive_invisible_count_++;
  period_ += time_diff;

  // Update history
  if (history_objects_.size() >= s_max_cached_object_size_) {
    history_objects_.pop_front();
  }
  history_objects_.push_back(current_object_);
  current_object_ = new_obj;
}

void ObjectTrack::SmoothTrackVelocity(const TrackedObjectPtr& new_object,
  const double time_diff) {
  // Smooth velocity over track history
  // 1. keep motion if accelaration of filter is greater than a threshold
  Eigen::Vector3f filter_anchor_point = Eigen::Vector3f::Zero();
  Eigen::Vector3f filter_velocity = Eigen::Vector3f::Zero();
  Eigen::Vector3f filter_velocity_accelaration = Eigen::Vector3f::Zero();
  filter_->GetState(&filter_anchor_point, &filter_velocity,
    &filter_velocity_accelaration);
  double filter_accelaration = filter_velocity_accelaration.norm();
  bool need_keep_motion = filter_accelaration >
    s_claping_accelaration_threshold_;
  // use tighter threshold for pedestrian
  if (filter_accelaration > s_claping_accelaration_threshold_ / 2 &&
    current_object_->object_ptr->type == PEDESTRIAN) {
    need_keep_motion = true;
  }
  if (need_keep_motion) {
    Eigen::Vector3f last_velocity = Eigen::Vector3f::Zero();
    if (history_objects_.size() > 0) {
      last_velocity = history_objects_[history_objects_.size() - 1]->velocity;
    }
    belief_velocity_ = last_velocity;
    belief_velocity_accelaration_ = Eigen::Vector3f::Zero();

    // NEED TO NOTICE: All the states would be collected mainly based on
    // states of tracked object included temporaryly occluded ones. Thus,
    // update tracked object when you update the state of track !!!!
    current_object_->velocity = belief_velocity_;

    // keep static hypothesis
    return;
  }
  // 2. static hypothesis check
  is_static_hypothesis_ = CheckTrackStaticHypothesis(new_object->object_ptr,
    time_diff);

  if (is_static_hypothesis_) {
    belief_velocity_ = Eigen::Vector3f::Zero();
    belief_velocity_accelaration_ = Eigen::Vector3f::Zero();

    // NEED TO NOTICE: All the states would be collected mainly based on
    // states of tracked object included temporaryly occluded ones. Thus,
    // update tracked object when you update the state of track !!!!
    current_object_->velocity = belief_velocity_;
  }
}

void ObjectTrack::SmoothTrackOrientation() {
  // Smooth orientation over track history
  TrackedObjectPtr previous_object = history_objects_.back();
  Eigen::Vector3f previous_dir = previous_object->direction;
  Eigen::Vector3f current_dir = current_object_->direction;
  float previous_speed = previous_object->velocity.head(2).norm();
  float current_speed = current_object_->velocity.head(2).norm();
  if (current_speed > 1.0f) {
    current_dir = current_object_->velocity;
  } else {
    ComputeMostConsistentBboxDirection(previous_dir, &current_dir);
    float previous_weight = 1.0;
    if (previous_speed + current_speed > FLT_EPSILON) {
      previous_weight = (previous_speed + 1) / (previous_speed + current_speed + 1);
    }
    current_dir = previous_weight * previous_dir + (1 - previous_weight) * current_dir;
  }
  current_dir(2) = 0;
  current_dir.normalize();
  Eigen::Vector3d new_size;
  Eigen::Vector3d new_center;
  compute_bbox_size_center_xy<pcl_util::Point>(
    current_object_->object_ptr->cloud,
    current_dir.cast<double>(), new_size, new_center);
  current_object_->direction = current_dir;
  current_object_->center = new_center.cast<float>();
  current_object_->size = new_size.cast<float>();
}

void ObjectTrack::SmoothTrackClassIdx() {
}

bool ObjectTrack::CheckTrackStaticHypothesis(const ObjectPtr& new_object,
  const double time_diff) {
  // Check whether track is static
  bool is_velocity_angle_change =
    CheckTrackStaticHypothesisByVelocityAngleChange(new_object, time_diff);
  // Define track as static & clap velocity to 0
  // when velocity angle change & it is smaller than a threshold
  double speed = belief_velocity_.head(2).norm();
  bool velocity_is_small = speed < (s_claping_speed_threshold_ / 2);
  // use loose threshold for pedestrian
  if (speed < s_claping_speed_threshold_ &&
    current_object_->object_ptr->type != PEDESTRIAN) {
    velocity_is_small = true;
  }
  // Need to notice: claping small velocity may not reasonable when the true
  // velocity of target object is really small. e.g. a moving out vehicle in
  // a parking lot. Thus, instead of clapping all the small velocity, we clap
  // those whose history trajectory or performance is close to a static one.
  if (velocity_is_small && is_velocity_angle_change) {
    return true;
  }
  return false;
}

bool ObjectTrack::CheckTrackStaticHypothesisByVelocityAngleChange(
  const ObjectPtr& new_object,
  const double time_diff) {
  // Sub strategy of checking whether track is static or not - let track be
  // static if velocity angle change from previous and current frame is
  // larger than M_PI / 3.0
  Eigen::Vector3f previous_velocity =
    history_objects_[history_objects_.size() - 1]->velocity;
  Eigen::Vector3f current_velocity = current_object_->velocity;
  double velocity_angle_change = vector_theta_2d_xy(previous_velocity,
    current_velocity);
  // Previously, this threshold is set to PI/3. Now, as the smoothness of the
  // filter is improved by Robust Adaptive Kalman Filter, we would use more
  // strict threshold in the future.
  if (fabs(velocity_angle_change) > M_PI / 4.0) {
    return true;
  }
  return false;
}

/*class ObjectTrackSet*/
ObjectTrackSet::ObjectTrackSet(): age_threshold_(5),
  minimum_visible_ratio_(0.6f),
  maximum_consecutive_invisible_count_(1) {
  tracks_.reserve(1000);
}

ObjectTrackSet::~ObjectTrackSet() {
  std::cout << "Release ObjectTrackSet...\n";
  clear();
  std::cout << "Release ObjectTrackSet End\n";
}

void ObjectTrackSet::clear() {
  for (size_t i = 0; i < tracks_.size(); i++) {
    if (tracks_[i]) {
      delete (tracks_[i]);
      tracks_[i] = NULL;
    }
  }
  tracks_.clear();
}

int ObjectTrackSet::remove_lost_tracks() {
  size_t track_num = 0;
  for (size_t i = 0; i < tracks_.size(); i++) {
    if (tracks_[i]->age_ < age_threshold_ &&
      tracks_[i]->consecutive_invisible_count_ > 1) {
      continue;
    }

    float visible_ratio = tracks_[i]->total_visible_count_ * 1.0f /
      tracks_[i]->age_;
    if (visible_ratio < minimum_visible_ratio_) {
      continue;
    }

    if (tracks_[i]->consecutive_invisible_count_
      > maximum_consecutive_invisible_count_) {
      continue;
    }

    if (i == track_num) {
      track_num++;
      continue;
    } else {
      ObjectTrackPtr tmp = tracks_[i];
      tracks_[i] = tracks_[track_num];
      tracks_[track_num] = tmp;
      track_num++;
    }
  }

  int no_removed = tracks_.size() - track_num;
  for (size_t i = track_num; i < tracks_.size(); i++) {
    if (tracks_[i] != NULL) {
      delete (tracks_[i]);
      tracks_[i] = NULL;
    }
  }
  tracks_.resize(track_num);
  return no_removed;
}

}  // namespace perception
}  // namespace apollo
