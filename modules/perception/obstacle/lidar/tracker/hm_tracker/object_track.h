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

#ifndef MODULES_PERCEPTION_OBSTACLE_LIDAR_TRACKER_HM_TRACKER_OBJECT_TRACK_H_
#define MODULES_PERCEPTION_OBSTACLE_LIDAR_TRACKER_HM_TRACKER_OBJECT_TRACK_H_

#include <boost/shared_ptr.hpp>
#include <deque>
#include <queue>
#include <string>
#include <vector>

#include "Eigen/Core"
#include "modules/common/macro.h"
#include "modules/perception/obstacle/base/object.h"
#include "modules/perception/obstacle/lidar/tracker/hm_tracker/base_filter.h"
#include "modules/perception/obstacle/lidar/tracker/hm_tracker/tracked_object.h"

namespace apollo {
namespace perception {

class ObjectTrack {
 public:
  explicit ObjectTrack(TrackedObjectPtr obj);
  ~ObjectTrack();

  // @brief set filter method for all the object track objects
  // @params[IN] filter_method: method name of filtering algorithm
  // @return nothing
  static void SetFilterMethod(const FilterType& filter_method);

  // @brief get next avaiable track id
  // @return next avaiable track id
  static int GetNextTrackId();

  // @brief predict the state of track
  // @params[IN] time_diff: time interval for predicting
  // @return predicted states of track
  Eigen::VectorXf Predict(const double time_diff);

  // @brief update track with object
  // @params[IN] new_object: new object for current updating
  // @params[IN] time_diff: time interval from last updating
  // @return nothing
  void UpdateWithObject(TrackedObjectPtr* new_object, const double time_diff);

  // @brief update track without object
  // @params[IN] time_diff: time interval from last updating
  // @return nothing
  void UpdateWithoutObject(const double time_diff);

  // @brief update track without object with given predicted state
  // @params[IN] predict_state: given predicted state of track
  // @params[IN] time_diff: time interval from last updating
  // @return nothing
  void UpdateWithoutObject(const Eigen::VectorXf& predict_state,
                           const double time_diff);

 protected:
  // @brief smooth velocity over track history
  // @params[IN] new_object: new detected object for updating
  // @params[IN] time_diff: time interval from last updating
  // @return nothing
  void SmoothTrackVelocity(const TrackedObjectPtr& new_object,
                           const double time_diff);

  // @brief smooth orientation over track history
  // @return nothing
  void SmoothTrackOrientation();

  // @brief smooth track class idx over track history
  // @return nothing
  void SmoothTrackClassIdx();

  // @brief check whether track is static or not
  // @params[IN] new_object: new detected object just updated
  // @params[IN] time_diff: time interval between last two updating
  // @return true if track is static, otherwise return false
  bool CheckTrackStaticHypothesis(const ObjectPtr& new_object,
                                  const double time_diff);

  // @brief sub strategy of checking whether track is static or not via
  // considering the velocity angle change
  // @params[IN] new_object: new detected object just updated
  // @params[IN] time_diff: time interval between last two updating
  // @return true if track is static, otherwise return false
  bool CheckTrackStaticHypothesisByVelocityAngleChange(
    const ObjectPtr& new_object,
    const double time_diff);

 private:
  ObjectTrack();

 public:
  // algorithm setup
  static FilterType s_filter_method_;
  BaseFilter* filter_;

  // basic info
  int idx_;
  int age_;
  int total_visible_count_;
  int consecutive_invisible_count_;
  double period_;

  TrackedObjectPtr current_object_;

  // history
  std::deque<TrackedObjectPtr> history_objects_;

  // history type info
  std::vector<float> accumulated_type_probs_;
  int type_life_time_;

  // states
  // NEED TO NOTICE: All the states would be collected mainly based on states
  // of tracked object. Thus, update tracked object when you update the state
  // of track !!!!!
  bool is_static_hypothesis_;
  Eigen::Vector3f belief_anchor_point_;
  Eigen::Vector3f belief_velocity_;
  Eigen::Vector3f belief_velocity_accelaration_;

 private:
  // global setup
  static int s_track_idx_;
  static const int s_max_cached_object_size_ = 20;
  static constexpr double s_claping_speed_threshold_ = 0.4;
  static constexpr double s_claping_accelaration_threshold_ = 10;

  DISALLOW_COPY_AND_ASSIGN(ObjectTrack);
};  // class ObjectTrack

typedef ObjectTrack* ObjectTrackPtr;

class ObjectTrackSet {
 public:
  ObjectTrackSet();
  ~ObjectTrackSet();

  inline std::vector<ObjectTrackPtr>& get_tracks() {
    return tracks_;
  }

  inline const std::vector<ObjectTrackPtr>& get_tracks() const {
    return tracks_;
  }

  inline int size() const {
    return tracks_.size();
  }

  void add_track(const ObjectTrackPtr& track) {
    tracks_.push_back(track);
  }

  int remove_lost_tracks();

  void clear();

 private:
  std::vector<ObjectTrackPtr> tracks_;
  int age_threshold_;
  double minimum_visible_ratio_;
  int maximum_consecutive_invisible_count_;
};  // class ObjectTrackSet

}  // namespace perception
}  // namespace apollo

#endif  // MODULES_PERCEPTION_OBSTACLE_LIDAR_TRACKER_HM_TRACKER_OBJECT_TRACK_H_
