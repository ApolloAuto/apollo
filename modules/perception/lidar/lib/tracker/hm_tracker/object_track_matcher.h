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
#ifndef PERCEPTION_LIDAR_LIB_TRACKER_HM_TRACKER_OBJECT_TRACK_MATCHER_H_
#define PERCEPTION_LIDAR_LIB_TRACKER_HM_TRACKER_OBJECT_TRACK_MATCHER_H_

#include <Eigen/Core>
#include <string>
#include <utility>
#include <vector>

#include "modules/perception/common/graph/secure_matrix.h"
#include "modules/perception/lidar/lib/interface/base_bipartite_graph_matcher.h"
#include "modules/perception/lidar/lib/tracker/common/track_data.h"
#include "modules/perception/lidar/lib/tracker/common/tracked_object.h"
#include "modules/perception/lidar/lib/tracker/hm_tracker/track_object_distance.h"

namespace apollo {
namespace perception {
namespace lidar {

struct ObjectTrackMatcherInitOptions {
  bool is_background = false;
  std::string matcher_name;
};

struct ObjectTrackMatcherOptions {};

class ObjectTrackMatcher {
 public:
  ObjectTrackMatcher() = default;
  ~ObjectTrackMatcher() = default;

  typedef std::pair<size_t, size_t> TrackObjectPair;

  // init distance and matcher
  bool Init(const ObjectTrackMatcherInitOptions &options);

  // set method
  // bool set_match_method(std::string method);

  // @brief set max Match distance for matcher object
  // @params[IN] max_match_distance: threshold of Match distance
  // @return true if set successfuly, otherwise return false
  bool set_max_match_distance(float max_match_distance);
  // @brief set bound value for matcher
  // @params[IN] bound_value, 100 * max_match_diastance would be effecitive
  // @return true is set successfully
  bool set_bound_value(float bound_value);
  // @brief Match detected objects to tracks
  // @params[IN] objects: new detected objects for matching
  // @params[IN] tracks: maintaining tracks for matching
  // @params[IN] time_diff: time interval from last matching
  // @params[OUT] assignments: assignment pair of object & track
  // @params[OUT] unassigned_tracks: tracks without matched object
  // @params[OUT] unassigned_objects: objects without matched track
  // @return nothing
  void Match(const ObjectTrackMatcherOptions &options,
             std::vector<TrackedObjectPtr> &objects,
             const std::vector<TrackDataPtr> &tracks,
             const std::vector<Eigen::VectorXf> &tracks_predict,
             const double time_diff, std::vector<TrackObjectPair> *assignments,
             std::vector<size_t> *unassigned_tracks,
             std::vector<size_t> *unassigned_objects);

  common::SecureMat<float> *CostMatrix() { return matcher_->cost_matrix(); }

  std::string Name() const { return "ObjectTrackMatcher"; }

 protected:
  // @brief compute association matrix
  // @params[IN] tracks: maintained tracks for matching
  // @params[IN] tracks_predict: predicted states of maintained tracks
  // @params[IN] new_objects: new detected objects for matching
  // @params[IN] time_diff: time interval from last matching
  // @params[OUT] association_mat: matrix of association distance
  // @return nothing
  void ComputeAssociateMatrix(
      const std::vector<TrackDataPtr> &tracks,
      const std::vector<Eigen::VectorXf> &tracks_predict,
      const std::vector<TrackedObjectPtr> &new_objects, const double time_diff,
      common::SecureMat<float> *association_mat);

  // @brief compute distance between track & object
  // @params[IN] track: track for matching
  // @params[IN] track_predict: predicted states of track for matching
  // @params[IN] new_object: new detected object for matching
  // @params[IN] time_diff: time interval from last matching
  // @return distance of given track & object
  float ComputeTrackObjectDistance(const TrackDataPtr &track,
                                   const Eigen::VectorXf &track_predict,
                                   const TrackedObjectPtr &new_object,
                                   const double time_diff);

  ObjectTrackMatcher(const ObjectTrackMatcher &) = delete;
  ObjectTrackMatcher &operator=(const ObjectTrackMatcher &) = delete;

 protected:
  std::shared_ptr<TrackObjectDistance> track_object_distance_;
  std::shared_ptr<BaseBipartiteGraphMatcher> matcher_;

  // threshold of matching
  float bound_value_ = 0.0f;
  float max_match_distance_ = 0.0f;
};  // class ObjectTrackMatcher

}  // namespace lidar
}  // namespace perception
}  // namespace apollo

#endif  // PERCEPTION_LIDAR_LIB_TRACKER_HM_TRACKER_OBJECT_TRACK_MATCHER_H_
