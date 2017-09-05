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

#ifndef MODULES_PERCEPTION_LIDAR_TRACKER_HM_TRACKER_HUNGARIAN_MATCHER_H_
#define MODULES_PERCEPTION_LIDAR_TRACKER_HM_TRACKER_HUNGARIAN_MATCHER_H_

#include <string>
#include <vector>
#include "modules/perception/obstacle/lidar/tracker/hm_tracker/base_matcher.h"

namespace apollo {
namespace perception {

class HungarianMatcher: public BaseMatcher{
 public:
  HungarianMatcher() {}
  ~HungarianMatcher() {}

  // @brief set max match distance for matcher object
  // @params[IN] max_match_distance: threshold of match distance
  // @return true if set successfuly, otherwise return false
  static bool SetMaxMatchDistance(float max_match_distance);

  // @brief match detected objects to tracks
  // @params[IN] objects: new detected objects for matching
  // @params[IN] tracks: maintaining tracks for matching
  // @params[IN] time_diff: time interval from last matching
  // @params[OUT] assignments: assignment pair of object & track
  // @params[OUT] unassigned_tracks: tracks without matched object
  // @params[OUT] unassigned_objects: objects without matched track
  // @return nothing
  void Match(
    std::vector<TrackedObjectPtr>* objects,
    const std::vector<ObjectTrackPtr>& tracks,
    const std::vector<Eigen::VectorXf>& tracks_predict,
    const double time_diff,
    std::vector<TrackObjectPair>* assignments,
    std::vector<int>* unassigned_tracks,
    std::vector<int>* unassigned_objects);

  // @brief match detected objects to tracks in component level
  // @params[IN] association_mat: association matrix of all objects to tracks
  // @params[IN] track_component: component of track
  // @params[IN] object_component: component of object
  // @params[OUT] sub_assignments: component assignment pair of object & track
  // @params[OUT] sub_unassigned_tracks: component tracks not matched
  // @params[OUT] sub_unasgined_objects: component objects not matched
  void MatchComponents(
    const Eigen::MatrixXf association_mat,
    const std::vector<int> track_component,
    const std::vector<int> obj_component,
    std::vector<TrackObjectPair>* sub_assignments,
    std::vector<int>* sub_unassigned_tracks,
    std::vector<int>* sub_unassigned_objects);

  std::string Name() const {
    return "HungarianMatcher";
  }

 protected:
  // @brief compute association matrix
  // @params[IN] tracks: maintained tracks for matching
  // @params[IN] tracks_predict: predicted states of maintained tracks
  // @params[IN] new_objects: new detected objects for matching
  // @params[IN] time_diff: time interval from last matching
  // @params[OUT] association_mat: matrix of association distance
  // @return nothing
  void ComputeAssociateMatrix(
    const std::vector<ObjectTrackPtr>& tracks,
    const std::vector<Eigen::VectorXf>& tracks_predict,
    const std::vector<TrackedObjectPtr>& new_objects,
    const double time_diff,
    Eigen::MatrixXf* association_mat);

  // @brief compute distance between track & object
  // @params[IN] track: track for matching
  // @params[IN] track_predict: predicted states of track for matching
  // @params[IN] new_object: new detected object for matching
  // @params[IN] time_diff: time interval from last matching
  // @return distance of given track & object
  float ComputeTrackObjectDistance(
    const ObjectTrackPtr& track,
    const Eigen::VectorXf& track_predict,
    const TrackedObjectPtr& new_object,
    const double time_diff) const;

  // @brief compute connected components within given threshold
  // @params[IN] association_mat: matrix of association distance
  // @params[IN] connected_threshold: threshold of connected components
  // @params[OUT] track_components: connected objects of given tracks
  // @params[OUT] obj_components: connected tracks of given objects
  // @return nothing
  void ComputeConnectedComponents(
    const Eigen::MatrixXf& association_mat,
    const float connected_threshold,
    std::vector<std::vector<int> >* track_components,
    std::vector<std::vector<int> >* obj_components);

  // @brief assign objects to tracks using components
  // @params[IN] association_mat: matrix of association distance
  // @params[IN] max_distance: threshold of matching
  // @params[OUT] assignments: assignment pair of matched object & track
  // @params[OUT] unassigned_tracks: tracks without matched object
  // @params[OUT] unassigned_objects: objects without matched track
  // @return nothing
  void AssignObjectsToTracks(
    const Eigen::MatrixXf& association_mat,
    const double max_distance,
    std::vector<TrackObjectPair>* assignments,
    std::vector<int>* unassigned_tracks,
    std::vector<int>* unassigned_objects);

 private:
    // threshold of matching
    static float                s_max_match_distance_;

    DISALLOW_COPY_AND_ASSIGN(HungarianMatcher);
};  // class HmMatcher

}  // namespace perception
}  // namespace apollo

#endif  // MODULES_PERCEPTION_LIDAR_TRACKER_HM_TRACKER_HUNGARIAN_MATCHER_H_
