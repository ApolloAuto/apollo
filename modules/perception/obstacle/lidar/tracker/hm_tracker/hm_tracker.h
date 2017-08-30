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

#ifndef MODULES_PERCEPTION_OBSTACLE_LIDAR_TRACKER_HM_TRACKER_H_
#define MODULES_PERCEPTION_OBSTACLE_LIDAR_TRACKER_HM_TRACKER_H_

#include <string>
#include <utility>
#include <vector>
#include "modules/common/macro.h"
#include "modules/perception/obstacle/base/object.h"
#include "modules/perception/obstacle/lidar/interface/base_tracker.h"
#include "modules/perception/obstacle/lidar/tracker/hm_tracker/base_matcher.h"
#include "modules/perception/obstacle/lidar/tracker/hm_tracker/object_track.h"
#include "modules/perception/obstacle/lidar/tracker/hm_tracker/tracked_object.h"

namespace apollo {
namespace perception {

class HmObjectTracker : public BaseTracker{
 public:
  typedef std::pair<int, int>           TrackObjectPair;

  HmObjectTracker();
  virtual ~HmObjectTracker();

  // @brief initialize tracker's configs
  // @return true if initialize successfully, otherwise return false
  bool Init();

  // @brief track detected objects over consecutive frames
  // @params[IN] objects: new detected objects for tracking
  // @params[IN] timestamp: current timestamp for tracking
  // @params[IN] options: tracker options contain information like pose
  // @params[OUT] tracked_objects: tracked objects include objects may be
  // occluded temporaryly
  // @return true if track successfully, otherwise return false
  bool Track(const std::vector<ObjectPtr>& objects,
             double timestamp,
             const TrackerOptions& options,
             std::vector<ObjectPtr>* tracked_objects);

  // @brief get object tracks of tracker
  // @return object tracks maintained in tracker
  const std::vector<ObjectTrackPtr>& GetObjectTracks() const;

  // @brief get pose of velodyne to local
  // @return matrix of pose of velodyne to local
  Eigen::Matrix4d GetPose() const;

  std::string name() const {
    return "HmObjectTracker";
  }

 protected:
  // @brief initialize tracker after obtaining first frame's detections
  // @params[IN] objects: new objects for tracking
  // @params[IN] timestamp: current timestamp for tracking
  // @params[IN] options: tracker options contain information like pose
  // @params[IN] tracked_objects: tracked  objects
  // @return true if initialize successfully, otherwise return false
  bool Initialize(const std::vector<ObjectPtr>& objects,
                  double timestamp,
                  const TrackerOptions& options,
                  std::vector<ObjectPtr>* tracked_objects);

  // @brief transform v2world pose to v2local pose intend to avoid big float
  // value
  // @params[IN] pose: v2world pose
  // @return nothing
  void TransformPoseGlobal2Local(Eigen::Matrix4d* pose);

  // @brief construct tracked objects via necessray transformation & feature
  // computing
  // @params[IN] objects: new objects for constructions
  // @params[OUT] tracked_objects: constructed objects for tracking
  // @params[IN] pose: pose using for coordinate transformation
  // @return nothing
  void ConstructTrackedObjects(const std::vector<ObjectPtr>& objects,
                               std::vector<TrackedObjectPtr>* tracked_objects,
                               const Eigen::Matrix4d& pose);

  // @brief compute objects' shape feature
  // @params[IN] object: object for computing shape feature
  // @return nothing
  void ComputeShapeFeatures(TrackedObjectPtr* obj);

  // @brief transform tracked object with given pose
  // @params[IN] obj: tracked object for transfromation
  // @params[IN] pose: pose using for coordinate transformation
  // @return nothing
  void TransformTrackedObject(TrackedObjectPtr* obj,
                              const Eigen::Matrix4d& pose);

  // @brief transform object with given pose
  // @params[IN] obj: object for transfromation
  // @params[IN] pose: pose using for coordinate transformation
  // @return nothing
  void TransformObject(ObjectPtr* obj,
                       const Eigen::Matrix4d& pose);

  // @brief decompose foreground background from detected objects pool
  // @params[IN] objects: detected objects waiting for decomposition
  // @params[OUT] fg_objects: foreground objects
  // @params[OUT] bg_objects: background objects
  void DecomposeForegroundBackgroundObjects(
    std::vector<TrackedObjectPtr>* objects,
    std::vector<TrackedObjectPtr>* fg_objects,
    std::vector<TrackedObjectPtr>* bg_objects);

  // @brief compute tracks' predict states
  // @params[OUT] tracks_predict: tracks' predict states
  // @params[IN] time_diff: time interval for predicting
  // @return nothing
  void ComputeTracksPredict(std::vector<Eigen::VectorXf>* tracks_predict,
                            const double time_diff);

  // @brief update assigned tracks
  // @params[IN] tracks_predict: tracks' predict states
  // @params[IN] new_objects: new objects for current updating
  // @params[IN] assignments: assignment pair of new objects & tracks
  // @params[IN] time_diff: time interval for updating
  // @return nothing
  void UpdateAssignedTracks(std::vector<Eigen::VectorXf>* tracks_predict,
                            std::vector<TrackedObjectPtr>* new_objects,
                            const std::vector<TrackObjectPair>& assignments,
                            const double time_diff);

  // @brief update tracks without matched objects
  // @params[IN] tracks_predict: tracks' predict states
  // @params[IN] unassigned_tracks: index of unassigned tracks
  // @params[IN] time_diff: time interval for updating
  // @return nothing
  void UpdateUnassignedTracks(
    const std::vector<Eigen::VectorXf>& tracks_predict,
    const std::vector<int>& unassigned_tracks,
    const double time_diff);

  // @brief create new tracks for objects without matched tracks
  // @params[IN] new_objects: new objects for current updating
  // @params[IN] unassigned_objects: index of unassigned objects
  // @params[IN] time_diff: time interval for updating
  // @return nothing
  void CreateNewTracks(const std::vector<TrackedObjectPtr>& new_objects,
                       const std::vector<int>& unassigned_objects,
                       const double time_diff);

  // @brief delete lost tracks
  // @return nothing
  void DeleteLostTracks();

  // @brief collect tracked results for reporting
  // @params[OUT] tracked_objects: tracked objects include objects may be
  // occluded temporaryly
  // @return nothing
  void CollectTrackedResults(std::vector<ObjectPtr>* tracked_objects);

  // @brief compute track ids for rencet objects
  // @params[OUT] tracked_objects: tracked objects include objects may be
  // occluded temporaryly
  // @return nothing
  void ComputeTrackIdsForRecentObjects(
    const std::vector<TrackedObjectPtr>& objects);

 private:
  // algorithm setup
  MatcherType                                 matcher_method_;
  FilterType                                  filter_method_;
  bool                                        use_histogram_for_match_;
  int                                         histogram_bin_size_;

  // matcher
  BaseMatcher*                                matcher_;

  // tracks
  ObjectTrackSet                              object_tracks_;
  std::vector<int>                            track_ids_for_recent_objects_;

  // add background objects which are not tracked and are per-frame updated
  std::vector<TrackedObjectPtr>               background_objects_;

  // track data is stored in a local coordinate system, which is offset of
  // the global coordinate system by _global_to_local_offset
  Eigen::Matrix4d                             velodyne_to_local_pose_;
  Eigen::Vector3d                             global_to_local_offset_;
  double                                      time_stamp_;
  bool                                        valid_;
  // local coordinate system
  Eigen::Vector3f                             ref_location_;
  Eigen::Vector3f                             ref_orientation_;
  Eigen::Vector3f                             ref_translation_;

  DISALLOW_COPY_AND_ASSIGN(HmObjectTracker);
};  // class HmObjectTracker

REGISTER_TRACKER(HmObjectTracker);

}  // namespace perception
}  // namespace apollo

#endif  // MODULES_PERCEPTION_OBSTACLE_LIDAR_TRACKER_HM_TRACKER_H_
