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

#include <memory>
#include <string>
#include <utility>
#include <vector>

#include "modules/common/macro.h"
#include "modules/perception/obstacle/base/object.h"
#include "modules/perception/obstacle/lidar/interface/base_tracker.h"
#include "modules/perception/obstacle/lidar/tracker/hm_tracker/base_matcher.h"
#include "modules/perception/obstacle/lidar/tracker/hm_tracker/object_track.h"
#include "modules/perception/obstacle/lidar/tracker/hm_tracker/tracked_object.h"
#include "modules/perception/obstacle/onboard/hdmap_input.h"

namespace apollo {
namespace perception {

class HmObjectTracker : public BaseTracker {
 public:
  HmObjectTracker();
  virtual ~HmObjectTracker() = default;

  // @brief initialize tracker's configs
  // @return true if initialize successfully, otherwise return false
  bool Init();

  // @brief set matcher method
  // @params[IN] matcher_method_name: name of mathcer method
  // @return true if set successfully, otherwise return fasle
  bool SetMatcherMethod(const std::string& matcher_method_name);

  // @brief set collect consecutive invisible maximum
  // @params[IN] collect_consecutive_invisible_maximum: collect consecutive
  // invisible maximum
  // @return true if set successfully, otherwise return fasle
  bool SetCollectConsecutiveInvisibleMaximum(
      const int& collect_consecutive_invisible_maximum);

  // @brief set collect age minimum
  // @params[IN] collect_age_minimum: collect age minimum
  // @return true if set successfully, otherwise return fasle
  bool SetCollectAgeMinimum(const int& collect_age_minimum);

  // @brief set histogram bin size
  // @params[IN] histogram_bin_size: histogram bin size
  // @return true if set successfully, otherwise return fasle
  bool SetHistogramBinSize(const int& histogram_bin_size);

  // @brief track detected objects over consecutive frames
  // @params[IN] objects: recently detected objects
  // @params[IN] timestamp: timestamp of recently detected objects
  // @params[IN] options: tracker options with necessary information
  // @params[OUT] tracked_objects: tracked objects with tracking information
  // @return true if track successfully, otherwise return false
  bool Track(const std::vector<std::shared_ptr<Object>>& objects,
             double timestamp, const TrackerOptions& options,
             std::vector<std::shared_ptr<Object>>* tracked_objects);

  // @brief get object tracks of tracker
  // @return object tracks maintained in tracker
  const std::vector<ObjectTrackPtr>& GetObjectTracks() const;

  std::string name() const { return "HmObjectTracker"; }

 protected:
  // @brief initialize tracker after obtaining detection of first frame
  // @params[IN] objects: recently detected objects
  // @params[IN] timestamp: timestamp of recently detected objects
  // @params[IN] options: tracker options with necessary information
  // @params[OUT] tracked_objects: tracked objects with tracking information
  // @return true if initialize successfully, otherwise return false
  bool Initialize(const std::vector<std::shared_ptr<Object>>& objects,
                  const double& timestamp, const TrackerOptions& options,
                  std::vector<std::shared_ptr<Object>>* tracked_objects);

  // @brief transform v2world pose to v2local pose intend to avoid huge value
  // float computing
  // @params[OUT] pose: v2world pose
  // @return nothing
  void TransformPoseGlobal2Local(Eigen::Matrix4d* pose);

  // @brief construct tracked objects via necessray transformation & feature
  // computing
  // @params[IN] objects: objects for construction
  // @params[OUT] tracked_objects: constructed objects
  // @params[IN] pose: pose using for coordinate transformation
  // @params[IN] options: tracker options with necessary information
  // @return nothing
  void ConstructTrackedObjects(
      const std::vector<std::shared_ptr<Object>>& objects,
      std::vector<std::shared_ptr<TrackedObject>>* tracked_objects,
      const Eigen::Matrix4d& pose, const TrackerOptions& options);

  // @brief compute objects' shape feature
  // @params[OUT] object: object for computing shape feature
  // @return nothing
  void ComputeShapeFeatures(std::shared_ptr<TrackedObject>* obj);

  // @brief transform tracked object with given pose
  // @params[OUT] obj: tracked object for transfromation
  // @params[IN] pose: pose using for coordinate transformation
  // @return nothing
  void TransformTrackedObject(std::shared_ptr<TrackedObject>* obj,
                              const Eigen::Matrix4d& pose);

  // @brief transform object with given pose
  // @params[OUT] obj: object for transfromation
  // @params[IN] pose: pose using for coordinate transformation
  // @return nothing
  void TransformObject(std::shared_ptr<Object>* obj,
                       const Eigen::Matrix4d& pose);

  // @brief compute predicted states of maintained tracks
  // @params[OUT] tracks_predict: predicted states of maintained tracks
  // @params[IN] time_diff: time interval for predicting
  // @return nothing
  void ComputeTracksPredict(std::vector<Eigen::VectorXf>* tracks_predict,
                            const double& time_diff);

  // @brief update assigned tracks
  // @params[IN] tracks_predict: predicted states of maintained tracks
  // @params[IN] new_objects: recently detected objects
  // @params[IN] assignments: assignment pair of <track, object>
  // @params[IN] time_diff: time interval for updating
  // @return nothing
  void UpdateAssignedTracks(
      std::vector<Eigen::VectorXf>* tracks_predict,
      std::vector<std::shared_ptr<TrackedObject>>* new_objects,
      const std::vector<std::pair<int, int>>& assignments,
      const double& time_diff);

  // @brief update tracks without matched objects
  // @params[IN] tracks_predict: predicted states of maintained tracks
  // @params[IN] unassigned_tracks: index of unassigned tracks
  // @params[IN] time_diff: time interval for updating
  // @return nothing
  void UpdateUnassignedTracks(
      const std::vector<Eigen::VectorXf>& tracks_predict,
      const std::vector<int>& unassigned_tracks, const double& time_diff);

  // @brief create new tracks for objects without matched track
  // @params[IN] new_objects: recently detected objects
  // @params[IN] unassigned_objects: index of unassigned objects
  // @return nothing
  void CreateNewTracks(
      const std::vector<std::shared_ptr<TrackedObject>>& new_objects,
      const std::vector<int>& unassigned_objects);

  // @brief delete lost tracks
  // @return nothing
  void DeleteLostTracks();

  // @brief collect tracked results
  // @params[OUT] tracked_objects: tracked objects with tracking information
  // @return nothing
  void CollectTrackedResults(
      std::vector<std::shared_ptr<Object>>* tracked_objects);

 private:
  // algorithm setup
  MatcherType matcher_method_;
  FilterType filter_method_;
  int collect_consecutive_invisible_maximum_ = 0;
  int collect_age_minimum_ = 0;
  bool use_histogram_for_match_ = false;
  int histogram_bin_size_ = 10;

  // matcher
  std::unique_ptr<BaseMatcher> matcher_;

  // tracks
  ObjectTrackSet object_tracks_;

  // set offset to avoid huge value float computing
  Eigen::Vector3d global_to_local_offset_;
  double time_stamp_ = 0.0;
  bool valid_ = false;

  DISALLOW_COPY_AND_ASSIGN(HmObjectTracker);
};  // class HmObjectTracker

REGISTER_TRACKER(HmObjectTracker);

}  // namespace perception
}  // namespace apollo

#endif  // MODULES_PERCEPTION_OBSTACLE_LIDAR_TRACKER_HM_TRACKER_H_
