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
#ifndef PERCEPTION_LIDAR_LIB_TRACKER_HM_TRACKER_HM_MULTI_TARGET_TRACKER_H_
#define PERCEPTION_LIDAR_LIB_TRACKER_HM_TRACKER_HM_MULTI_TARGET_TRACKER_H_
#include <string>
#include <vector>

#include "modules/perception/base/object.h"
#include "modules/perception/common/geometry/convex_hull_2d.h"
#include "modules/perception/lidar/lib/interface/base_multi_target_tracker.h"
#include "modules/perception/lidar/lib/tracker/common/track_data.h"
#include "modules/perception/lidar/lib/tracker/hm_tracker/object_track_matcher.h"
#include "modules/perception/lidar/lib/tracker/hm_tracker/tracker.h"

namespace apollo {
namespace perception {
namespace lidar {

class HmMultiTargetTracker : public BaseMultiTargetTracker {
 public:
  typedef ObjectTrackMatcher::TrackObjectPair TrackObjectPair;

 public:
  HmMultiTargetTracker() = default;

  ~HmMultiTargetTracker() = default;

  bool Init(const MultiTargetTrackerInitOptions& options =
                MultiTargetTrackerInitOptions()) override;

  // @brief: track segmented objects, and estimate motion
  // @param [in]: options
  // @param [in/out]: tracked object
  // tracked objects should be filled, required,
  bool Track(const MultiTargetTrackerOptions& options,
             LidarFrame* frame) override;

  std::string Name() const override { return "HmMultiTargetTracker"; };

 protected:
  bool InitializeTracks(const MultiTargetTrackerOptions& options);

  bool UpdateTracks(const MultiTargetTrackerOptions& options);

  void CollectTrackedResult(LidarFrame* frame);

  void DecomposeForegroundBackgroundObjects(
      const std::vector<TrackedObjectPtr>& tracked_objects,
      std::vector<TrackedObjectPtr>* foreground_objects,
      std::vector<TrackedObjectPtr>* background_objects);

  void RemoveStaleTrackData();

  bool IsNeedToRemove(const TrackDataPtr& track);

  void MatchWrapper(const ObjectTrackMatcherOptions& options,
                    std::vector<TrackedObjectPtr>& objects,
                    const std::vector<TrackDataPtr>& tracks,
                    const std::vector<Eigen::VectorXf>& tracks_predict,
                    const double time_diff, ObjectTrackMatcher* matcher,
                    std::vector<TrackObjectPair>* assignments,
                    std::vector<size_t>* unassigned_tracks,
                    std::vector<size_t>* unassigned_objects);

  void CreateNewTracks(const std::vector<TrackedObjectPtr>& objects,
                       const std::vector<size_t>& unassigned_indices,
                       std::vector<TrackDataPtr>* track_datas);

  void ComputeTracksPredict(const std::vector<TrackDataPtr>& tracks,
                            std::vector<Eigen::VectorXf>* tracks_predict,
                            double timestamp);

  void ConstructTrackedObjects(const std::vector<base::ObjectPtr>& objects,
                               std::vector<TrackedObjectPtr>* tracked_objects,
                               const Eigen::Affine3d& pose,
                               const MultiTargetTrackerOptions& options);

  void UpdateAssignedAndUnassignedTracks(
      const std::vector<TrackDataPtr>& tracks,
      const std::vector<TrackedObjectPtr>& new_objects,
      const std::vector<TrackObjectPair>& assignments,
      const std::vector<size_t>& unassigned_tracks, const double timestamp);

 protected:
  // all the track data
  std::vector<TrackDataPtr> foreground_track_data_;
  std::vector<TrackDataPtr> background_track_data_;
  // transformed tracked objects
  std::vector<TrackedObjectPtr> foreground_objects_;
  std::vector<TrackedObjectPtr> background_objects_;
  // tracker
  std::unique_ptr<Tracker> tracker_;
  // object track matcher
  std::unique_ptr<ObjectTrackMatcher> foreground_matcher_;
  std::unique_ptr<ObjectTrackMatcher> background_matcher_;
  // match method
  std::string foreground_matcher_method_;
  std::string background_matcher_method_;
  std::string filter_method_;

  // add for use
  bool use_histogram_for_match_;
  int histogram_bin_size_;
  // frame information
  bool is_first_time_ = true;
  double current_timestamp_ = 0.0;
  double last_timestamp_ = 0.0;
  Eigen::Affine3d sensor_to_local_pose_;
  Eigen::Vector3d global_to_local_offset_;
  static int s_maximum_consecutive_invisible_count_;
  static float s_minimum_visible_ratio_;
  common::ConvexHull2D<base::PointDCloud, base::PolygonDType> hull_;
};  // class BaseMultiTargetTracker

}  // namespace lidar
}  // namespace perception
}  // namespace apollo

#endif
