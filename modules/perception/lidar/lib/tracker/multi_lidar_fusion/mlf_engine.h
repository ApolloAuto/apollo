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
#pragma once

#include <memory>
#include <set>
#include <string>
#include <vector>

#include "modules/perception/lidar/lib/interface/base_multi_target_tracker.h"
#include "modules/perception/lidar/lib/tracker/multi_lidar_fusion/mlf_track_object_matcher.h"
#include "modules/perception/lidar/lib/tracker/multi_lidar_fusion/mlf_tracker.h"

namespace apollo {
namespace perception {
namespace lidar {

class MlfEngine : public BaseMultiTargetTracker {
 public:
  MlfEngine() = default;
  ~MlfEngine() = default;

  bool Init(const MultiTargetTrackerInitOptions& options =
                MultiTargetTrackerInitOptions()) override;

  // @brief: track segmented objects from multiple lidar sensors
  // @params [in]: tracker options
  // @params [in/out]: lidar frame
  bool Track(const MultiTargetTrackerOptions& options,
             LidarFrame* frame) override;

  std::string Name() const override { return "MlfEngine"; };

 protected:
  // @brief: split foreground/background objects and attach to tracked objects
  // @params [in]: objects
  // @params [in]: sensor info
  void SplitAndTransformToTrackedObjects(
      const std::vector<base::ObjectPtr>& objects,
      const base::SensorInfo& sensor_info);

  // @brief: match tracks and objects and object-track assignment
  // @params [in]: match options
  // @params [in]: objects for match
  // @params [in]: name
  // @params [in/out]: tracks for match and assignment
  void TrackObjectMatchAndAssign(
      const MlfTrackObjectMatcherOptions& match_options,
      const std::vector<TrackedObjectPtr>& objects, const std::string& name,
      std::vector<MlfTrackDataPtr>* tracks);

  // @brief: filter tracks
  // @params [in]: tracks for filter
  // @params [in]: frame timestamp
  void TrackStateFilter(const std::vector<MlfTrackDataPtr>& tracks,
                        double frame_timestamp);

  // @brief: collect track results and store in frame tracked objects
  // @params [in/out]: lidar frame
  void CollectTrackedResult(LidarFrame* frame);

  // @brief: remove stale track data for memory management
  // @params: name
  // @params: timestamp
  // @params [in/out]: tracks to be cleaned
  void RemoveStaleTrackData(const std::string& name, double timestamp,
                            std::vector<MlfTrackDataPtr>* tracks);

 protected:
  // foreground and background track data
  std::vector<MlfTrackDataPtr> foreground_track_data_;
  std::vector<MlfTrackDataPtr> background_track_data_;
  // foreground and background tracked objects
  std::vector<TrackedObjectPtr> foreground_objects_;
  std::vector<TrackedObjectPtr> background_objects_;
  // tracker
  std::unique_ptr<MlfTracker> tracker_;
  // track object matcher
  std::unique_ptr<MlfTrackObjectMatcher> matcher_;
  // offset maintained for numeric issues
  Eigen::Vector3d global_to_local_offset_;
  Eigen::Affine3d sensor_to_local_pose_;
  // main sensor info
  std::set<std::string> main_sensor_;
  // params
  bool use_histogram_for_match_ = true;
  size_t histogram_bin_size_ = 10;
  bool output_predict_objects_ = false;
  double reserved_invisible_time_ = 0.3;
  bool use_frame_timestamp_ = false;
};

}  // namespace lidar
}  // namespace perception
}  // namespace apollo
