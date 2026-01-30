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

#include "modules/common_msgs/perception_msgs/perception_obstacle.pb.h"

#include "modules/perception/common/onboard/msg_serializer/msg_serializer.h"
#include "modules/perception/lidar_tracking/interface/base_multi_target_tracker.h"
#include "modules/perception/lidar_tracking/tracker/multi_lidar_fusion/mlf_track_object_matcher.h"
#include "modules/perception/lidar_tracking/tracker/multi_lidar_fusion/mlf_tracker.h"

namespace apollo {
namespace perception {
namespace lidar {

class MlfEngine : public BaseMultiTargetTracker {
 public:
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW

 public:
  MlfEngine() = default;
  virtual ~MlfEngine() = default;
  /**
   * @brief Init mlf engine
   *
   * @param options
   * @return true
   * @return false
   */
  bool Init(const MultiTargetTrackerInitOptions& options =
                MultiTargetTrackerInitOptions()) override;

  /**
   * @brief Track segmented objects from multiple lidar sensors
   *
   * @param options tracker options
   * @param frame lidar frame
   * @return true
   * @return false
   */
  bool Track(const MultiTargetTrackerOptions& options,
             LidarFrame* frame) override;
  /**
   * @brief Get class name
   *
   * @return std::string
   */
  std::string Name() const override { return "MlfEngine"; };

 protected:
  /**
   * @brief Split foreground/background objects and attach to tracked objects
   *
   * @param objects
   * @param sensor_info
   */
  void SplitAndTransformToTrackedObjects(
      const std::vector<base::ObjectPtr>& objects,
      const base::SensorInfo& sensor_info, double frame_timestamp);

  /**
   * @brief Match tracks and objets and object-track assignment
   *
   * @param match_options match options
   * @param objects for match
   * @param name
   * @param tracks tracks for match and assignment
   */
  void TrackObjectMatchAndAssign(
      const MlfTrackObjectMatcherOptions& match_options,
      const std::vector<TrackedObjectPtr>& objects, const std::string& name,
      std::vector<MlfTrackDataPtr>* tracks);

  /**
   * @brief Filter tracks
   *
   * @param tracks for filter
   * @param frame_timestamp
   */
  void TrackStateFilter(const std::vector<MlfTrackDataPtr>& tracks,
                        double frame_timestamp);

  /**
   * @brief Collect track results and store in frame tracked objects
   *
   * @param frame lidar frame
   */
  void CollectTrackedResult(LidarFrame* frame);

  /**
   * @brief Remove stale track data for memory management
   *
   * @param name
   * @param timestamp
   * @param tracks to be cleaned
   */
  void RemoveStaleTrackData(const std::string& name, double timestamp,
                            std::vector<MlfTrackDataPtr>* tracks);

  /**
   * @brief Output object-detection-model results and latest track results
   * 
   * @param frame lidar frame
   * @param log model or cluster
   */
  void ObjectsDebugInfo(LidarFrame* frame, bool foreground_log = true);
  void TrackDebugInfo(LidarFrame* frame);

  /**
   * @brief Clear all data
   *
   */
  void Clear();
  //  void AttachDebugInfo(
  //      std::vector<std::shared_ptr<base::Object>>* foreground_objs);

  //  void AttachSemanticPredictedTrajectory(
  //      const std::vector<MlfTrackDataPtr>& tracks);

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

  // params
  bool use_histogram_for_match_ = true;
  size_t histogram_bin_size_ = 10;
  bool output_predict_objects_ = false;
  double reserved_invisible_time_ = 0.3;
  bool use_frame_timestamp_ = false;
  bool use_semantic_map_ = false;
  bool print_debug_log_ = false;
  double delay_output_ = 0.3;
  size_t pub_track_times_ = 1;
  bool merge_foreground_background_ = false;
};

}  // namespace lidar
}  // namespace perception
}  // namespace apollo
