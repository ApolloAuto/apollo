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

#include <string>
#include <vector>

#include "cyber/common/macros.h"
#include "modules/perception/common/algorithm/graph/gated_hungarian_bigraph_matcher.h"
#include "modules/perception/multi_sensor_fusion/fusion/data_association/hm_data_association/track_object_distance.h"
#include "modules/perception/multi_sensor_fusion/interface/base_data_association.h"

namespace apollo {
namespace perception {
namespace fusion {

class HMTrackersObjectsAssociation : public BaseDataAssociation {
 public:
  HMTrackersObjectsAssociation() = default;
  ~HMTrackersObjectsAssociation() = default;

  /**
   * @brief initialization
   *
   * @param options
   * @return true
   * @return false
   */
  bool Init(const AssociationInitOptions &options) override {
    track_object_distance_.set_distance_thresh(
        static_cast<float>(s_match_distance_thresh_));
    return true;
  }

  /**
   * @brief Associate the obstacles measured by the sensor with the obstacles
   * tracked in current scene
   *
   * @param options
   * @param sensor_measurements obstacles measured by the sensor
   * @param scene obstacles tracked in current scene
   * @param association_result association result
   * @return true
   * @return false
   */
  bool Associate(const AssociationOptions& options,
                 SensorFramePtr sensor_measurements, ScenePtr scene,
                 AssociationResult* association_result) override;

  std::string Name() const override { return "HMTrackersObjectsAssociation"; }

 private:
  /**
   * @brief Calculate the association distance matrix
   *
   * @param fusion_tracks
   * @param sensor_objects
   * @param ref_point
   * @param unassigned_tracks
   * @param unassigned_measurements
   * @param association_mat
   */
  void ComputeAssociationDistanceMat(
      const std::vector<TrackPtr>& fusion_tracks,
      const std::vector<SensorObjectPtr>& sensor_objects,
      const Eigen::Vector3d& ref_point,
      const std::vector<size_t>& unassigned_tracks,
      const std::vector<size_t>& unassigned_measurements,
      std::vector<std::vector<double>>* association_mat);

  /**
   * @brief
   *
   * @param fusion_tracks
   * @param sensor_objects
   * @param assignments
   * @param unassigned_fusion_tracks
   * @param unassigned_sensor_objects
   * @param do_nothing
   * @param post
   */
  void IdAssign(const std::vector<TrackPtr>& fusion_tracks,
                const std::vector<SensorObjectPtr>& sensor_objects,
                std::vector<TrackMeasurmentPair>* assignments,
                std::vector<size_t>* unassigned_fusion_tracks,
                std::vector<size_t>* unassigned_sensor_objects,
                bool do_nothing = false, bool post = false);

  /**
   * @brief
   *
   * @param fusion_tracks
   * @param sensor_objects
   * @param unassigned_fusion_tracks
   * @param unassigned_sensor_objects
   * @param post_assignments
   */
  void PostIdAssign(const std::vector<TrackPtr>& fusion_tracks,
                    const std::vector<SensorObjectPtr>& sensor_objects,
                    const std::vector<size_t>& unassigned_fusion_tracks,
                    const std::vector<size_t>& unassigned_sensor_objects,
                    std::vector<TrackMeasurmentPair>* post_assignments);

  /**
   * @brief
   *
   * @param association_mat
   * @param track_ind_l2g
   * @param measurement_ind_l2g
   * @param assignments
   * @param unassigned_tracks
   * @param unassigned_measurements
   * @return true
   * @return false
   */
  bool MinimizeAssignment(
      const std::vector<std::vector<double>>& association_mat,
      const std::vector<size_t>& track_ind_l2g,
      const std::vector<size_t>& measurement_ind_l2g,
      std::vector<TrackMeasurmentPair>* assignments,
      std::vector<size_t>* unassigned_tracks,
      std::vector<size_t>* unassigned_measurements);

  /**
   * @brief calculate distance
   *
   * @param fusion_tracks
   * @param sensor_objects
   * @param unassigned_fusion_track
   * @param track_ind_g2l
   * @param measurement_ind_g2l
   * @param measurement_ind_l2g
   * @param association_mat
   * @param association_result
   */
  void ComputeDistance(const std::vector<TrackPtr>& fusion_tracks,
                       const std::vector<SensorObjectPtr>& sensor_objects,
                       const std::vector<size_t>& unassigned_fusion_track,
                       const std::vector<int>& track_ind_g2l,
                       const std::vector<int>& measurement_ind_g2l,
                       const std::vector<size_t>& measurement_ind_l2g,
                       const std::vector<std::vector<double>>& association_mat,
                       AssociationResult* association_result);

  /**
   * @brief Generate unmatched data
   *
   * @param track_num
   * @param objects_num
   * @param assignments
   * @param unassigned_tracks
   * @param unassigned_objects
   */
  void GenerateUnassignedData(
      size_t track_num, size_t objects_num,
      const std::vector<TrackMeasurmentPair>& assignments,
      std::vector<size_t>* unassigned_tracks,
      std::vector<size_t>* unassigned_objects);

 private:
  /// @brief gated hungarian matcher
  algorithm::GatedHungarianMatcher<float> optimizer_;

  /// @brief TrackObjectDistance
  TrackObjectDistance track_object_distance_;
  /// @brief match distance thresh
  static double s_match_distance_thresh_;
  /// @brief match distance bound
  static double s_match_distance_bound_;
  /// @brief association center dist threshold
  static double s_association_center_dist_threshold_;

  DISALLOW_COPY_AND_ASSIGN(HMTrackersObjectsAssociation);
};

}  // namespace fusion
}  // namespace perception
}  // namespace apollo
