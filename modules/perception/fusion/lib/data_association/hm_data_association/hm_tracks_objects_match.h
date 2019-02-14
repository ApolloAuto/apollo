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

#include "modules/perception/common/graph/gated_hungarian_bigraph_matcher.h"
#include "modules/perception/fusion/lib/data_association/hm_data_association/track_object_distance.h"
#include "modules/perception/fusion/lib/interface/base_data_association.h"

namespace apollo {
namespace perception {
namespace fusion {

class HMTrackersObjectsAssociation : public BaseDataAssociation {
 public:
  HMTrackersObjectsAssociation() {}
  ~HMTrackersObjectsAssociation() {}
  HMTrackersObjectsAssociation(const HMTrackersObjectsAssociation&) = delete;
  HMTrackersObjectsAssociation& operator=(const HMTrackersObjectsAssociation&) =
      delete;

  bool Init() override {
    track_object_distance_.set_distance_thresh(
        static_cast<float>(s_match_distance_thresh_));
    return true;
  }

  bool Associate(const AssociationOptions& options,
                 SensorFramePtr sensor_measurements, ScenePtr scene,
                 AssociationResult* association_result) override;

  std::string Name() const override { return "HMTrackersObjectsAssociation"; }

 private:
  void ComputeAssociationDistanceMat(
      const std::vector<TrackPtr>& fusion_tracks,
      const std::vector<SensorObjectPtr>& sensor_objects,
      const Eigen::Vector3d& ref_point,
      const std::vector<size_t>& unassigned_tracks,
      const std::vector<size_t>& unassigned_measurements,
      std::vector<std::vector<double>>* association_mat);

  void IdAssign(const std::vector<TrackPtr>& fusion_tracks,
                const std::vector<SensorObjectPtr>& sensor_objects,
                std::vector<TrackMeasurmentPair>* assignments,
                std::vector<size_t>* unassigned_fusion_tracks,
                std::vector<size_t>* unassigned_sensor_objects,
                bool do_nothing = false, bool post = false);

  void PostIdAssign(const std::vector<TrackPtr>& fusion_tracks,
                    const std::vector<SensorObjectPtr>& sensor_objects,
                    const std::vector<size_t>& unassigned_fusion_tracks,
                    const std::vector<size_t>& unassigned_sensor_objects,
                    std::vector<TrackMeasurmentPair>* post_assignments);

  bool MinimizeAssignment(
      const std::vector<std::vector<double>>& association_mat,
      const std::vector<size_t>& track_ind_l2g,
      const std::vector<size_t>& measurement_ind_l2g,
      std::vector<TrackMeasurmentPair>* assignments,
      std::vector<size_t>* unassigned_tracks,
      std::vector<size_t>* unassigned_measurements);

  void ComputeDistance(const std::vector<TrackPtr>& fusion_tracks,
                       const std::vector<SensorObjectPtr>& sensor_objects,
                       const std::vector<size_t>& unassigned_fusion_track,
                       const std::vector<int>& track_ind_g2l,
                       const std::vector<int>& measurement_ind_g2l,
                       const std::vector<size_t>& measurement_ind_l2g,
                       const std::vector<std::vector<double>>& association_mat,
                       AssociationResult* association_result);

  void GenerateUnassignedData(
      size_t track_num, size_t objects_num,
      const std::vector<TrackMeasurmentPair>& assignments,
      std::vector<size_t>* unassigned_tracks,
      std::vector<size_t>* unassigned_objects);

 private:
  common::GatedHungarianMatcher<float> optimizer_;
  TrackObjectDistance track_object_distance_;
  static double s_match_distance_thresh_;
  static double s_match_distance_bound_;
  static double s_association_center_dist_threshold_;
};

}  // namespace fusion
}  // namespace perception
}  // namespace apollo
