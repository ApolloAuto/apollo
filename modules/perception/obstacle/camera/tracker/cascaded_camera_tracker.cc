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

#include "modules/perception/obstacle/camera/tracker/cascaded_camera_tracker.h"

namespace apollo {
namespace perception {

bool CascadedCameraTracker::Init() {
  bool init_flag = true;

  init_flag &= cs2d_tracker_.Init();
  if (dl_feature_) init_flag &= dlf_tracker_.Init();
  init_flag &= kcf_tracker_.Init();

  return init_flag;
}

bool CascadedCameraTracker::Associate(
    const cv::Mat& img, const double& timestamp,
    std::vector<std::shared_ptr<VisualObject>>* objects) {
  if (!objects) return false;
  frame_idx_++;

  float scale = 1.0f;
  std::vector<Detected> detected;
  GetDetectedFromVO(img.size(), scale, *objects, &detected);

  // Affinity matrix
  std::vector<std::vector<float>> affinity_matrix;
  affinity_matrix = std::vector<std::vector<float>>(
      tracks_.size(), std::vector<float>(detected.size(), 1.0f));

  // cs2d
  std::vector<std::vector<float>> cs2d_affinity_matrix;
  cs2d_tracker_.SelectFull(tracks_.size(), detected.size());
  cs2d_tracker_.GetAffinityMatrix(img, tracks_, detected,
                                  &cs2d_affinity_matrix);
  MergeAffinityMatrix(cs2d_affinity_matrix, &affinity_matrix);

  // dlf
  if (dl_feature_) {
    std::vector<std::vector<float>> dlf_affinity_matrix;
    dlf_tracker_.SelectFull(tracks_.size(), detected.size());
    dlf_tracker_.GetAffinityMatrix(img, tracks_, detected,
                                   &dlf_affinity_matrix);

    // Merge
    MergeAffinityMatrix(dlf_affinity_matrix, &affinity_matrix);

    // High confidence selection filtering. Thresholds are tuned in 3 passes
    // For entry higher than threshold, see if there is other competing
    // high scores in the cross (same row or column)
    FilterAffinityMatrix(0.95f, 0.95f, &affinity_matrix);
    FilterAffinityMatrix(0.7f, 0.3f, &affinity_matrix);
    FilterAffinityMatrix(0.5f, 0.2f, &affinity_matrix);
  }

  // kcf
  if (use_kcf_) {
    std::vector<std::vector<float>> kcf_affinity_matrix;
    kcf_tracker_.SelectEntries(affinity_matrix);
    kcf_tracker_.GetAffinityMatrix(img, tracks_, detected,
                                   &kcf_affinity_matrix);
    MergeAffinityMatrix(kcf_affinity_matrix, &affinity_matrix);
  }

  // Matching
  std::unordered_map<int, int> local_matching;
  std::unordered_set<int> local_matched_detected;
  MatrixMatching(affinity_matrix, &local_matching, &local_matched_detected);

  // Tracker and ID: detect id to (track id, first_timestamp) mapping
  std::unordered_map<int, std::pair<int, double>> id_mapping;
  ManageTrackerAndID(local_matching, local_matched_detected, detected,
                     frame_idx_, timestamp, &tracks_, &next_track_id_,
                     &id_mapping);

  // Update information used in tracks for the next frame
  cs2d_tracker_.UpdateTracked(img, detected, &tracks_);
  if (dl_feature_) dlf_tracker_.UpdateTracked(img, detected, &tracks_);
  if (use_kcf_) kcf_tracker_.UpdateTracked(img, detected, &tracks_);

  for (auto obj_ptr : *objects) {
    obj_ptr->last_track_timestamp = timestamp;

    if (id_mapping.find(obj_ptr->id) != id_mapping.end()) {
      obj_ptr->track_id = id_mapping[obj_ptr->id].first;
      obj_ptr->track_age = timestamp - id_mapping[obj_ptr->id].second;
    } else {  // Should not happen
      AWARN << "Det: " << obj_ptr->id << " has no tracking ID";
      obj_ptr->track_id = -1;
    }
  }

  return true;
}

std::string CascadedCameraTracker::Name() const {
  return "CascadedCameraTracker";
}

}  // namespace perception
}  // namespace apollo
