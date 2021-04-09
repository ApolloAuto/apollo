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

#include "modules/perception/obstacle/camera/tracker/kcf/kcf_affinity_tracker.h"

namespace apollo {
namespace perception {

bool KCFAffinityTracker::Init() {
  detected_features_.clear();
  return kcf_component_.Init();
}

bool KCFAffinityTracker::GetAffinityMatrix(
    const cv::Mat &img, const std::vector<Tracked> &tracked,
    const std::vector<Detected> &detected,
    std::vector<std::vector<float>> *affinity_matrix) {
  affinity_matrix->clear();

  // Return if empty
  if (tracked.empty() || detected.empty()) return true;

  // Construct output. Default as 0.0 for not selected entries
  *affinity_matrix = std::vector<std::vector<float>>(
      tracked.size(), std::vector<float>(detected.size(), 0.0f));

  // Get features for detected boxes when needed
  detected_features_.clear();

  // Update score with KCF response
  for (size_t i = 0; i < selected_entry_matrix_.size(); ++i) {
    for (size_t j = 0; j < selected_entry_matrix_[0].size(); ++j) {
      if (selected_entry_matrix_[i][j]) {
        // Get detected box KCF stuff
        if (!detected_features_.count(j)) {
          // Enlarge detected search window
          cv::Rect box = detected[j].box_;
          box = EnlargeBox(img.size(), kScale_, box);

          std::vector<cv::Mat> z_f;
          kcf_component_.GetFeatures(img, box, &z_f);
          detected_features_[j] = z_f;
        }

        float score = 0.0f;
        kcf_component_.Detect(tracked[i], detected_features_[j], &score);

        // Keep threshold for KCF max response
        if (score > kKeepThreshold_) (*affinity_matrix)[i][j] = score;
      }
    }
  }

  return true;
}

bool KCFAffinityTracker::UpdateTracked(const cv::Mat &img,
                                       const std::vector<Detected> &detected,
                                       std::vector<Tracked> *tracked) {
  // Get x_f features and alpha_f for tracked objects
  for (auto &tracked_obj : *tracked) {
    // Reuse detected features if they match
    int det_id = tracked_obj.detect_id_;

    if (det_id >= 0 && detected_features_.count(det_id)) {
      tracked_obj.x_f_ = detected_features_[det_id];

      // Get alpha_f
      kcf_component_.Train(img, &tracked_obj);

    } else if (!tracked_obj.kcf_set_) {
      // Enlarge detected search window
      cv::Rect box = tracked_obj.box_;
      box = EnlargeBox(img.size(), kScale_, box);

      std::vector<cv::Mat> x_f;
      kcf_component_.GetFeatures(img, box, &x_f);
      tracked_obj.x_f_ = x_f;

      // Get alpha_f
      kcf_component_.Train(img, &tracked_obj);
    }

    tracked_obj.kcf_set_ = true;
  }

  return true;
}

}  // namespace perception
}  // namespace apollo
