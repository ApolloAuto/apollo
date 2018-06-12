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

#include "modules/perception/obstacle/camera/tracker/dlf/dlf_affinity_tracker.h"

namespace apollo {
namespace perception {

bool DLFAffinityTracker::Init() {
  return true;
}

bool DLFAffinityTracker::GetAffinityMatrix(
    const cv::Mat &img, const std::vector<Tracked> &tracked,
    const std::vector<Detected> &detected,
    std::vector<std::vector<float>> *affinity_matrix) {
  affinity_matrix->clear();

  if (tracked.empty() || detected.empty()) return true;

  // Output. Default as 0.0 for not selected entries
  *affinity_matrix = std::vector<std::vector<float>>(
      tracked.size(), std::vector<float>(detected.size(), 0.0f));

  size_t dim = tracked[0].features_.size();
  for (size_t i = 0; i < selected_entry_matrix_.size(); ++i) {
    for (size_t j = 0; j < selected_entry_matrix_[0].size(); ++j) {
      float sum = 0.0f;
      for (size_t k = 0; k < dim; ++k) {
        sum += tracked[i].features_[k] * detected[j].features_[k];
      }

      // Filtering. Ad-hoc
      if (sum >= kConfThreshold_) {
        sum = 1.0f;
      } else if (sum <= kFilterThreshold_) {
        sum = 0.0f;
      }

      (*affinity_matrix)[i][j] = sum;
    }
  }

  return true;
}

bool DLFAffinityTracker::UpdateTracked(const cv::Mat &img,
                                       const std::vector<Detected> &detected,
                                       std::vector<Tracked> *tracked) {
  int d_cnt = detected.size();
  for (auto &obj : *tracked) {
    int d_id = obj.detect_id_;
    if (0 <= d_id && d_id < d_cnt) obj.features_ = detected[d_id].features_;
  }

  return true;
}

}  // namespace perception
}  // namespace apollo
