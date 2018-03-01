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

#include "modules/perception/obstacle/camera/tracker/cs2d/cs2d_affinity_tracker.h"

namespace apollo {
namespace perception {

bool CS2DAffinityTracker::Init() {
  return true;
}

bool CS2DAffinityTracker::GetAffinityMatrix(
    const cv::Mat &img, const std::vector<Tracked> &tracked,
    const std::vector<Detected> &detected,
    std::vector<std::vector<float>> *affinity_matrix) {
  affinity_matrix->clear();

  // Return if empty
  if (tracked.empty() || detected.empty()) {
    return true;
  }

  // Construct output. Default as 0.0 for not selected entries
  affinity_matrix = new std::vector<std::vector<float>>(
      tracked.size(), std::vector<float>(detected.size(), 0.0f));

  for (size_t i = 0; i < selected_entry_matrix_.size(); ++i) {
    cv::Rect box = tracked[i].box_;
    float w = static_cast<float>(box.width);
    float h = static_cast<float>(box.height);
    float c_x = static_cast<float>(box.x) + w / 2;
    float c_y = static_cast<float>(box.y) + h / 2;

    // 2D center position change limits
    float x_min = c_x - pos_range_ * w;
    float x_max = c_x + pos_range_ * w;
    float y_min = c_y - pos_range_ * h;
    float y_max = c_y + pos_range_ * h;

    for (size_t j = 0; j < selected_entry_matrix_[0].size(); ++j) {
      bool related = true;

      cv::Rect box_d = detected[j].box_;
      float w_d = static_cast<float>(box_d.width);
      float h_d = static_cast<float>(box_d.height);
      float c_x_d = static_cast<float>(box_d.x) + w_d / 2;
      float c_y_d = static_cast<float>(box_d.y) + h_d / 2;

      // 2D size change limits
      float ratio_w = w_d / w;
      if (ratio_w > (1.0f + sz_lim_) || ratio_w < (1.0f - sz_lim_)) {
        related = false;
      }
      float ratio_h = h_d / h;
      if (ratio_h > (1.0f + sz_lim_) || ratio_h < (1.0f - sz_lim_)) {
        related = false;
      }

      // 2D center position change limits
      if (c_x_d > x_max || c_x_d < x_min) {
        related = false;
      }
      if (c_y_d > y_max || c_y_d < y_min) {
        related = false;
      }

      if (related) {
        (*affinity_matrix)[i][j] = 1.0f;
      }
    }
  }

  return true;
}
// TODO(unknown): dummy implementation for pure virtual method in base class
bool CS2DAffinityTracker::UpdateTracked(const cv::Mat &img,
                                         const std::vector<Detected> &detected,
                                         std::vector<Tracked> *tracked) {
  return true;
}

}  // namespace perception
}  // namespace apollo
