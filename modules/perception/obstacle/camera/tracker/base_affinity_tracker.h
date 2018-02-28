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

#ifndef MODULES_PERCEPTION_OBSTACLE_CAMERA_TRACKER_BASE_AFFINITY_TRACKER_H
#define MODULES_PERCEPTION_OBSTACLE_CAMERA_TRACKER_BASE_AFFINITY_TRACKER_H

#include <opencv2/opencv.hpp>

#include <limits>
#include <vector>


namespace apollo {
namespace perception {

class Tracked {
 public:
  // Time
  double _last_seen_timestamp;
  int _last_seen_frame_cnt;

  // Basic information
  cv::Rect _box;
  int _track_id;   // unique tracking id
  int _detect_id;  // -1 means unmatched but kept

  // DLF
  std::vector<float> _features;

  // DAT
  cv::Mat _box_hist;
  cv::Mat _prob_look_up;
  float _self_dat_score;

  // KCF
  bool _kcf_set = false;
  std::vector<cv::Mat> _x_f;
  cv::Mat _alpha_f;

  // Debug Purpose: May be removed or nor used.
  cv::Mat _box_img;
};

class Detected {
 public:
  double _last_seen_timestamp;
  int _last_seen_frame_cnt;

  cv::Rect _box;
  size_t _detect_id;

  // DLF: Deep Learning ROI Pooling features from detection's layers
  std::vector<float> _features;
};

// Change the usage of Tracked and Detected to array of pointers after the
// size and contents become huge

class BaseAffinityTracker {
 public:
  BaseAffinityTracker() {}

  virtual ~BaseAffinityTracker() {}

  virtual bool init() = 0;

  // For high computation cost methods, the specific pair of affinity can be
  // chosen to do computation
  // The positive value entries in prev_affinity_matrix is chosen to get score
  // Input must be regular and of the size corresponding to the following call
  // (Ex: there must be the same number of entries in each row of the matrix)
  virtual bool set_selected_entries(
      const std::vector<std::vector<float>> &prev_matrix) {
    if (prev_matrix.empty()) {
      return true;
    }

    _selected_entry_matrix.clear();
    _selected_entry_matrix = std::vector<std::vector<bool>>(
        prev_matrix.size(), std::vector<bool>(prev_matrix[0].size(), false));
    for (size_t i = 0; i < prev_matrix.size(); ++i) {
      for (size_t j = 0; j < prev_matrix[0].size(); ++j) {
        if (prev_matrix[i][j] > std::numeric_limits<float>::epsilon() &&
            prev_matrix[i][j] < 9.0f) {
          _selected_entry_matrix[i][j] = true;
        }
      }
    }

    return true;
  }

  // Set the selection matrix to full entries
  virtual bool set_full_selection(const size_t &row, const size_t &col) {
    _selected_entry_matrix.clear();
    _selected_entry_matrix =
        std::vector<std::vector<bool>>(row, std::vector<bool>(col, true));
    return true;
  }

  // Get affinity_matrix between tracked objs and detected objs
  // row:tracked objects, col:detected objects
  // affinity_matrix[t_i][d_j]: score for the affinity of i_th tracked and j_th
  // detected
  virtual bool get_affinity_matrix(
      const cv::Mat &img, const std::vector<Tracked> &tracked,
      const std::vector<Detected> &detected,
      std::vector<std::vector<float>> *affinity_matrix) = 0;

  // Update features and information used in tracked objects for the next frame
  virtual bool update_tracked(const cv::Mat &img,
                              const std::vector<Detected> &detected,
                              std::vector<Tracked> *tracked) = 0;

 protected:
  // Matrix for selecting which entries need calculation
  std::vector<std::vector<bool>> _selected_entry_matrix;
};
}  // namespace perception
}  // namespace apollo

#endif  // MODULES_PERCEPTION_OBSTACLE_CAMERA_TRACKER_BASE_AFFINITY_TRACKER_H
