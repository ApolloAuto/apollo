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

#ifndef MODULES_PERCEPTION_OBSTACLE_CAMERA_TRACKER_BASE_AFFINITY_TRACKER_H_
#define MODULES_PERCEPTION_OBSTACLE_CAMERA_TRACKER_BASE_AFFINITY_TRACKER_H_

#include <opencv2/opencv.hpp>
#include <limits>
#include <vector>

#include "Eigen/Core"

namespace apollo {
namespace perception {

class Tracked {
 public:
  cv::Rect box_;   // 2D bounding box
  int track_id_;   // unique tracking id
  int detect_id_;  // -1 means unmatched but kept
  double first_timestamp_;
  double last_timestamp_;
  int last_frame_idx_;

  // DLF: Deep Learning ROI Pooling features from detection
  std::vector<float> features_;

  // 3D position observed in camera space
  Eigen::Vector3f center_ = Eigen::Vector3f::Zero();

  // KCF
  bool kcf_set_ = false;
  std::vector<cv::Mat> x_f_;
  cv::Mat alpha_f_;
};

class Detected {
 public:
  cv::Rect box_;
  int detect_id_;

  // DLF: Deep Learning ROI Pooling features from detection
  std::vector<float> features_;

  // 3D position observed in camera space
  Eigen::Vector3f center_ = Eigen::Vector3f::Zero();
};

class BaseAffinityTracker {
 public:
  BaseAffinityTracker() {}

  virtual ~BaseAffinityTracker() {}

  virtual bool Init() = 0;

  // @brief Choose some entries in the matrix to do computation for affinity
  //
  // Skip the entry with
  // 1. 0 or negative affinity score
  // 2. High score (>= 9.0)
  //
  // Input must be regular and of the size in the following calls
  // (There must be the same number of entries in each row of the matrix)
  //
  virtual bool SelectEntries(
      const std::vector<std::vector<float>> &prev_matrix) {
    if (prev_matrix.empty()) return true;

    int row = prev_matrix.size();
    if (!row) return true;
    int col = prev_matrix[0].size();

    selected_entry_matrix_.clear();
    selected_entry_matrix_ =
        std::vector<std::vector<bool>>(row, std::vector<bool>(col, false));
    for (int i = 0; i < row; ++i) {
      for (int j = 0; j < col; ++j) {
        if (9.0f > prev_matrix[i][j] &&
            prev_matrix[i][j] > std::numeric_limits<float>::epsilon()) {
          selected_entry_matrix_[i][j] = true;
        }
      }
    }

    return true;
  }

  // @brief Set the selection matrix to full entries
  virtual bool SelectFull(const int &row, const int &col) {
    selected_entry_matrix_.clear();
    selected_entry_matrix_ =
        std::vector<std::vector<bool>>(row, std::vector<bool>(col, true));
    return true;
  }

  // @brief Get affinity_matrix between tracked objs and detected objs
  // rows: tracked objects, cols: detected objects
  // affinity_matrix[i][j]: score for the affinity between i_th tracked obj and
  // j_th detected obj
  virtual bool GetAffinityMatrix(
      const cv::Mat &img, const std::vector<Tracked> &tracked,
      const std::vector<Detected> &detected,
      std::vector<std::vector<float>> *affinity_matrix) = 0;

  // @brief Update information used in tracked objects, for the next frame
  virtual bool UpdateTracked(const cv::Mat &img,
                             const std::vector<Detected> &detected,
                             std::vector<Tracked> *tracked) = 0;

 protected:
  // @brief Matrix for selecting which entries need calculation
  std::vector<std::vector<bool>> selected_entry_matrix_;
};

}  // namespace perception
}  // namespace apollo

#endif  // MODULES_PERCEPTION_OBSTACLE_CAMERA_TRACKER_BASE_AFFINITY_TRACKER_H_
