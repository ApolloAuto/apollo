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

#include <algorithm>
#include <utility>

#include "modules/common/log.h"
#include "modules/perception/obstacle/camera/tracker/cascaded_camera_tracker_util.h"

namespace apollo {
namespace perception {

void GetDetectedFromVO(const cv::Size &sz,
                       const std::vector<VisualObjectPtr> &objects,
                       const float &scale, std::vector<Detected> *detected) {
  size_t i = 0;
  detected->clear();
  for (auto obj_ptr : objects) {
    // Get the boxes and make sure they are within the original image
    // double x1 = std::min(obj_ptr->upper_left.x(),
    // static_cast<double>(sz.width - 1));
    double x1 = std::min(static_cast<float>(obj_ptr->upper_left[0]),
                         static_cast<float>(sz.width - 1));
    x1 = std::max(x1, 0.0);
    // double y1 = std::min(obj_ptr->upper_left.y(),
    // static_cast<double>(sz.height - 1));
    double y1 = std::min(static_cast<float>(obj_ptr->upper_left[1]),
                         static_cast<float>(sz.height - 1));
    y1 = std::max(y1, 0.0);

    // double x2 = std::min(obj_ptr->lower_right.x(),
    // static_cast<double>(sz.width - 1));
    double x2 = std::min(static_cast<float>(obj_ptr->lower_right[0]),
                         static_cast<float>(sz.width - 1));
    x2 = std::max(x2, 0.0);
    // double y2 = std::min(obj_ptr->lower_right.y(),
    // static_cast<double>(sz.height - 1));
    double y2 = std::min(static_cast<float>(obj_ptr->lower_right[1]),
                         static_cast<float>(sz.height - 1));
    y2 = std::max(y2, 0.0);

    int x = static_cast<int>(x1 * scale);
    int y = static_cast<int>(y1 * scale);
    int width = static_cast<int>((x2 - x1) * scale);
    int height = static_cast<int>((y2 - y1) * scale);

    Detected obj;
    obj.detect_id_ = i;
    obj_ptr->id = static_cast<int>(i);
    obj.box_ = cv::Rect(x, y, width, height);
    obj.features_ = obj_ptr->dl_roi_feature;
    detected->push_back(obj);
    ++i;
  }
}

void MergeAffinityMatrix(const std::vector<std::vector<float>> &to_merge,
                         std::vector<std::vector<float>> *affinity_matrix) {
  if (to_merge.empty() || affinity_matrix->empty()) {
    return;
  } else if (to_merge.size() != affinity_matrix->size()) {
    return;
  } else if (to_merge[0].size() != (*affinity_matrix)[0].size()) {
    return;
  }

  for (size_t i = 0; i < affinity_matrix->size(); ++i) {
    for (size_t j = 0; j < (*affinity_matrix)[0].size(); ++j) {
      if ((*affinity_matrix)[i][j] <
          9.0f) {  // For saving high confidence selection
        (*affinity_matrix)[i][j] *= to_merge[i][j];
      }
    }
  }

  return;
}

void FilterAffinityMatrix(float high_threshold, float other_threshold,
                          std::vector<std::vector<float>> *affinity_matrix) {
  for (size_t i = 0; i < affinity_matrix->size(); ++i) {
    for (size_t j = 0; j < (*affinity_matrix)[0].size(); ++j) {
      if ((*affinity_matrix)[i][j] > high_threshold) {
        bool other_high_conf = false;

        // Same column
        for (size_t k = 0; k < affinity_matrix->size(); ++k) {
          if (k == i) continue;
          if ((*affinity_matrix)[k][j] > other_threshold)
            other_high_conf = true;
        }
        // Same row
        for (size_t k = 0; k < (*affinity_matrix)[0].size(); ++k) {
          if (k == j) continue;
          if ((*affinity_matrix)[i][k] > other_threshold)
            other_high_conf = true;
        }

        // The only one in cross
        if (!other_high_conf) {
          // High confidence selection
          (*affinity_matrix)[i][j] = 10.0;

          // Same column reduces to 0
          for (size_t k = 0; k < affinity_matrix->size(); ++k) {
            if (k == i) continue;
            (*affinity_matrix)[k][j] = 0.0f;
          }
          // Same row reduces to 0
          for (size_t k = 0; k < (*affinity_matrix)[0].size(); ++k) {
            if (k == j) continue;
            (*affinity_matrix)[i][k] = 0.0f;
          }
        }
      }
    }
  }

  return;
}

void MatrixMatching(const std::vector<std::vector<float>> &affinity_matrix,
                    std::unordered_map<int, int> *local_matching,
                    std::unordered_set<int> *local_matched_detected) {
  if (affinity_matrix.empty()) {
    return;
  }

  // Matching with one stage affinity matrix (Maximize the affinity)
  std::vector<std::vector<double>> affinity(
      affinity_matrix.size(),
      std::vector<double>(affinity_matrix[0].size(), 0.0));
  for (size_t i = 0; i < affinity_matrix.size(); ++i) {
    for (size_t j = 0; j < affinity_matrix[0].size(); ++j) {
      affinity[i][j] = static_cast<double>(affinity_matrix[i][j]);
    }
  }
  HungarianOptimizer optimizer = HungarianOptimizer(affinity);
  std::vector<int> agent;
  std::vector<int> task;
  optimizer.maximize(&agent, &task);

  // !! Cautious about corner cases when the entry in affinity matrix is 0
  // This Hungarian method implementation will still give matching for the
  // remaining with 0

  local_matching->clear();
  local_matched_detected->clear();
  for (size_t i = 0; i < agent.size(); ++i) {
    // Ignore 0-matching
    if (affinity_matrix[agent[i]][task[i]] > 0.0f) {
      (*local_matching)[agent[i]] = task[i];
      local_matched_detected->emplace(task[i]);
    }
  }
}

void ManageTrackerAndID(const std::unordered_map<int, int> &local_matching,
                        const std::unordered_set<int> &local_matched_detected,
                        const std::vector<Detected> &detected,
                        std::vector<Tracked> *tracked, int *next_tracked_id,
                        std::map<int, int> *id_mapping, int curr_frame_cnt) {
  // Output:
  // Create, update and delete tracks, with the given matching result. (Ad-hoc
  // strategy here of test)
  // ID mapping done here as well
  id_mapping->clear();
  std::vector<Tracked> new_tracked;
  const int max_kept_frame_cnt = 10;

  // Sort local matching output based on tracked_id for easier debugging
  std::map<int, std::pair<int, int>> tracked_id_local_index;
  for (const auto &pair : local_matching) {
    int track_id = (*tracked)[pair.first].track_id_;

    if (tracked_id_local_index.find(track_id) == tracked_id_local_index.end()) {
      tracked_id_local_index[track_id] =
          std::make_pair(pair.first, pair.second);
    } else {
      AWARN << "Should not contain duplicated tracked id";
    }
  }

  // Update tracked, which is sorted by track_id
  std::unordered_map<int, int> trackedID_to_detectedID;
  for (const auto &item : tracked_id_local_index) {
    int track_id =
        (*tracked)[item.second.first].track_id_;  // The same as item.first
    size_t detect_id = detected[item.second.second].detect_id_;

    Tracked curr_tracked;
    curr_tracked.last_frame_idx_ = curr_frame_cnt;
    curr_tracked.last_timestamp_ = detected[item.second.second].last_timestamp_;
    curr_tracked.track_id_ = track_id;
    curr_tracked.detect_id_ = detect_id;
    curr_tracked.box_ = detected[item.second.second].box_;
    new_tracked.emplace_back(curr_tracked);

    (*id_mapping)[static_cast<int>(detect_id)] = track_id;
    trackedID_to_detectedID[track_id] = static_cast<int>(detect_id);
  }

  // Create new tracked based on unmatched detected
  for (size_t i = 0; i < detected.size(); ++i) {
    if (local_matched_detected.find(i) == local_matched_detected.end()) {
      Tracked curr_tracked;
      curr_tracked.last_frame_idx_ = curr_frame_cnt;
      curr_tracked.last_timestamp_ = detected[i].last_timestamp_;
      curr_tracked.track_id_ = *next_tracked_id;
      curr_tracked.detect_id_ = detected[i].detect_id_;
      curr_tracked.box_ = detected[i].box_;
      new_tracked.emplace_back(curr_tracked);

      (*id_mapping)[static_cast<int>(detected[i].detect_id_)] =
          *next_tracked_id;
      trackedID_to_detectedID[*next_tracked_id] =
          static_cast<int>(detected[i].detect_id_);

      // ID management
      ++(*next_tracked_id);
    }
  }

  // Keep unmatched tracks here
  for (auto &trk : *tracked) {
    if (!trackedID_to_detectedID.count(trk.track_id_) &&
        trk.last_frame_idx_ + max_kept_frame_cnt >= curr_frame_cnt) {
      trk.detect_id_ = -1;
      new_tracked.emplace_back(trk);
    }
  }

  std::swap(new_tracked, *tracked);
}

void PrintAffinityMatrix(const std::vector<std::vector<float>> &affinity_matrix,
                         const std::vector<Tracked> &tracked,
                         const std::vector<Detected> &detected) {
  if (!affinity_matrix.empty()) {
    std::string to_print = "T/ detect ID:";
    for (size_t j = 0; j < affinity_matrix[0].size(); ++j) {
      to_print += " " + std::to_string(detected[j].detect_id_);
    }
    AINFO << to_print;

    for (size_t i = 0; i < affinity_matrix.size(); ++i) {
      std::string to_print_affinity = "";
      for (size_t j = 0; j < affinity_matrix[i].size(); ++j) {
        to_print_affinity += std::to_string(affinity_matrix[i][j]) + " ";
      }

      AINFO << i << "th T, track_id:" << tracked[i].track_id_
            << " predetect_id_:" << tracked[i].detect_id_ << "\t"
            << to_print_affinity;
    }
  } else {
    AINFO << "Empty affinity_matrix";
  }
}

}  // namespace perception
}  // namespace apollo
