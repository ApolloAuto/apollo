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

#include "kcf_affinity_tracker.h"

namespace adu {
namespace perception {
namespace obstacle {

bool KCFAffinityTracker::init() {
  _detected_features.clear();
  return _kcf_component.init();
}

bool KCFAffinityTracker::get_affinity_matrix(
    const cv::Mat &img, const std::vector<Tracked> &tracked,
    const std::vector<Detected> &detected,
    std::vector<std::vector<float>> &affinity_matrix) {
  affinity_matrix.clear();

  // Return if empty
  if (tracked.empty() || detected.empty()) {
    return true;
  }

  // Construct output. Default as 0.0 for not selected entries
  affinity_matrix = std::vector<std::vector<float>>(
      tracked.size(), std::vector<float>(detected.size(), 0.0f));

  // Get features for detected boxes when needed
  _detected_features.clear();

  // TODO HACK: copy for now
  std::vector<Tracked> tracked_copy = tracked;
  for (size_t i = 0; i < tracked.size(); ++i) {
    tracked_copy[i]._box = tracked[i]._box;
    tracked_copy[i]._track_id = tracked[i]._track_id;
    tracked_copy[i]._detect_id = tracked[i]._detect_id;

    tracked_copy[i]._kcf_set = tracked[i]._kcf_set;
    tracked_copy[i]._x_f = tracked[i]._x_f;
    tracked_copy[i]._alpha_f = tracked[i]._alpha_f;
  }

  // Update score with KCF response
  for (size_t i = 0; i < _selected_entry_matrix.size(); ++i) {
    for (size_t j = 0; j < _selected_entry_matrix[0].size(); ++j) {
      if (_selected_entry_matrix[i][j]) {
        // Get detected box KCF stuff
        if (_detected_features.count(j) == 0) {
          std::vector<cv::Mat> z_f;

          // Enlarge detected search window to 2.5 times
          cv::Rect box = detected[j]._box;
          float scale = 2.5f;
          enlarge_box(img.size(), scale, box);

          _kcf_component.get_feature(img, box, z_f);
          _detected_features[j] = z_f;
        }

        // // Get tracked box KCF stuff
        // if (!tracked_copy[i]._kcf_set) {
        //   // Enlarge detected search window to 2.5 times
        //   cv::Rect box = tracked_copy[i]._box;
        //   float scale = 2.5f;
        //   enlarge_box(_prev_img.size(), scale, box);
        //
        //   std::vector<cv::Mat> x_f;
        //   _kcf_component.get_feature(_prev_img, tracked_copy[i]._box, x_f);
        //   tracked_copy[i]._x_f = x_f;
        //
        //   // Get alpha_f
        //   _kcf_component.train(_prev_img, tracked_copy[i]);
        //
        //   tracked_copy[i]._kcf_set = true;
        // }

        float score = 0.0f;
        _kcf_component.detect(tracked_copy[i], _detected_features[j], score);

        // Keep threshold for KCF max response
        if (score > _keep_threshold) {
          affinity_matrix[i][j] = score;
        }
      }
    }
  }

  return true;
}

bool KCFAffinityTracker::update_tracked(const cv::Mat &img,
                                        const std::vector<Detected> &detected,
                                        std::vector<Tracked> &tracked) {
  _prev_img = img.clone();

  // Get x_f features and alpha_f for tracked objects
  for (auto &tracked_obj : tracked) {
    // Reuse detected features if they match
    int det_id = tracked_obj._detect_id;

    if (det_id >= 0 &&
        _detected_features.find(det_id) != _detected_features.end()) {
      tracked_obj._x_f = _detected_features[det_id];

      // Get alpha_f
      _kcf_component.train(img, tracked_obj);

      tracked_obj._kcf_set = true;
    } else if (!tracked_obj._kcf_set) {
      // Enlarge detected search window to 2.5 times
      cv::Rect box = tracked_obj._box;
      float scale = 2.5f;
      enlarge_box(img.size(), scale, box);

      std::vector<cv::Mat> x_f;
      _kcf_component.get_feature(img, box, x_f);
      tracked_obj._x_f = x_f;

      // Get alpha_f
      _kcf_component.train(img, tracked_obj);

      tracked_obj._kcf_set = true;
    }
  }

  return true;
}

}  // namespace adu
}  // namespace perception
}  // namespace obstacle
