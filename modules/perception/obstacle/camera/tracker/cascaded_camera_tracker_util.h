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

#ifndef MODULES_PERCEPTION_OBSTACLE_CAMERA_TRACKER_CASCADED_UTIL_H_
#define MODULES_PERCEPTION_OBSTACLE_CAMERA_TRACKER_CASCADED_UTIL_H_

#include <iomanip>
#include <iostream>
#include <map>
#include <memory>
#include <string>
#include <unordered_map>
#include <unordered_set>
#include <utility>
#include <vector>

#include "modules/perception/obstacle/camera/common/util.h"
#include "modules/perception/obstacle/camera/common/visual_object.h"
#include "modules/perception/obstacle/camera/tracker/base_affinity_tracker.h"
#include "modules/perception/obstacle/common/hungarian_bigraph_matcher.h"

namespace apollo {
namespace perception {

void GetDetectedFromVO(
    const cv::Size &sz, const float &scale,
    const std::vector<std::shared_ptr<VisualObject>> &objects,
    std::vector<Detected> *detected);

void MergeAffinityMatrix(const std::vector<std::vector<float>> &to_merge,
                         std::vector<std::vector<float>> *affinity_matrix);

void FilterAffinityMatrix(float high_threshold, float other_threshold,
                          std::vector<std::vector<float>> *affinity_matrix);

void MatrixMatching(const std::vector<std::vector<float>> &affinity_matrix,
                    std::unordered_map<int, int> *local_matching,
                    std::unordered_set<int> *local_matched_detected);

// @brief Create, update and delete tracks, from the matching result
// ID mapping is done here as well
void ManageTrackerAndID(
    const std::unordered_map<int, int> &local_matching,
    const std::unordered_set<int> &local_matched_detected,
    const std::vector<Detected> &detected, const int &frame_idx,
    const double &timestamp, std::vector<Tracked> *tracked,
    int *next_tracked_id,
    std::unordered_map<int, std::pair<int, double>> *id_mapping);

void PrintAffinityMatrix(const std::vector<std::vector<float>> &affinity_matrix,
                         const std::vector<Tracked> &tracked,
                         const std::vector<Detected> &detected);

cv::Rect EnlargeBox(const cv::Size &img_size, const float &scale,
                    const cv::Rect &box);

}  // namespace perception
}  // namespace apollo

#endif  // MODULES_PERCEPTION_OBSTACLE_CAMERA_TRACKER_CASCADED_UTIL_H_
