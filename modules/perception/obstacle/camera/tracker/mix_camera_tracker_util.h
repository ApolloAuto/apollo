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

#ifndef MODULES_PERCEPTION_OBSTACLE_CAMERA_TRACKER_MIX_CAMERA_TRACKER_UTIL_H
#define MODULES_PERCEPTION_OBSTACLE_CAMERA_TRACKER_MIX_CAMERA_TRACKER_UTIL_H

#include <iomanip>
#include <iostream>
#include <map>
#include <string>
#include <unordered_map>
#include <unordered_set>
#include <vector>

#include "modules/perception/obstacle/camera/common/util.h"
#include "modules/perception/obstacle/camera/common/visual_object.h"
#include "modules/perception/obstacle/common/hungarian_bigraph_matcher.h"

#include "modules/perception/obstacle/camera/tracker/base_affinity_tracker.h"

namespace apollo {
namespace perception {

void get_detected_from_vo(const cv::Size &sz,
                          const std::vector<VisualObjectPtr> &objects,
                          const float &scale, std::vector<Detected> *detected);

void merge_affinity_matrix(const std::vector<std::vector<float>> &to_merge,
                           std::vector<std::vector<float>> *affinity_matrix);

void filter_affinity_matrix(float high_threshold, float other_threshold,
                            std::vector<std::vector<float>> *affinity_matrix);

void matrix_matching(const std::vector<std::vector<float>> &affinity_matrix,
                     std::unordered_map<int, int> *local_matching,
                     std::unordered_set<int> *local_matched_detected);

void tracker_and_id_management(
    const std::unordered_map<int, int> &local_matching,
    const std::unordered_set<int> &local_matched_detected,
    const std::vector<Detected> &detected, std::vector<Tracked> *tracked,
    int *next_tracked_id, std::map<int, int> *id_mapping, int curr_frame_cnt);

void print_affinity_matrix(
    const std::vector<std::vector<float>> &affinity_matrix,
    const std::vector<Tracked> &tracked, const std::vector<Detected> &detected);

}  // namespace perception
}  // namespace apollo

#endif  // MODULES_PERCEPTION_OBSTACLE_CAMERA_TRACKER_MIX_CAMERA_TRACKER_UTIL_H
