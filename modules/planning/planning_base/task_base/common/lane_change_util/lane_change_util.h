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

/**
 * @file lane_change_util.h
 * @brief This file contains the static member functions from LaneChangeDecider,
 * which are deleted from the class.
 * It is a temporary scheme. The code will be reorganized for path function
 * cohesion later.
 **/

#include <limits>

namespace apollo {
namespace planning {

class ReferenceLineInfo;
class Frame;
class PlanningContext;

/**
 * @brief A static function to check if the ChangeLanePath type of reference
 * line is safe or if current reference line is safe to deviate away and come
 * back
 */
bool IsClearToChangeLane(ReferenceLineInfo* reference_line_info);

/**
 * @brief A static function to estimate if an obstacle in certain range in
 * front of ADV blocks too much space perception behind itself by beam
 * scanning
 * @param search_beam_length is the length of scanning beam
 * @param search_beam_radius_intensity is the resolution of scanning
 * @param search_range is the scanning range centering at ADV heading
 * @param is_block_angle_threshold is the threshold to tell how big a block
 *        angle range is perception blocking
 */
bool IsPerceptionBlocked(const ReferenceLineInfo& reference_line_info,
                         const double search_beam_length,
                         const double search_beam_radius_intensity,
                         const double search_range,
                         const double is_block_angle_threshold);

/**
 * @brief A static function to update the prepararion distance for lane change
 * @param is_opt_succeed if the optimization succeed
 * @param frame frame data for each planning cycle
 * @param reference_line_info struct containing reference line information
 * @param planning_context planning context for each planning cycle
 */
void UpdatePreparationDistance(
    const bool is_opt_succeed, const Frame* frame,
    const ReferenceLineInfo* const reference_line_info,
    PlanningContext* planning_context);

bool HysteresisFilter(const double obstacle_distance,
                      const double safe_distance, const double distance_buffer,
                      const bool is_obstacle_blocking);

}  // namespace planning
}  // namespace apollo
