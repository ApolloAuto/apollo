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
 * @file lm_provider.h
 * @brief The class of LMProvider.
 */

#ifndef MODULES_LOCALIZATION_LMD_PREDICTOR_PERCEPTION_LM_PROVIDER_H_
#define MODULES_LOCALIZATION_LMD_PREDICTOR_PERCEPTION_LM_PROVIDER_H_

#include <utility>

#include "modules/common/proto/geometry.pb.h"
#include "modules/localization/proto/odometry_lane_marker.pb.h"

#include "modules/common/util/file.h"
#include "modules/localization/common/localization_gflags.h"

/**
 * @namespace apollo::localization
 * @brief apollo::localization
 */
namespace apollo {
namespace localization {

/**
 * @class LMProvider
 *
 * @brief  Provider of  odometry lane markers.
 */
class LMProvider {
 public:
  LMProvider();

  /**
   * @brief Find the nearest lane marker index with specified position.
   * @param position Specified position.
   * @return index pair of the nearest lane marker or max int pair
   */
  const std::pair<int64_t, int64_t> FindNearestLaneMarkerIndex(
      const apollo::common::PointENU& position) const;

  /**
   * @brief Get the index of prev lane marker.
   * @param current_index index pair of current lane marker.
   * @return index pair of prev lane marker or max int pair.
   */
  const std::pair<int64_t, int64_t> GetPrevLaneMarkerIndex(
      const std::pair<int64_t, int64_t>& current_index) const;

  /**
   * @brief Get the index of next lane marker.
   * @param current_index index pair of current lane marker.
   * @return index pair of next lane marker or max int pair.
   */
  const std::pair<int64_t, int64_t> GetNextLaneMarkerIndex(
      const std::pair<int64_t, int64_t>& current_index) const;

  /**
   * @brief Get the index of left lane marker.
   * @param current_index index pair of current lane marker.
   * @return index pair of left lane marker or max int pair.
   */
  const std::pair<int64_t, int64_t> GetLeftLaneMarkerIndex(
      const std::pair<int64_t, int64_t>& current_index) const;

  /**
   * @brief Get the index of right lane marker.
   * @param current_index index pair of current lane marker.
   * @return index pair of right lane marker or max int pair.
   */
  const std::pair<int64_t, int64_t> GetRightLaneMarkerIndex(
      const std::pair<int64_t, int64_t>& current_index) const;

  /**
   * @brief Get lane marker according to index pair.
   * @brief current_index index pair of current lane marker
   * @return A lane marker with the desired index pair or nullptr.
   */
  const apollo::localization::OdometryLaneMarker* GetLaneMarker(
      const std::pair<int64_t, int64_t>& current_index) const;

  /**
   * @brief Get the lane marker pack size.
   * @return size of LaneMarkersPack_
   */
  const int64_t GetLaneMarkerPackSize() const;

  /**
   * @brief Get the lane marker size according to the given pack_index.
   * @return size of Lane Markers of the given pack_index
   */
  const int64_t GetLaneMarkerSize(const int64_t& pack_index) const;

  /**
   * @brief Calclute the distance from point position to the line of start_pos
   * and end_pos.
   * @param position the point
   * @param start_pos the location of start point of the line
   * @param end_pos   the location of end point of the line
   * @return the distance value
   */
  double CalculateDistance(const apollo::common::PointENU& position,
                           const apollo::common::PointENU& current_pos) const;

 private:
  apollo::localization::OdometryLaneMarkersPack LaneMarkersPack_;
};

}  // namespace localization
}  // namespace apollo

#endif  // MODULES_LOCALIZATION_LMD_PREDICTOR_PERCEPTION_LM_PROVIDER_H_
