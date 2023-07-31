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
#pragma once

#include <string>

#include "modules/perception/common/algorithm/geometry/convex_hull_2d.h"
#include "modules/perception/lidar_tracking/tracker/multi_lidar_fusion/mlf_base_filter.h"

namespace apollo {
namespace perception {
namespace lidar {

class MlfShapeFilter : public MlfBaseFilter {
 public:
  MlfShapeFilter() = default;
  virtual ~MlfShapeFilter() = default;
  /**
   * @brief Init mlf filter
   *
   * @param options
   * @return true
   * @return false
   */
  bool Init(
      const MlfFilterInitOptions& options = MlfFilterInitOptions()) override;
  /**
   * @brief Updating shape filter with object
   *
   * @param options for updating
   * @param track_data not include new object
   * @param new_object new object for updating
   */
  void UpdateWithObject(const MlfFilterOptions& options,
                        const MlfTrackDataConstPtr& track_data,
                        TrackedObjectPtr new_object) override;
  /**
   * @brief Updating shape filter without object
   *
   * @param options for updating
   * @param timestamp current timestamp
   * @param track_data track data to be updated
   */
  void UpdateWithoutObject(const MlfFilterOptions& options, double timestamp,
                           MlfTrackDataPtr track_data) override;
  /**
   * @brief Get class name
   *
   * @return std::string
   */
  std::string Name() const override { return "MlfShapeFilter"; }

 protected:
  algorithm::ConvexHull2D<base::PointDCloud, base::PolygonDType> hull_;
  double bottom_points_ignore_threshold_ = 0.1;
  double top_points_ignore_threshold_ = 1.6;
};  // class MlfShapeFilter

}  // namespace lidar
}  // namespace perception
}  // namespace apollo
