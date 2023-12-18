/******************************************************************************
 * Copyright 2023 The Apollo Authors. All Rights Reserved.
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
#include <vector>
#include <limits>

#include "cyber/common/log.h"
#include "cyber/common/file.h"

#include "modules/perception/common/algorithm/geometry/roi_filter.h"
#include "modules/perception/common/algorithm/geometry/common.h"
#include "modules/perception/lidar_detection_filter/interface/base_object_filter.h"
#include "modules/perception/lidar_detection_filter/object_filter_bank/proto/background_filter_config.pb.h"

namespace apollo {
namespace perception {
namespace lidar {

class BackgroundFilter : public BaseObjectFilter {
 public:
  template <class EigenType>
  using EigenVector = apollo::common::EigenVector<EigenType>;

 public:
  /**
   * @brief Construct a new Background Filter object
   * 
   */
  BackgroundFilter() = default;

  /**
   * @brief Destroy the Background Filter object
   * 
   */
  virtual ~BackgroundFilter() = default;

  /**
   * @brief Init of Background Filter
   * 
   * @param options object filer options
   * @return true 
   * @return false 
   */
  bool Init(const ObjectFilterInitOptions& options =
                ObjectFilterInitOptions()) override;

  /**
   * @brief Filter obejcts usign background filter
   * 
   * @param options object filter options
   * @param frame lidar frame
   * @return true 
   * @return false 
   */
  bool Filter(const ObjectFilterOptions& options, LidarFrame* frame) override;

  /**
   * @brief Name of background filter
   * 
   * @return std::string name
   */
  std::string Name() const override { return "BackgroundFilter"; }

  bool FillInRoiFlag(const ObjectFilterOptions& options,
                     const LidarFrame& frame);

  void BuildWorldPolygons(const ObjectFilterOptions& options,
                          const LidarFrame& frame);

 private:
  double outside_roi_filter_distance_;
  EigenVector<perception::base::PointDCloud> polygons_in_world_;
  std::vector<bool> is_in_roi_flag_;
};  // class BackgroundFilter

CYBER_PLUGIN_MANAGER_REGISTER_PLUGIN(
    apollo::perception::lidar::BackgroundFilter, BaseObjectFilter)

}  // namespace lidar
}  // namespace perception
}  // namespace apollo
