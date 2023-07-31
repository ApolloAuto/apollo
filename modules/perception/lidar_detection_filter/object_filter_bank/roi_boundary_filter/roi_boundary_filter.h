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
#include <vector>

#include "gtest/gtest_prod.h"

#include "Eigen/Dense"

#include "modules/common/util/eigen_defs.h"
#include "modules/perception/common/base/point_cloud.h"
#include "modules/perception/lidar_detection_filter/interface/base_object_filter.h"

namespace apollo {
namespace perception {
namespace lidar {

class ROIBoundaryFilter : public BaseObjectFilter {
 public:
  template <class EigenType>
  using EigenVector = apollo::common::EigenVector<EigenType>;

 public:
  /**
   * @brief Construct a new ROIBoundaryFilter object
   * 
   */
  ROIBoundaryFilter() = default;

  /**
   * @brief Destroy the ROIBoundaryFilter object
   * 
   */
  virtual ~ROIBoundaryFilter() = default;

  /**
   * @brief Init of ROIBoundaryFilter object
   * 
   * @param options object filer options
   * @return true 
   * @return false 
   */
  bool Init(const ObjectFilterInitOptions& options =
                ObjectFilterInitOptions()) override;

  /**
   * @brief filter objects using roi boundary filter algorithm
   * 
   * @param options object filter options
   * @param frame lidar frame to filter
   * @return true 
   * @return false 
   */
  bool Filter(const ObjectFilterOptions& options, LidarFrame* frame) override;

  /**
   * @brief Name of ROIBoundaryFilter object
   * 
   * @return std::string name
   */
  std::string Name() const override { return "ROIBoundaryFilter"; }

 private:
  // @brief: given input objects, build polygon in world frame
  void BuildWorldPolygons(const ObjectFilterOptions& options,
                          const LidarFrame& frame);
  // @brief: fill is_in_roi in lidar object supplement
  void FillObjectRoiFlag(const ObjectFilterOptions& options, LidarFrame* frame);
  // @brief: filter outside objects based on distance to boundary
  void FilterObjectsOutsideBoundary(const ObjectFilterOptions& options,
                                    LidarFrame* frame,
                                    std::vector<bool>* objects_valid_flag);
  // @brief: filter inside objects based on distance to boundary
  void FilterObjectsInsideBoundary(const ObjectFilterOptions& options,
                                   LidarFrame* frame,
                                   std::vector<bool>* objects_valid_flag);
  // @brief: filter objects based on position and confidence
  void FilterObjectsByConfidence(const ObjectFilterOptions& options,
                                 LidarFrame* frame,
                                 std::vector<bool>* objects_valid_flag);

 private:
  FRIEND_TEST(ROIBoundaryFilterTest, roi_boundary_filter_test);

  EigenVector<perception::base::PointDCloud> polygons_in_world_;
  std::vector<bool> objects_cross_roi_;
  std::vector<bool> objects_valid_flag_;
  // params
  double distance_to_boundary_threshold_ = 1.0;
  double inside_threshold_ = 1.0;
  float confidence_threshold_ = 0.5f;
  float cross_roi_threshold_ = 0.6f;
};  // class ROIBoundaryFilter

CYBER_PLUGIN_MANAGER_REGISTER_PLUGIN(
    apollo::perception::lidar::ROIBoundaryFilter, BaseObjectFilter)

}  // namespace lidar
}  // namespace perception
}  // namespace apollo
