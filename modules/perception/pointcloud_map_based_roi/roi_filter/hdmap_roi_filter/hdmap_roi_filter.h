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

#include "modules/perception/pointcloud_map_based_roi/roi_filter/hdmap_roi_filter/proto/hdmap_roi_filter.pb.h"

#include "modules/common/util/eigen_defs.h"
#include "modules/perception/common/base/point_cloud.h"
#include "modules/perception/common/lidar/scene_manager/ground_service/ground_service.h"
#include "modules/perception/common/lidar/scene_manager/roi_service/roi_service.h"
#include "modules/perception/common/onboard/inner_component_messages/lidar_inner_component_messages.h"
#include "modules/perception/pointcloud_map_based_roi/interface/base_roi_filter.h"
#include "modules/perception/pointcloud_map_based_roi/roi_filter/hdmap_roi_filter/bitmap2d.h"

namespace apollo {
namespace perception {
namespace lidar {

class HdmapROIFilter : public BaseROIFilter {
 public:
  /**
   * @brief Construct a new Hdmap ROI Filter object
   * 
   */
  HdmapROIFilter()
      : range_(120.0),
        cell_size_(0.25),
        extend_dist_(0.0),
        no_edge_table_(false) {}
  /**
   * @brief Destroy the Hdmap ROI Filter object
   * 
   */
  ~HdmapROIFilter() = default;

  /**
   * @brief Init of Hdmap ROI Filter object
   * 
   * @param options roi filter options
   * @return true 
   * @return false 
   */
  bool Init(const ROIFilterInitOptions& options) override;

  /**
   * @brief filter point cloud outside of hdmap roi
   * 
   * @param options roi filter options
   * @param frame lidar frame
   * @return true 
   * @return false 
   */
  bool Filter(const ROIFilterOptions& options, LidarFrame* frame) override;

  /**
   * @brief Name of Hdmap ROI Filter object
   * 
   * @return std::string 
   */
  std::string Name() const override { return "HdmapROIFilter"; }

 private:
  void TransformFrame(
      const base::PointFCloudPtr& cloud, const Eigen::Affine3d& vel_pose,
      const apollo::common::EigenVector<base::PolygonDType*>& polygons_world,
      apollo::common::EigenVector<base::PolygonDType>* polygons_local,
      base::PointFCloudPtr* cloud_local);

  bool FilterWithPolygonMask(
      const base::PointFCloudPtr& cloud,
      const apollo::common::EigenVector<base::PolygonDType>& map_polygons,
      base::PointIndices* roi_indices);

  bool Bitmap2dFilter(const base::PointFCloudPtr& in_cloud,
                      const Bitmap2D& bitmap, base::PointIndices* roi_indices);

  // parameters for polygons scans convert
  double range_ = 120.0;
  double cell_size_ = 0.25;
  double extend_dist_ = 0.0;
  bool no_edge_table_ = false;
  bool set_roi_service_ = false;
  apollo::common::EigenVector<base::PolygonDType*> polygons_world_;
  apollo::common::EigenVector<base::PolygonDType> polygons_local_;
  Bitmap2D bitmap_;
  ROIServiceContent roi_service_content_;
};

CYBER_PLUGIN_MANAGER_REGISTER_PLUGIN(apollo::perception::lidar::HdmapROIFilter,
                                     BaseROIFilter)

}  // namespace lidar
}  // namespace perception
}  // namespace apollo
