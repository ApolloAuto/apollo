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
#ifndef MODULES_PERCEPTION_OBSTACLE_LIDAR_INTERFACE_HDMAP_ROI_FILTER_H_
#define MODULES_PERCEPTION_OBSTACLE_LIDAR_INTERFACE_HDMAP_ROI_FILTER_H_

#include <algorithm>
#include <string>
#include <vector>

#include "Eigen/Core"
#include "gflags/gflags.h"

#include "modules/common/log.h"
#include "modules/perception/common/perception_gflags.h"
#include "modules/perception/lib/config_manager/config_manager.h"
#include "modules/perception/obstacle/base/hdmap_struct.h"
#include "modules/perception/obstacle/lidar/interface/base_roi_filter.h"
#include "modules/perception/obstacle/lidar/roi_filter/hdmap_roi_filter/bitmap2d.h"
#include "modules/perception/obstacle/lidar/roi_filter/hdmap_roi_filter/polygon_mask.h"
#include "modules/perception/obstacle/lidar/roi_filter/hdmap_roi_filter/polygon_scan_converter.h"
#include "modules/perception/obstacle/onboard/hdmap_input.h"

namespace apollo {
namespace perception {

typedef typename Bitmap2D::DirectionMajor MajorDirection;

/**
 * @class HdmapROIFilter
 * @brief This is ROI(Region of Interest) Filter based on HD map, which can
 * figure out which point is in the regions what we focus on(Almost in the
 * road).
 * This is an optional process, the most advantage of ROI filter is to filter
 * out some points to reduce calculation in the following process.
 */
class HdmapROIFilter : public BaseROIFilter {
 public:
  HdmapROIFilter() : BaseROIFilter() {}
  ~HdmapROIFilter() {}

  bool Init() override;
  std::string name() const override {
    return "HdmapROIFilter";
  }

  /**
   * @params[In] cloud: All the cloud points with local coordinates
   * @params[In] roi_filter_options: Type definition in
   * "../../interface/base_roi_filter.h". Contains the information
   * of ROI and world to local coordinates transformation matrix.
   * @params[Out] roi_indices: The indices of points within ROI
   * @return true if filter points successfully, otherwise return false
   */
  bool Filter(const pcl_util::PointCloudPtr& cloud,
              const ROIFilterOptions& roi_filter_options,
              pcl_util::PointIndices* roi_indices) override;

  /**
   * @brief: Merge junction polygons and road boundaries in a vector.
   */
  void MergeHdmapStructToPolygons(const HdmapStructConstPtr& hdmap_struct_ptr,
                                  std::vector<PolygonDType>* polygons);

 protected:
  /**
   * @brief: Draw polygons into grids in bitmap and check each point whether
   * is in the grids within ROI.
   */
  bool FilterWithPolygonMask(const pcl_util::PointCloudPtr& cloud,
                             const std::vector<PolygonType>& map_polygons,
                             pcl_util::PointIndices* roi_indices);

  /**
   * @brief: Transform polygon points and cloud points from world coordinates
   * system to local.
   */
  void TransformFrame(const pcl_util::PointCloudConstPtr& cloud,
                      const Eigen::Affine3d& vel_pose,
                      const std::vector<PolygonDType>& polygons_world,
                      std::vector<PolygonType>* polygons_local,
                      pcl_util::PointCloudPtr cloud_local);

  /**
   * @brief: Get major direction. Transform polygons type to what we want.
   */
  MajorDirection GetMajorDirection(
      const std::vector<PolygonType>& map_polygons,
      std::vector<PolygonScanConverter::Polygon>* polygons);

  /**
   * @brief: Merge left boundary and right boundary of each road into polygon
   * type.
   */
  void MergeRoadBoundariesToPolygons(
      const std::vector<RoadBoundary>& road_boundaries,
      std::vector<PolygonDType>* polygons);

  /**
   * @brief: After drawing polygons into grids in bitmap. We check each point
   * whether is in the grids within ROI.
   */
  bool Bitmap2dFilter(
      const pcl::PointCloud<pcl_util::Point>::ConstPtr in_cloud_ptr,
      const Bitmap2D& bitmap, pcl_util::PointIndices* roi_indices_ptr);

  // We only filter point with local coordinates x, y in [-range, range] in
  // meters
  double range_ = 0.0;

  // Hight and width of grid in bitmap
  double cell_size_ = 0.0;

  // The distance extended away from the ROI boundary
  double extend_dist_ = 0.0;
};

REGISTER_ROIFILTER(HdmapROIFilter);

}  // namespace perception
}  // namespace apollo

#endif  // MODULES_PERCEPTION_OBSTACLE_LIDAR_INTERFACE_HDMAP_ROI_FILTER_H_
