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

#include "modules/perception/base/point_cloud.h"
#include "modules/perception/lidar/lib/interface/base_roi_filter.h"
#include "modules/perception/lidar/lib/roi_filter/hdmap_roi_filter/bitmap2d.h"
#include "modules/perception/lidar/lib/scene_manager/roi_service/roi_service.h"

namespace apollo {
namespace perception {
namespace lidar {
class HdmapROIFilterTest;
class HdmapROIFilter : public BaseROIFilter {
 public:
  HdmapROIFilter()
      : BaseROIFilter(),
        range_(120.0),
        cell_size_(0.25),
        extend_dist_(0.0),
        no_edge_table_(false) {}
  ~HdmapROIFilter() = default;

  bool Init(const ROIFilterInitOptions& options) override;

  std::string Name() const override { return "HdmapROIFilter"; }

  bool Filter(const ROIFilterOptions& options, LidarFrame* frame) override;

 private:
  void TransformFrame(const base::PointFCloudPtr& cloud,
                      const Eigen::Affine3d& vel_pose,
                      const std::vector<base::PolygonDType*>& polygons_world,
                      std::vector<base::PolygonDType>* polygons_local,
                      base::PointFCloudPtr* cloud_local);

  bool FilterWithPolygonMask(
      const base::PointFCloudPtr& cloud,
      const std::vector<base::PolygonDType>& map_polygons,
      base::PointIndices* roi_indices);

  bool Bitmap2dFilter(const base::PointFCloudPtr& in_cloud,
                      const Bitmap2D& bitmap, base::PointIndices* roi_indices);

  // parameters for polygons scans convert
  double range_ = 120.0;
  double cell_size_ = 0.25;
  double extend_dist_ = 0.0;
  bool no_edge_table_ = false;
  bool set_roi_service_ = false;
  std::vector<base::PolygonDType*> polygons_world_;
  std::vector<base::PolygonDType> polygons_local_;
  Bitmap2D bitmap_;
  ROIServiceContent roi_service_content_;

  // unit tests only
  friend class HdmapROIFilterTest;
  friend class LidarLibROIServiceTest;
};

}  // namespace lidar
}  // namespace perception
}  // namespace apollo
