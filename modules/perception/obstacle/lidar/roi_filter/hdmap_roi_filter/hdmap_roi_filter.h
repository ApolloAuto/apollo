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

#include "modules/perception/obstacle/lidar/interface/base_roi_filter.h"

#include <vector>
#include <gflags/gflags.h>
#include <Eigen/Core>

#include "modules/common/log.h"
#include "modules/perception/lib/config_manager/config_manager.h"
#include "modules/perception/lib/base/singleton.h"
#include "modules/perception/obstacle/base/hdmap_struct.h"
#include "modules/perception/obstacle/onboard/hdmap_input.h"
#include "modules/perception/obstacle/lidar/roi_filter/hdmap_roi_filter/polygon_scan_cvter.h"
#include "modules/perception/obstacle/lidar/roi_filter/hdmap_roi_filter/polygon_mask.h"
#include "modules/perception/obstacle/lidar/roi_filter/hdmap_roi_filter/bitmap2d.h"
#include "modules/perception/common/perception_gflags.h"

namespace apollo {
namespace perception {

class HdmapROIFilter: public BaseROIFilter {
public:
    HdmapROIFilter() : BaseROIFilter() {}
    ~HdmapROIFilter() {}

    bool Init() override;
    std::string name() const {return "HdmapROIFilter";}

    bool Filter(const pcl_util::PointCloudPtr& cloud,
                const ROIFilterOptions &roi_filter_options,
                pcl_util::PointIndices* roi_indices) override;

    bool FilterWithPolygonMask(const pcl_util::PointCloudPtr& cloud,
                               const std::vector<PolygonType>& map_polygons,
                               pcl_util::PointIndices* roi_indices);

protected:
    void TransformFrame(
            const pcl_util::PointCloudConstPtr& cloud,
            const Eigen::Affine3d& vel_pose,
            const std::vector<PolygonDType>& polygons_world,
            std::vector<PolygonType>& polygons_local,
            pcl_util::PointCloudPtr& cloud_local);

      // parameters for polygons scans convert
    double range_;
    double cell_size_;
    double extend_dist_;
};


} // perception
} // apollo

#endif // MODULES_PERCEPTION_OBSTACLE_LIDAR_INTERFACE_HDMAP_ROI_FILTER_H_
