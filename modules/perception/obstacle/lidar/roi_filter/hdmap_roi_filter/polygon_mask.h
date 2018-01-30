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
#ifndef MODULES_PERCEPTION_OBSTACLE_LIDAR_ROI_FILTER_HDMAP_ROI_FILTER_PM_H_
#define MODULES_PERCEPTION_OBSTACLE_LIDAR_ROI_FILTER_HDMAP_ROI_FILTER_PM_H_

#include <algorithm>
#include <limits>
#include <vector>

#include "Eigen/StdVector"

#include "modules/common/log.h"
#include "modules/perception/obstacle/lidar/roi_filter/hdmap_roi_filter/bitmap2d.h"
#include "modules/perception/obstacle/lidar/roi_filter/hdmap_roi_filter/polygon_scan_converter.h"

namespace apollo {
namespace perception {

typedef typename PolygonScanConverter::Interval Interval;

void DrawPolygonInBitmap(const PolygonScanConverter::Polygon& polygon,
                         const double extend_dist, Bitmap2D* bitmap);

void DrawPolygonInBitmap(
    const std::vector<PolygonScanConverter::Polygon>& polygons,
    const double extend_dist, Bitmap2D* bitmap);

/*
 * @brief: Get valid x range(Major direction range)
 */
void GetValidXRange(const PolygonScanConverter::Polygon& polygon,
                    const Bitmap2D& bitmap,
                    const PolygonScanConverter::DirectionMajor major_dir,
                    const double major_dir_grid_size, Interval* valid_x_range);

}  // namespace perception
}  // namespace apollo

#endif  // MODULES_PERCEPTION_OBSTACLE_LIDAR_ROI_FILTER_HDMAP_ROI_FILTER_PM_H_
