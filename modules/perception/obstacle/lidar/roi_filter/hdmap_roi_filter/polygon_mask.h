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
#ifndef MODULES_PERCEPTION_OBSTACLE_LIDAR_ROI_FILTER_HDMAP_ROI_FILTER_PLYGON_MASK_H_
#define MODULES_PERCEPTION_OBSTACLE_LIDAR_ROI_FILTER_HDMAP_ROI_FILTER_PLYGON_MASK_H_

#include <limits>
#include <vector>
#include <algorithm>
#include <Eigen/StdVector>

#include "modules/common/log.h"
#include "modules/perception/obstacle/lidar/roi_filter/hdmap_roi_filter/bitmap2d.h"
#include "modules/perception/obstacle/lidar/roi_filter/hdmap_roi_filter/polygon_scan_cvter.h"

namespace apollo {
namespace perception {

void DrawPolygonMask(const typename PolygonScanConverter::Polygon &polygon,
                       Bitmap2D &bitmap, const double extend_dist);

void DrawPolygonMask(const std::vector<typename PolygonScanConverter::Polygon> &polygons,
                       Bitmap2D &bitmap, const double extend_dist);

} // perception
} // apollo

#endif // MODULES_PERCEPTION_OBSTACLE_LIDAR_ROI_FILTER_HDMAP_ROI_FILTER_PLYGON_MASK_H_

