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
#include "modules/perception/obstacle/lidar/roi_filter/hdmap_roi_filter/polygon_mask.h"

namespace apollo {
namespace perception {

void GetValidXRange(const typename PolygonScanConverter::Polygon& polygon,
                    const Bitmap2D& bitmap,
                    const PolygonScanConverter::DirectionMajor major_dir,
                    const double major_dir_grid_size, Interval* valid_x_range) {
  Eigen::Vector2d polygon_min_pt, polygon_max_pt;
  polygon_min_pt.setConstant(std::numeric_limits<double>::max());
  polygon_max_pt.setConstant(std::numeric_limits<double>::min());

  for (const auto& point : polygon) {
    polygon_min_pt.x() = std::min(polygon_min_pt.x(), point.x());
    polygon_min_pt.y() = std::min(polygon_min_pt.y(), point.y());

    polygon_max_pt.x() = std::max(polygon_max_pt.x(), point.x());
    polygon_max_pt.y() = std::max(polygon_max_pt.y(), point.y());
  }

  const Eigen::Vector2d& bitmap_min_pt = bitmap.get_min_p();
  const Eigen::Vector2d& bitmap_max_pt = bitmap.get_max_p();

  valid_x_range->first =
      std::max(polygon_min_pt[major_dir], bitmap_min_pt[major_dir]);
  valid_x_range->second =
      std::min(polygon_max_pt[major_dir], bitmap_max_pt[major_dir]);

  // For numerical stability
  valid_x_range->first =
      (static_cast<int>((valid_x_range->first - bitmap_min_pt[major_dir]) /
                        major_dir_grid_size) +
       0.5) *
          major_dir_grid_size +
      bitmap_min_pt[major_dir];
}

void DrawPolygonInBitmap(const typename PolygonScanConverter::Polygon& polygon,
                         const double extend_dist, Bitmap2D* bitmap) {
  PolygonScanConverter::DirectionMajor major_dir = bitmap->get_dir_major();
  PolygonScanConverter::DirectionMajor op_major_dir =
      bitmap->get_op_dir_major();
  double major_dir_grid_size = bitmap->get_grid_size()[major_dir];

  // 1. Get valid x range
  Interval valid_x_range;
  GetValidXRange(polygon, *bitmap, major_dir, major_dir_grid_size,
                 &valid_x_range);

  // 2. Convert polygon to scan intervals(Most important)
  std::vector<std::vector<Interval>> scans_intervals;

  PolygonScanConverter polygon_scan_converter;
  polygon_scan_converter.Init(major_dir, valid_x_range, polygon,
                              major_dir_grid_size);
  polygon_scan_converter.ConvertScans(&scans_intervals);

  // 3. Draw grids in bitmap based on scan intervals
  const Eigen::Vector2d& bitmap_min_pt = bitmap->get_min_p();
  const Eigen::Vector2d& bitmap_max_pt = bitmap->get_max_p();
  double x = valid_x_range.first;
  for (size_t i = 0; i < scans_intervals.size();
       x += major_dir_grid_size, ++i) {
    for (const auto& scan_interval : scans_intervals[i]) {
      if (scan_interval.first > scan_interval.second) {
        AERROR << "scan interval is not valid: "
               << "scan_interval.first = " << scan_interval.first << ", "
               << "scan_interval.second = " << scan_interval.second << ".";
      }
      Interval valid_y_range;
      valid_y_range.first = std::max(bitmap_min_pt[op_major_dir],
                                     scan_interval.first - extend_dist);

      valid_y_range.second = std::min(bitmap_max_pt[op_major_dir],
                                      scan_interval.second + extend_dist);

      if (valid_y_range.first > valid_y_range.second) {
        continue;
      }
      bitmap->Set(x, valid_y_range.first, valid_y_range.second);
    }
  }
}

void DrawPolygonInBitmap(
    const std::vector<typename PolygonScanConverter::Polygon>& polygons,
    const double extend_dist, Bitmap2D* bitmap) {
  for (const auto& polygon : polygons) {
    DrawPolygonInBitmap(polygon, extend_dist, bitmap);
  }
}

}  // namespace perception
}  // namespace apollo
