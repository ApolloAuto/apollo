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

void GetValidXRange(const typename PolygonScanConverter::Polygon &polygon,
                    const Bitmap2D &bitmap,
                    const PolygonScanConverter::DirectionMajor major_dir,
                    const double major_dir_grid_size,
                    Interval* valid_x_range) {

  Eigen::Vector2d polygon_bottom_left_p, polygon_top_right_p;
  polygon_bottom_left_p.setConstant(std::numeric_limits<double>::max());
  polygon_top_right_p.setConstant(std::numeric_limits<double>::min());

  for (const auto &point : polygon) {
    polygon_bottom_left_p.x() = std::min(polygon_bottom_left_p.x(), point.x());
    polygon_bottom_left_p.y() = std::min(polygon_bottom_left_p.y(), point.y());

    polygon_top_right_p.x() = std::max(polygon_top_right_p.x(), point.x());
    polygon_top_right_p.y() = std::max(polygon_top_right_p.y(), point.y());
  }

  const Eigen::Vector2d& bitmap_bottom_left_p = bitmap.get_min_p();
  const Eigen::Vector2d& bitmap_top_right_p = bitmap.get_max_p();

  valid_x_range->first = std::max(polygon_bottom_left_p[major_dir],
                                  bitmap_bottom_left_p[major_dir]);
  valid_x_range->second = std::min(polygon_top_right_p[major_dir],
                                  bitmap_top_right_p[major_dir]);

  // For numerical stability
  valid_x_range->first = (
      static_cast<int>((valid_x_range->first - bitmap_bottom_left_p[major_dir])
      / major_dir_grid_size) + 0.5)
      * major_dir_grid_size + bitmap_bottom_left_p[major_dir];
}

void DrawPolygonInBitmap(const typename PolygonScanConverter::Polygon& polygon,
    const double extend_dist, Bitmap2D* bitmap) {
  PolygonScanConverter::DirectionMajor major_dir = bitmap->get_dir_major();
  PolygonScanConverter::DirectionMajor op_major_dir = bitmap->get_op_dir_major();
  double major_dir_grid_size = bitmap->get_grid_size()[major_dir];

  Interval valid_x_range;
  GetValidXRange(polygon, *bitmap, major_dir, major_dir_grid_size, &valid_x_range);

  std::vector<std::vector<Interval>> scans_intervals;
  PolygonScanConverter polygon_scan_converter(major_dir);
  polygon_scan_converter.ConvertScans(valid_x_range, polygon,
                                  major_dir_grid_size, &scans_intervals);


  const Eigen::Vector2d& bitmap_bottom_left_p = bitmap->get_min_p();
  const Eigen::Vector2d& bitmap_top_right_p = bitmap->get_max_p();
  double x = valid_x_range.first;
  for (size_t i = 0; i < scans_intervals.size(); x += major_dir_grid_size, ++i) {
    for (const auto &scan_interval : scans_intervals[i]) {
      Interval valid_y_range;
      valid_y_range.first = std::max(bitmap_bottom_left_p[op_major_dir],
          scan_interval.first - extend_dist);

      valid_y_range.second = std::min(bitmap_top_right_p[op_major_dir],
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
    for (const auto &polygon : polygons) {
        DrawPolygonInBitmap(polygon, extend_dist, bitmap);
    }
}


} // perception
} // apollo




