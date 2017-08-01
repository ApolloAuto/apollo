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

void DrawPolygonMask(const typename PolygonScanConverter::Polygon &polygon,
    Bitmap2D &bitmap, const double extend_dist) {
  typedef typename PolygonScanConverter::Interval Interval;

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
  const Eigen::Vector2d& grid_size = bitmap.get_grid_size();

  //Important!!! In the following code, we assume the major direction as x
  //coordinates, op major direction as y coordinates.
  PolygonScanConverter::DirectionMajor major_dir = bitmap.get_dir_major();
  PolygonScanConverter::DirectionMajor op_major_dir = bitmap.get_op_dir_major();

  // check major x range
  Interval valid_x_range;
  valid_x_range.first = std::max(polygon_bottom_left_p[major_dir],
                                 bitmap_bottom_left_p[major_dir]);
  valid_x_range.second = std::min(polygon_top_right_p[major_dir],
                                  bitmap_top_right_p[major_dir]);

  // for numerical stability
  valid_x_range.first = (static_cast<int>(
      (valid_x_range.first - bitmap_bottom_left_p[major_dir])
      / grid_size[major_dir]) + 0.5)
      * grid_size[major_dir] + bitmap_bottom_left_p[major_dir];

  PolygonScanConverter polygon_scan_cvter(major_dir);


  std::vector<std::vector<Interval>> scans_intervals;
  polygon_scan_cvter.ConvertScans(valid_x_range, polygon, grid_size[major_dir], &scans_intervals);

  double x = valid_x_range.first;
  for (size_t i = 0; i < scans_intervals.size(); x += grid_size[major_dir], ++i) {
    for (const auto &scan_interval : scans_intervals[i]) {
      Interval valid_y_range;
      valid_y_range.first = std::max(bitmap_bottom_left_p[op_major_dir],
              scan_interval.first - extend_dist);

      valid_y_range.second = std::min(bitmap_top_right_p[op_major_dir],
              scan_interval.second + extend_dist);

      if (valid_y_range.first > valid_y_range.second) {
        continue;
      }
      bitmap.Set(x, valid_y_range.first, valid_y_range.second);
    }
  }
}

void DrawPolygonMask(const std::vector<typename PolygonScanConverter::Polygon>& polygons,
        Bitmap2D& bitmap, const double extend_dist) {
    for (const auto &polygon : polygons) {
        DrawPolygonMask(polygon, bitmap, extend_dist);
    }
}


} // perception
} // apollo




