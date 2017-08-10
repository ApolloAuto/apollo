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
#ifndef MODULES_PERCEPTION_OBSTACLE_LIDAR_ROI_FILTER_HDMAP_ROI_FILTER_POLYGON_SCAN_CONVERTER_H_
#define MODULES_PERCEPTION_OBSTACLE_LIDAR_ROI_FILTER_HDMAP_ROI_FILTER_POLYGON_SCAN_CONVERTER_H_

#include <limits>
#include <vector>
#include "Eigen/Core"
#include "Eigen/StdVector"
#include "gflags/gflags.h"

#include "modules/common/log.h"
#include "modules/perception/obstacle/lidar/roi_filter/hdmap_roi_filter/bitmap2d.h"

namespace apollo {
namespace perception {

/*
 * @class PolygonScanConverter
 * @brief: This is a converter from polygon to scan lines, by which we can build
 * bitmap. Assume major direction as x direction, we scan polygons in x ascending
 * order.
 */

class PolygonScanConverter {
 public:
  typedef Eigen::Matrix<double, 2, 1> Point;
  typedef std::vector<Point, Eigen::aligned_allocator<Point>> Polygon;
  typedef std::pair<double, double> Interval;
  typedef std::pair<Point, Point> Segment;
  struct Edge {
      bool operator<(const Edge& other) const { return y < other.y; }
      bool MoveUp(const double delta_x);

      double max_x;
      double max_y;

      double x;
      double y;
      double k;
  };

  typedef Bitmap2D::DirectionMajor DirectionMajor;

  PolygonScanConverter(DirectionMajor major_dir) : major_dir_(major_dir) {
    op_major_dir_ = opposite_direction(major_dir);
  }

  static inline DirectionMajor opposite_direction(DirectionMajor dir_major) {
    return static_cast<DirectionMajor>(dir_major ^ 1);
  }

  void ConvertScans(const Interval& valid_range, const Polygon& polygon,
                    const double step,
                    std::vector<std::vector<Interval>>* scans_intervals);

 private:
  Polygon polygon_;
  std::vector<Segment> segments_;

  std::vector<double> slope_;

  std::vector<std::vector<Edge>> edge_table_;

  std::vector<Edge> active_edge_table_;

  double bottom_x_;
  double step_;
  size_t scans_size_;
  DirectionMajor major_dir_;
  DirectionMajor op_major_dir_;

  /*
   * @brief: If some point of polygon happens to be around the scan line we will
   * use, lightly change the x coordinate to avoid situation of singular point.
   */
  void DisturbPolygon();


  void BuildEdgeTable();

  void UpdateActiveEdgeTable(const size_t x_id,
                             std::vector<Interval>* scan_intervals);

  void ConvertPolygonToSegments();

  bool ConvertSegmentToEdge(size_t seg_id, std::pair<int, Edge>& out_edge);
};

}  // perception
}  // apollo
#endif  // MODULES_PERCEPTION_OBSTACLE_LIDAR_ROI_FILTER_HDMAP_ROI_FILTER_POLYGON_SCAN_CONVERTER_H_
