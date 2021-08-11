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

#include <algorithm>
#include <limits>
#include <vector>

#include "modules/perception/lidar/common/lidar_log.h"
#include "modules/perception/lidar/lib/roi_filter/hdmap_roi_filter/bitmap2d.h"
#include "modules/perception/lidar/lib/roi_filter/hdmap_roi_filter/polygon_scan_cvter.h"

namespace apollo {
namespace perception {
namespace lidar {

template <typename T>
bool DrawPolygonMask(const typename PolygonScanCvter<T>::Polygon& polygon,
                     Bitmap2D* bitmap, const double extend_dist = 0.0,
                     const bool no_edge_table = false);

template <typename T>
bool DrawPolygonsMask(
    const std::vector<typename PolygonScanCvter<T>::Polygon>& polygons,
    Bitmap2D* bitmap, const double extend_dist = 0.0,
    const bool no_edge_table = false);

template <typename T>
bool DrawPolygonMask(const typename PolygonScanCvter<T>::Polygon& polygon,
                     Bitmap2D* bitmap, const double extend_dist,
                     const bool no_edge_table) {
  typedef typename PolygonScanCvter<T>::IntervalIn IntervalIn;
  typedef typename PolygonScanCvter<T>::IntervalOut IntervalOut;
  typedef typename PolygonScanCvter<T>::DirectionMajor PolyDirMajor;
  if (bitmap->Empty()) {
    AERROR << "bitmap is empty";
    return false;
  }
  Eigen::Vector2d poly_min_p, poly_max_p;
  poly_min_p.setConstant(std::numeric_limits<double>::max());
  poly_max_p = -poly_min_p;
  for (const auto& pt : polygon) {
    poly_min_p.x() = std::min(pt.x(), poly_min_p.x());
    poly_min_p.y() = std::min(pt.y(), poly_min_p.y());

    poly_max_p.x() = std::max(pt.x(), poly_max_p.x());
    poly_max_p.y() = std::max(pt.y(), poly_max_p.y());
  }
  if (poly_max_p.x() <= poly_min_p.x()) {
    AERROR << "Invalid polygon";
    return false;
  }
  if (poly_max_p.y() <= poly_min_p.y()) {
    AERROR << "Invalid polygon";
    return false;
  }
  const Eigen::Vector2d& bitmap_min_range = bitmap->min_range();
  const Eigen::Vector2d& bitmap_max_range = bitmap->max_range();
  const Eigen::Vector2d& cell_size = bitmap->cell_size();
  const int major_dir = bitmap->dir_major();
  const int op_major_dir = bitmap->op_dir_major();

  // check major x range
  IntervalIn valid_range;
  valid_range.first =
      std::max(poly_min_p[major_dir], bitmap_min_range[major_dir]);
  valid_range.second =
      std::min(poly_max_p[major_dir], bitmap_max_range[major_dir]);

  // for numerical stability
  valid_range.first =
      (static_cast<int>((valid_range.first - bitmap_min_range[major_dir]) /
                        cell_size[major_dir]) +
       0.5) *
          cell_size[major_dir] +
      bitmap_min_range[major_dir];

  if (valid_range.second < valid_range.first + cell_size[major_dir]) {
    AWARN << "Invalid range: " << valid_range.first << " " << valid_range.second
          << ". polygon major directory range: " << poly_min_p[major_dir] << " "
          << poly_max_p[major_dir] << ". cell size: " << cell_size[major_dir];
    return true;
  }

  // start calculating intervals of scans
  PolygonScanCvter<T> poly_scan_cvter;
  poly_scan_cvter.Init(polygon);
  std::vector<std::vector<IntervalOut>> scans_intervals;
  if (no_edge_table) {
    size_t scans_size = static_cast<size_t>(
        (valid_range.second - valid_range.first) / cell_size[major_dir]);
    scans_intervals.resize(scans_size);
    for (size_t i = 0; i < scans_size; ++i) {
      double scan_loc = valid_range.first + i * cell_size[major_dir];
      poly_scan_cvter.ScanCvt(scan_loc, static_cast<PolyDirMajor>(major_dir),
                              &(scans_intervals[i]));
    }
  } else {
    poly_scan_cvter.ScansCvt(valid_range, static_cast<PolyDirMajor>(major_dir),
                             cell_size[major_dir], &(scans_intervals));
  }
  // start to draw
  double x = valid_range.first;
  for (size_t i = 0; i < scans_intervals.size();
       x += cell_size[major_dir], ++i) {
    for (auto scan_interval : scans_intervals[i]) {
      if (scan_interval.first > scan_interval.second) {
        AERROR << "The input polygon is illegal(complex polygon)";
        return false;
      }

      // extend
      scan_interval.first -= extend_dist;
      scan_interval.second += extend_dist;

      IntervalOut valid_y_range;
      valid_y_range.first =
          std::max(bitmap_min_range[op_major_dir], scan_interval.first);
      valid_y_range.second =
          std::min(bitmap_max_range[op_major_dir], scan_interval.second);
      if (valid_y_range.first > valid_y_range.second) {
        continue;
      }
      bitmap->Set(x, valid_y_range.first, valid_y_range.second);
    }
  }
  return true;
}

template <typename T>
bool DrawPolygonsMask(
    const std::vector<typename PolygonScanCvter<T>::Polygon>& polygons,
    Bitmap2D* bitmap, const double extend_dist, const bool no_edge_table) {
  for (const auto& polygon : polygons) {
    bool flag = DrawPolygonMask<T>(polygon, bitmap, extend_dist, no_edge_table);
    if (!flag) {
      return false;
    }
  }
  return true;
}

}  // namespace lidar
}  // namespace perception
}  // namespace apollo
