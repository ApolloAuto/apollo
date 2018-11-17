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
#include "modules/perception/obstacle/lidar/roi_filter/hdmap_roi_filter/polygon_scan_converter.h"

namespace apollo {
namespace perception {

const double PolygonScanConverter::kEpsilon =
    std::numeric_limits<float>::epsilon();
const double PolygonScanConverter::kInf =
    std::numeric_limits<double>::infinity();

void PolygonScanConverter::Init(const DirectionMajor major_dir,
                                const Interval &valid_x_range,
                                const Polygon &polygon, const double step) {
  major_dir_ = major_dir;
  op_major_dir_ = OppositeDirection(major_dir);

  polygon_ = polygon;
  min_x_ = valid_x_range.first;
  scans_size_ = (valid_x_range.second - valid_x_range.first) / step;
  step_ = step;
}

void PolygonScanConverter::ConvertScans(
    std::vector<std::vector<Interval>> *scans_intervals) {
  scans_intervals->resize(scans_size_);

  DisturbPolygon();
  ConvertPolygonToSegments();
  BuildEdgeTable();

  for (const auto &edge : edge_table_[0]) {
    if (std::isfinite(edge.k)) {
      active_edge_table_.push_back(edge);
    } else {
      (*scans_intervals)[0].push_back(Interval(edge.y, edge.max_y));
    }
  }

  std::sort(active_edge_table_.begin(), active_edge_table_.end());
  for (size_t i = 0; i < active_edge_table_.size(); i += 2) {
    double min_y = active_edge_table_[i].y;
    double max_y = active_edge_table_[i + 1].y;
    (*scans_intervals)[0].push_back(Interval(min_y, max_y));
  }

  for (size_t i = 1; i < scans_size_; ++i) {
    UpdateActiveEdgeTable(i, &(scans_intervals->at(i)));
  }
}

void PolygonScanConverter::UpdateActiveEdgeTable(
    const size_t x_id, std::vector<Interval> *scan_intervals) {
  size_t valid_edges_num = active_edge_table_.size();
  size_t invalid_edges_num = 0;

  // For each edege in active edge table, check whether it is still valid.
  for (auto &edge : active_edge_table_) {
    if (!edge.MoveUp(step_)) {
      --valid_edges_num;
      ++invalid_edges_num;
      edge.y = kInf;
    }
  }

  size_t new_edges_num = 0;
  for (const auto &edge : edge_table_[x_id]) {
    if (std::isfinite(edge.k)) {
      ++valid_edges_num;
      ++new_edges_num;
      active_edge_table_.push_back(edge);
    } else {
      scan_intervals->emplace_back(edge.y, edge.max_y);
    }
  }

  if (invalid_edges_num != 0 || new_edges_num != 0) {
    std::sort(active_edge_table_.begin(), active_edge_table_.end(),
              [](const Edge &a, const Edge &b) { return a.y < b.y; });
    active_edge_table_.erase(next(active_edge_table_.begin(), valid_edges_num),
                             active_edge_table_.end());
  }

  for (size_t i = 0; i + 1 < active_edge_table_.size(); i += 2) {
    double min_y = active_edge_table_[i].y;
    double max_y = active_edge_table_[i + 1].y;

    scan_intervals->emplace_back(min_y, max_y);
  }
}

void PolygonScanConverter::BuildEdgeTable() {
  edge_table_.resize(scans_size_);

  std::vector<std::pair<int, Edge>> edges;
  edges.reserve(segments_.size());
  for (size_t i = 0; i < segments_.size(); ++i) {
    std::pair<int, Edge> out_edge;
    if (ConvertSegmentToEdge(i, &out_edge)) {
      edges.push_back(out_edge);
    }
  }

  active_edge_table_.reserve(edges.size());

  for (size_t i = 0; i < edges.size(); ++i) {
    int x_id = edges[i].first;
    const Edge &edge = edges[i].second;

    if (x_id >= static_cast<int>(scans_size_)) {
      continue;
    }

    if (x_id >= 0) {
      edge_table_[x_id].push_back(edge);
    } else {
      Edge active_edge = edge;
      if (active_edge.MoveUp(0.0 - active_edge.x)) {
        active_edge_table_.push_back(active_edge);
      }
    }
  }
}

bool PolygonScanConverter::ConvertSegmentToEdge(
    const size_t seg_id, std::pair<int, Edge> *out_edge) {
  const Segment &segment = segments_[seg_id];

  double min_x = segment.first[major_dir_] - min_x_;
  double min_y = segment.first[op_major_dir_];

  int x_id = std::ceil(min_x / step_);
  out_edge->first = x_id;

  Edge &edge = out_edge->second;
  edge.x = x_id * step_;
  edge.max_x = segment.second[major_dir_] - min_x_;
  edge.max_y = segment.second[op_major_dir_];
  edge.k = slope_[seg_id];

  if (std::isfinite(edge.k)) {
    edge.y = min_y + (edge.x - min_x) * edge.k;
  } else {
    edge.y = min_y;
    if (edge.y > edge.max_y) {
      std::swap(edge.y, edge.max_y);
    }
  }

  if (std::isfinite(edge.k) && edge.max_x < edge.x) {
    return false;
  }
  return true;
}

void PolygonScanConverter::ConvertPolygonToSegments() {
  size_t vertices_num = polygon_.size();

  segments_.reserve(vertices_num);
  slope_.reserve(vertices_num);

  for (size_t i = 0; i < vertices_num; ++i) {
    const Point &cur_vertex = polygon_[i];
    const Point &next_vertex = polygon_[(i + 1) % vertices_num];
    if (cur_vertex[major_dir_] < next_vertex[major_dir_]) {
      segments_.emplace_back(cur_vertex, next_vertex);
    } else {
      segments_.emplace_back(next_vertex, cur_vertex);
    }

    double x_diff = next_vertex[major_dir_] - cur_vertex[major_dir_];
    double y_diff = next_vertex[op_major_dir_] - cur_vertex[op_major_dir_];
    std::abs(cur_vertex[major_dir_] - next_vertex[major_dir_]) < kEpsilon
        ? slope_.push_back(kInf)
        : slope_.push_back(y_diff / x_diff);
  }
}

void PolygonScanConverter::DisturbPolygon() {
  for (auto &pt : polygon_) {
    // For current point, find the x coordinate of nearest scan line.
    // If they are too close, disturb the point in case of the point locating on
    // the line.
    double &x = pt[major_dir_];
    double d_x = (x - min_x_) / step_;
    int int_d_x = std::round(d_x);
    double delta_x = d_x - int_d_x;
    if (std::abs(delta_x) < kEpsilon) {
      if (delta_x > 0) {
        x = (int_d_x + kEpsilon) * step_ + min_x_;
      } else {
        x = (int_d_x - kEpsilon) * step_ + min_x_;
      }
    }
  }
}

bool PolygonScanConverter::Edge::MoveUp(const double delta_x) {
  if (delta_x < 0 || !std::isfinite(k)) {
    return false;
  }
  x += delta_x;
  if (x > max_x) {
    return false;
  }
  y += (delta_x * k);
  return true;
}

}  // namespace perception
}  // namespace apollo
