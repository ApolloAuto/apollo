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
#include <utility>
#include <vector>

#include "Eigen/Core"
#include "Eigen/StdVector"

#include "modules/perception/lidar/common/lidar_log.h"

namespace apollo {
namespace perception {
namespace lidar {

template <typename T = double>
class PolygonScanCvter {
 public:
  typedef Eigen::Matrix<T, 2, 1> Point;
  typedef std::vector<Point, Eigen::aligned_allocator<Point>> Polygon;
  typedef std::pair<T, T> IntervalIn;
  typedef std::pair<double, double> IntervalOut;
  typedef std::pair<Point, Point> Segment;

  struct Edge {
    bool operator<(const Edge& other) const { return y < other.y; }
    bool MoveUp(double delta_x) {
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
    friend std::ostream& operator<<(std::ostream& out, const Edge& edge) {
      out << boost::format(
                 "max_x: %13.10lf max_y: %13.10lf"
                 "min_x: %13.10lf min_y: %13.10lf"
                 "x: %13.10lf y: %13.10lf k %13.10lf") %
                 edge.max_x % edge.max_y % edge.min_x % edge.min_y % edge.x %
                 edge.y % edge.k;
      return out;
    }
    // high x end point constant vars
    double max_x = 0.0;
    double max_y = 0.0;
    // for test
    double min_x = 0.0;
    double min_y = 0.0;
    // initial to low x y and move
    double x = 0.0;
    double y = 0.0;
    double k = 0.0;
  };

  enum DirectionMajor { XMAJOR = 0, YMAJOR = 1 };

  static inline DirectionMajor OppositeDirection(
      const DirectionMajor dir_major) {
    return static_cast<DirectionMajor>(dir_major ^ 1);
  }

  PolygonScanCvter() = default;
  virtual ~PolygonScanCvter() = default;

  const Polygon& polygon() const { return polygon_; }
  void Init(const Polygon& polygon);
  void Reset();

  // get scan intervals
  // HORIZONTAL = 0 for x, VERTICAL = 1 for y
  void ScanCvt(const T& scan_loc, DirectionMajor dir_major,
               std::vector<IntervalOut>* scan_intervals);
  void ScansCvt(const IntervalIn& scans_interval,
                const DirectionMajor dir_major, const T& step,
                std::vector<std::vector<IntervalOut>>* scans_intervals);

  static const double s_epsilon_;
  static const double s_inf_;

 private:
  void DisturbPolygon(const DirectionMajor dir_major);
  void ParsePolygon(const DirectionMajor dir_major, const bool disturb = false);
  void BuildEt();
  void UpdateAet(std::vector<IntervalOut>* scan_intervals);
  // convert segment, x  from continuous domain to discrete domain
  bool ConvertSegment(const size_t seg_id, std::pair<int, Edge>* out_edge);

  Polygon polygon_;
  Polygon polygon_disturbed_;
  std::vector<std::vector<bool>> is_singular_;  // 0 for x 1 for y
  std::vector<std::vector<Segment>> segments_;  // 0 for x 1 for y
  // ensure the k = _s_inf edge to be filled
  std::vector<std::vector<double>> ks_;  // 0 for x 1 for y

  std::vector<std::pair<double, double>> top_segments_;

  // edge table
  std::vector<std::vector<Edge>> et_;
  // active edge table
  std::pair<size_t, std::vector<Edge>> aet_;
  double bottom_x_ = 0.0;
  double step_ = 0.0;
  size_t scans_size_ = 0;
  DirectionMajor dir_major_ = DirectionMajor::XMAJOR;
  DirectionMajor op_dir_major_ = DirectionMajor::YMAJOR;
};

template <typename T>
const double PolygonScanCvter<T>::s_epsilon_ =
    std::numeric_limits<float>::epsilon();

template <typename T>
const double PolygonScanCvter<T>::s_inf_ = std::numeric_limits<T>::infinity();

template <typename T>
void PolygonScanCvter<T>::Init(const Polygon& polygon) {
  Reset();
  polygon_ = polygon;
  is_singular_.resize(2);
  segments_.resize(2);
  ks_.resize(2);
}

template <typename T>
void PolygonScanCvter<T>::Reset() {
  polygon_ = Polygon();
  is_singular_.clear();
  segments_.clear();
  ks_.clear();
}

template <typename T>
void PolygonScanCvter<T>::ScanCvt(const T& scan_loc, DirectionMajor dir_major,
                                  std::vector<IntervalOut>* scan_intervals) {
  DirectionMajor op_dir_major = OppositeDirection(dir_major);
  double x = scan_loc;
  auto& is_singular = is_singular_[dir_major];
  auto& segments = segments_[dir_major];
  auto& ks = ks_[dir_major];
  CHECK_EQ(segments.size(), polygon_.size());
  CHECK_EQ(segments.size(), ks.size());
  scan_intervals->clear();
  scan_intervals->reserve(polygon_.size());
  std::vector<double> nodes;
  nodes.reserve(polygon_.size());
  // for normal edge
  for (size_t i = 0; i < segments.size(); ++i) {
    const Segment& segment = segments[i];
    const Point& low_vertex = segment.first;
    const Point& high_vertex = segment.second;
    double high_x = high_vertex[dir_major];
    double high_y = high_vertex[op_dir_major];
    double low_x = low_vertex[dir_major];
    double low_y = low_vertex[op_dir_major];
    double k = ks[i];
    if (std::isfinite(k)) {
      if ((x >= low_x && x < high_x) || (x <= low_x && x > high_x)) {
        if (x == low_x) {
          nodes.push_back(low_y);
          if (is_singular[i]) {
            nodes.push_back(low_y);
          }
        } else {
          nodes.push_back(low_y + (x - low_x) * k);
        }
      }
    } else {
      if (std::abs(x - low_x) < s_epsilon_ ||
          std::abs(x - high_x) < s_epsilon_) {
        if (low_y < high_y) {
          scan_intervals->push_back(IntervalOut(low_y, high_y));
        } else {
          scan_intervals->push_back(IntervalOut(high_y, low_y));
        }
      }
    }
  }
  CHECK_EQ(nodes.size() % 2, static_cast<size_t>(0));
  std::sort(nodes.begin(), nodes.end());
  for (size_t i = 0; i < nodes.size(); i += 2) {
    scan_intervals->push_back(IntervalOut(nodes[i], nodes[i + 1]));
  }
}

template <typename T>
void PolygonScanCvter<T>::ScansCvt(
    const IntervalIn& scans_interval, const DirectionMajor dir_major,
    const T& step, std::vector<std::vector<IntervalOut>>* scans_intervals) {
  dir_major_ = dir_major;
  op_dir_major_ = OppositeDirection(dir_major_);
  CHECK_GT(step, 0.0);
  step_ = step;
  CHECK_GT(scans_interval.second, scans_interval.first + step);

  bottom_x_ = scans_interval.first;
  double top_x = scans_interval.second;
  scans_size_ = static_cast<size_t>((top_x - bottom_x_) / step_);

  top_segments_.clear();
  top_segments_.reserve(2);

  ParsePolygon(dir_major, true);

  // allocate output data
  scans_intervals->clear();
  scans_intervals->resize(scans_size_);

  BuildEt();
  // initialization aet
  (*scans_intervals)[0].reserve(polygon_.size());
  // add et to aet
  for (const auto& edge : et_[0]) {
    if (std::isfinite(edge.k)) {
      aet_.second.push_back(edge);
    } else {
      (*scans_intervals)[0].push_back(IntervalOut(edge.y, edge.max_y));
    }
  }
  // sort
  std::sort(aet_.second.begin(), aet_.second.end());
  CHECK_EQ(aet_.second.size() & 1, static_cast<size_t>(0));

  // add aet to result
  for (size_t i = 0; i < aet_.second.size(); i += 2) {
    double min_y = aet_.second[i].y;
    double max_y = aet_.second[i + 1].y;
    (*scans_intervals)[0].push_back(IntervalOut(min_y, max_y));
  }
  for (size_t i = 1; i < scans_size_; ++i) {
    UpdateAet(&((*scans_intervals)[i]));
  }

  if (top_segments_.size()) {
    scans_intervals->resize(scans_size_ + 1);
    for (auto& seg : top_segments_) {
      double y1 = seg.first;
      double y2 = seg.second;
      double min_y = std::min(y1, y2);
      double max_y = std::max(y1, y2);
      (*scans_intervals)[scans_size_].push_back(IntervalOut(min_y, max_y));
    }
  }
}

template <typename T>
void PolygonScanCvter<T>::DisturbPolygon(const DirectionMajor dir_major) {
  for (auto& pt : polygon_disturbed_) {
    T& x = pt[dir_major];
    double d_x = (x - bottom_x_) / step_;
    int int_d_x = static_cast<int>(std::round(d_x));
    double delta_x = d_x - int_d_x;
    if (std::abs(delta_x) < s_epsilon_) {
      if (delta_x > 0) {
        x = static_cast<T>((int_d_x + s_epsilon_) * step_ + bottom_x_);
      } else {
        x = static_cast<T>((int_d_x - s_epsilon_) * step_ + bottom_x_);
      }
    }
  }
}

template <typename T>
void PolygonScanCvter<T>::ParsePolygon(const DirectionMajor dir_major,
                                       const bool disturb) {
  polygon_disturbed_ = polygon_;
  if (disturb) {
    DisturbPolygon(dir_major);
  }
  DirectionMajor op_dir_major = OppositeDirection(dir_major);
  size_t vertices_size = polygon_disturbed_.size();
  is_singular_[dir_major].clear();
  is_singular_[dir_major].reserve(vertices_size);
  segments_[dir_major].clear();
  segments_[dir_major].reserve(vertices_size);
  ks_[dir_major].clear();
  ks_[dir_major].reserve(vertices_size);

  auto& is_singular = is_singular_[dir_major];
  auto& segments = segments_[dir_major];
  auto& ks = ks_[dir_major];

  for (size_t i = 0; i < vertices_size; ++i) {
    const Point& pre_vertex =
        polygon_disturbed_[(i + vertices_size - 1) % vertices_size];
    const Point& vertex = polygon_disturbed_[i];
    const Point& nex_vertex = polygon_disturbed_[(i + 1) % vertices_size];
    T pre_x = pre_vertex[dir_major];
    T x = vertex[dir_major];
    T y = vertex[op_dir_major];
    T nex_x = nex_vertex[dir_major];
    T nex_y = nex_vertex[op_dir_major];

    // get segment
    Segment line_seg(vertex, nex_vertex);
    double x_diff = nex_x - x;
    double y_diff = nex_y - y;

    // get k
    segments.push_back(line_seg);
    std::abs(x_diff) < s_epsilon_ ? ks.push_back(s_inf_)
                                  : ks.push_back(y_diff / x_diff);
    double pre_x_diff = pre_x - x;

    // get singular property
    // ensure fill edge
    // case for zero
    if (std::abs(x_diff) < s_epsilon_ || std::abs(pre_x_diff) < s_epsilon_) {
      is_singular.push_back(true);
    } else {
      pre_x_diff* x_diff > 0 ? is_singular.push_back(true)
                             : is_singular.push_back(false);
    }
  }
}

template <typename T>
void PolygonScanCvter<T>::BuildEt() {
  const auto& segments = segments_[dir_major_];
  // allocate memory
  et_.clear();
  et_.resize(scans_size_);
  for (auto& edge_list : et_) {
    edge_list.reserve(polygon_.size());
  }

  // convert segments to edges
  std::vector<std::pair<int, Edge>> edges;
  edges.reserve(segments.size());
  for (size_t i = 0; i < segments.size(); ++i) {
    std::pair<int, Edge> out_edge;
    if (ConvertSegment(i, &(out_edge))) {
      edges.push_back(out_edge);
    }
  }

  // initial active edge table
  aet_.first = 0;
  aet_.second.clear();
  aet_.second.reserve(segments.size());
  for (size_t i = 0; i < edges.size(); ++i) {
    int x_id = edges[i].first;
    const Edge& edge = edges[i].second;
    if (x_id >= static_cast<int>(scans_size_)) {
      continue;
    }
    // add x_id == 0 to act
    if (x_id >= 0) {
      et_[x_id].push_back(edge);
    } else {
      // check if intersect at x_id = 0
      Edge active_edge = edge;
      if (active_edge.MoveUp(0.0 - active_edge.x)) {
        aet_.second.push_back(active_edge);
      }
    }
  }
}

template <typename T>
void PolygonScanCvter<T>::UpdateAet(std::vector<IntervalOut>* scan_intervals) {
  size_t x_id = aet_.first + 1;
  CHECK_LT(x_id, et_.size());
  aet_.first += 1;
  scan_intervals->clear();
  scan_intervals->reserve(polygon_.size());

  // check
  size_t valid_edges_num = aet_.second.size();
  size_t invalid_edges_num = 0;
  for (auto& edge : aet_.second) {
    if (!edge.MoveUp(step_)) {
      --valid_edges_num;
      ++invalid_edges_num;
      edge.y = s_inf_;
    }
  }

  // add et to aet
  size_t new_edges_num = 0;
  for (const auto& edge : et_[x_id]) {
    if (std::isfinite(edge.k)) {
      ++valid_edges_num;
      ++new_edges_num;
      aet_.second.push_back(edge);
    } else {
      scan_intervals->push_back(IntervalOut(edge.y, edge.max_y));
    }
  }
  CHECK_EQ(valid_edges_num & 1, static_cast<size_t>(0))
      << boost::format(
             "valid edges num: %d x: %lf bottom_x: %lf \n vertices num: %d "
             "\n") %
             valid_edges_num % (x_id * step_ + bottom_x_) % bottom_x_ %
             polygon_.size()
      << aet_.second[0] << "\n"
      << aet_.second[1] << "\n"
      << aet_.second[2] << "\n"
      << aet_.second[3];

  // sort
  if (invalid_edges_num != 0 || new_edges_num != 0) {
    std::sort(aet_.second.begin(), aet_.second.end());
    // remove invalid edges
    aet_.second.resize(valid_edges_num);
  }
  for (size_t i = 0; i < aet_.second.size(); i += 2) {
    // corner case, same point in the pre scan and complex polygon
    double min_y = aet_.second[i].y;
    double max_y = aet_.second[i + 1].y;
    scan_intervals->push_back(IntervalOut(min_y, max_y));
  }
}

template <typename T>
bool PolygonScanCvter<T>::ConvertSegment(const size_t seg_id,
                                         std::pair<int, Edge>* out_edge) {
  const auto& segments = segments_[dir_major_];
  const auto& ks = ks_[dir_major_];

  CHECK_LT(seg_id, segments.size());
  Segment segment = segments[seg_id];
  double k = ks[seg_id];
  const Point& low_vertex = segment.first;
  const Point& high_vertex = segment.second;
  if (low_vertex[dir_major_] > high_vertex[dir_major_]) {
    std::swap(segment.first, segment.second);
  }
  // return pair of int id and edge
  Edge& edge = out_edge->second;
  int& x_id = out_edge->first;
  const Point& min_vertex = segment.first;
  double min_x = min_vertex[dir_major_] - bottom_x_;
  double min_y = min_vertex[op_dir_major_];
  x_id = static_cast<int>(std::ceil(min_x / step_));
  double min_x_ceil = x_id * step_;

  edge.x = min_x_ceil;
  edge.min_x = edge.x;
  edge.max_x = segment.second[dir_major_] - bottom_x_;
  edge.max_y = segment.second[op_dir_major_];
  edge.k = k;
  // handle special edges
  if (std::isfinite(edge.k)) {
    edge.y = min_y + (edge.x - min_x) * edge.k;
  } else {
    edge.y = min_y;
    // swap
    if (edge.y > edge.max_y) {
      std::swap(edge.y, edge.max_y);
    }
  }
  edge.min_y = edge.y;

  // save top edge
  if (static_cast<size_t>(x_id) >= scans_size_) {
    std::pair<double, double> seg(low_vertex[op_dir_major_],
                                  high_vertex[op_dir_major_]);
    top_segments_.push_back(seg);
  }
  // check edge valid, for length < step
  if (std::isfinite(edge.k) && edge.max_x < edge.x) {
    return false;
  }
  return true;
}

}  // namespace lidar
}  // namespace perception
}  // namespace apollo
