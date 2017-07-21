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

/**
 * @file sgraph_edge.h
 **/

#ifndef MODULES_PLANNING_OPTIMIZER_DP_POLY_PATH_GRAPH_EDGE_H
#define MODULES_PLANNING_OPTIMIZER_DP_POLY_PATH_GRAPH_EDGE_H

#include <vector>

#include "modules/planning/math/curve1d/quintic_polynomial_curve1d.h"
#include "modules/common/proto/path_point.pb.h"

namespace apollo {
namespace planning {

class GraphEdge {
 public:
  explicit GraphEdge();
  explicit GraphEdge(const size_t from_vertex, const size_t to_vertex, const size_t level);
  ~GraphEdge() = default;

  void set_from_vertex(const size_t index);
  void set_to_vertex(const size_t index);
  void set_level(const size_t level);
  void set_path(const std::vector<common::FrenetFramePoint> &path);
  void set_poly_path(const QuinticPolynomialCurve1d &ploy_path);
  void set_edge_index(const size_t index);
  void set_cost(const double cost);

  size_t from_vertex() const;
  size_t to_vertex() const;
  size_t level() const;
  const std::vector<common::FrenetFramePoint> &path() const;
  const QuinticPolynomialCurve1d &poly_path() const;
  size_t edge_index() const;
  double cost() const;

 private:
  size_t _edge_index;
  size_t _from_vertex;
  size_t _to_vertex;
  size_t _level;
  double _cost;
  std::vector<common::FrenetFramePoint> _path;
  QuinticPolynomialCurve1d _poly_path;
};

}  // planning
}  // apollo

#endif  // MODULES_PLANNING_OPTIMIZER_DP_POLY_PATH_GRAPH_EDGE_H
