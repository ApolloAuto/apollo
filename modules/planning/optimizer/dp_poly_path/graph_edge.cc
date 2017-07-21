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
 * @file graph_edge.cpp
 **/

#include <cstdio>
#include "modules/planning/optimizer/dp_poly_path/graph_edge.h"

namespace apollo {
namespace planning {

GraphEdge::GraphEdge() : GraphEdge(0, 0, 0) {}

GraphEdge::GraphEdge(const size_t from_vertex, const size_t to_vertex,
                     const size_t level)
    : _edge_index(0),
      _from_vertex(from_vertex),
      _to_vertex(to_vertex),
      _level(level),
      _cost(0.0) {}

void GraphEdge::set_from_vertex(const size_t from_vertex) {
  _from_vertex = from_vertex;
}

void GraphEdge::set_to_vertex(const size_t to_vertex) {
  _to_vertex = to_vertex;
}

void GraphEdge::set_level(const size_t level) { _level = level; }

void GraphEdge::set_path(
    const std::vector<common::FrenetFramePoint> &path) {
  _path = path;
}

void GraphEdge::set_poly_path(const QuinticPolynomialCurve1d &ploy_path) {
  _poly_path = ploy_path;
}

void GraphEdge::set_edge_index(const size_t index) { _edge_index = index; }

void GraphEdge::set_cost(const double cost) { _cost = cost; }

size_t GraphEdge::from_vertex() const { return _from_vertex; }

size_t GraphEdge::to_vertex() const { return _to_vertex; }

size_t GraphEdge::level() const { return _level; }

const std::vector<common::FrenetFramePoint> &GraphEdge::path() const {
  return _path;
}

const QuinticPolynomialCurve1d &GraphEdge::poly_path() const {
  return _poly_path;
}

size_t GraphEdge::edge_index() const { return _edge_index; }

double GraphEdge::cost() const { return _cost; }

}  // namespace planning
}  // namespace apollo
