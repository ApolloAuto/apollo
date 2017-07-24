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
 * @file sgraph_vertex.cpp
 **/

#include <limits>
#include <sstream>
#include <string>
#include "modules/planning/optimizer/dp_poly_path/graph_vertex.h"

namespace apollo {
namespace planning {

GraphVertex::GraphVertex(const common::FrenetFramePoint &frame_point,
                         const uint32_t index, const uint32_t level) :
    _frame_point(frame_point),
    _index(index),
    _level(level),
    _type(Type::REGULAR),
    _accumulated_cost(std::numeric_limits<double>::max()) {
}

const common::FrenetFramePoint &GraphVertex::frame_point() const {
  return _frame_point;
}

common::FrenetFramePoint *GraphVertex::mutable_frame_point() {
  return &_frame_point;
}

void GraphVertex::set_type(const Type type) {
  _type = type;
}

void GraphVertex::set_accumulated_cost(const double accumulated_cost) {
  _accumulated_cost = accumulated_cost;
}

void GraphVertex::set_index(const uint32_t index) {
  _index = index;
}

void GraphVertex::set_level(const uint32_t level) {
  _level = level;
}

bool GraphVertex::is_graph_head() const {
  return _type == Type::GRAPH_HEAD;
}

bool GraphVertex::is_dead_end() const {
  return _type == Type::DEAD_END;
}

double GraphVertex::accumulated_cost() const {
  return _accumulated_cost;
}

uint32_t GraphVertex::index() const {
  return _index;
}

uint32_t GraphVertex::level() const {
  return _level;
}

void GraphVertex::add_out_vertex(const uint32_t vertex_index) {
  _vertices_out.push_back(vertex_index);
}

void GraphVertex::add_out_vertex(const std::vector<uint32_t> &vertices) {
  _vertices_out.insert(_vertices_out.end(), vertices.begin(), vertices.end());
}

void GraphVertex::add_in_vertex(const uint32_t vertex_index) {
  _vertices_in.push_back(vertex_index);
}

void GraphVertex::add_in_vertex(const std::vector<uint32_t> &vertices) {
  _vertices_in.insert(_vertices_in.end(), vertices.begin(), vertices.end());
}

void GraphVertex::add_out_edge(const uint32_t edge_index) {
  _edges_out.push_back(edge_index);
}

void GraphVertex::add_out_edge(const std::vector<uint32_t> &edges) {
  _edges_out.insert(_edges_out.end(), edges.begin(), edges.end());
}

void GraphVertex::add_in_edge(const uint32_t edge_index) {
  _edges_in.push_back(edge_index);
}

void GraphVertex::add_in_edge(const std::vector<uint32_t> &edges) {
  _edges_in.insert(_edges_in.end(), edges.begin(), edges.end());
}

const std::vector<uint32_t> &GraphVertex::vertices_out() const {
  return _vertices_out;
}

const std::vector<uint32_t> &GraphVertex::vertices_in() const {
  return _vertices_in;
}

const std::vector<uint32_t> &GraphVertex::edges_out() const {
  return _edges_out;
}

const std::vector<uint32_t> &GraphVertex::edges_in() const {
  return _edges_in;
}

std::string GraphVertex::to_string() const {
  std::ostringstream sout;
  sout << "index = " << _index << "; accumulated_cost = " << _accumulated_cost
       << "s = " << _frame_point.s() << "; l = " << _frame_point.l()
       << "; is_dead_end = " << std::boolalpha << is_dead_end() << std::endl;
  for (const auto &v : _vertices_out) {
    sout << "_vertices_out[" << v << "] ";
  }
  sout.flush();
  return sout.str();
}

}  // namespace planning
}  // namespace apollo
