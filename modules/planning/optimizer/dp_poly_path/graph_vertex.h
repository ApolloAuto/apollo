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
 * @file graph_point.h
 **/

#ifndef MODULES_PLANNING_OPTIMIZER_DP_POLY_PATH_GRAPH_VERTEX_H
#define MODULES_PLANNING_OPTIMIZER_DP_POLY_PATH_GRAPH_VERTEX_H

#include <vector>
#include <string>
#include "modules/common/proto/path_point.pb.h"

namespace apollo {
namespace planning {

class GraphVertex {
 public:
  enum class Type {
    GRAPH_HEAD,
    REGULAR,
    DEAD_END
  };

  explicit GraphVertex(const common::FrenetFramePoint &frame_point,
                       const uint32_t index, const uint32_t level);
  ~GraphVertex() = default;

  const common::FrenetFramePoint &frame_point() const;
  common::FrenetFramePoint *mutable_frame_point();

  void set_type(const Type type);
  void set_index(const uint32_t index);
  void set_level(const uint32_t level);
  void set_accumulated_cost(const double accumulated_cost);

  bool is_graph_head() const;
  bool is_dead_end() const;
  double accumulated_cost() const;
  uint32_t index() const;
  uint32_t level() const;

  const std::vector<uint32_t> &vertices_out() const;
  const std::vector<uint32_t> &vertices_in() const;
  const std::vector<uint32_t> &edges_out() const;
  const std::vector<uint32_t> &edges_in() const;

  std::string to_string() const;

 public:
  void add_in_vertex(const std::vector<uint32_t> &vertices);
  void add_in_vertex(const uint32_t vertex_index);
  void add_out_vertex(const uint32_t vertex_index);
  void add_out_vertex(const std::vector<uint32_t> &vertices);
  void add_in_edge(const uint32_t edge_index);
  void add_in_edge(const std::vector<uint32_t> &edges);
  void add_out_edge(const uint32_t edge_index);
  void add_out_edge(const std::vector<uint32_t> &edges);

 private:
  common::FrenetFramePoint _frame_point;
  uint32_t _index = 0;
  uint32_t _level = 0;
  Type _type = Type::REGULAR;

  std::vector<uint32_t> _vertices_in;
  std::vector<uint32_t> _vertices_out;
  std::vector<uint32_t> _edges_in;
  std::vector<uint32_t> _edges_out;

  double _accumulated_cost = 0.0;
};

}  // namespace planning
}  // namespace apollo

#endif  // MODULES_PLANNING_OPTIMIZER_DP_POLY_PATH_GRAPH_VERTEX_H
