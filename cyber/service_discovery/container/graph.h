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

#ifndef CYBER_SERVICE_DISCOVERY_CONTAINER_GRAPH_H_
#define CYBER_SERVICE_DISCOVERY_CONTAINER_GRAPH_H_

#include <cstdint>
#include <string>
#include <unordered_map>

#include "cyber/base/atomic_rw_lock.h"

namespace apollo {
namespace cyber {
namespace service_discovery {

/**
 * @brief describe the flow direction between nodes
 * As the DAG below
 * A---->B----->C<-----D
 * GetDirectionOf(A, B) is UPSTREAM
 * GetDirectionOf(C, A) is DOWNSTREAM
 * GetDirectionOf(D, A) is UNREACHABLE
 * GetDirectionOf(A, D) is UNREACHABLE
 */
enum FlowDirection {
  UNREACHABLE,
  UPSTREAM,
  DOWNSTREAM,
};

// opt impl of Vertice/Edge/Graph, replace stl with base
class Vertice {
 public:
  explicit Vertice(const std::string& val = "");
  Vertice(const Vertice& other);
  virtual ~Vertice();

  Vertice& operator=(const Vertice& rhs);
  bool operator==(const Vertice& rhs) const;
  bool operator!=(const Vertice& rhs) const;

  bool IsDummy() const;
  const std::string& GetKey() const;

  const std::string& value() const { return value_; }

 private:
  std::string value_;
};

class Edge {
 public:
  Edge();
  Edge(const Edge& other);
  Edge(const Vertice& src, const Vertice& dst, const std::string& val);
  virtual ~Edge();

  Edge& operator=(const Edge& rhs);
  bool operator==(const Edge& rhs) const;

  bool IsValid() const;
  std::string GetKey() const;

  const Vertice& src() const { return src_; }
  void set_src(const Vertice& v) { src_ = v; }

  const Vertice& dst() const { return dst_; }
  void set_dst(const Vertice& v) { dst_ = v; }

  const std::string& value() const { return value_; }
  void set_value(const std::string& val) { value_ = val; }

 private:
  Vertice src_;
  Vertice dst_;
  std::string value_;
};

class Graph {
 public:
  using VerticeSet = std::unordered_map<std::string, Vertice>;
  using AdjacencyList = std::unordered_map<std::string, VerticeSet>;

  Graph();
  virtual ~Graph();

  void Insert(const Edge& e);
  void Delete(const Edge& e);

  uint32_t GetNumOfEdge();
  FlowDirection GetDirectionOf(const Vertice& lhs, const Vertice& rhs);

 private:
  struct RelatedVertices {
    RelatedVertices() {}

    VerticeSet src;
    VerticeSet dst;
  };
  using EdgeInfo = std::unordered_map<std::string, RelatedVertices>;

  void InsertOutgoingEdge(const Edge& e);
  void InsertIncomingEdge(const Edge& e);
  void InsertCompleteEdge(const Edge& e);
  void DeleteOutgoingEdge(const Edge& e);
  void DeleteIncomingEdge(const Edge& e);
  void DeleteCompleteEdge(const Edge& e);
  bool LevelTraverse(const Vertice& start, const Vertice& end);

  EdgeInfo edges_;
  AdjacencyList list_;
  base::AtomicRWLock rw_lock_;
};

}  // namespace service_discovery
}  // namespace cyber
}  // namespace apollo

#endif  // CYBER_SERVICE_DISCOVERY_CONTAINER_GRAPH_H_
