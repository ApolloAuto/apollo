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

#include "cyber/service_discovery/container/graph.h"

#include <queue>

namespace apollo {
namespace cyber {
namespace service_discovery {

using base::AtomicRWLock;
using base::ReadLockGuard;
using base::WriteLockGuard;

Vertice::Vertice(const std::string& val) : value_(val) {}

Vertice::Vertice(const Vertice& other) { this->value_ = other.value_; }

Vertice::~Vertice() {}

Vertice& Vertice::operator=(const Vertice& rhs) {
  if (this != &rhs) {
    this->value_ = rhs.value_;
  }
  return *this;
}

bool Vertice::operator==(const Vertice& rhs) const {
  return this->value_ == rhs.value_;
}

bool Vertice::operator!=(const Vertice& rhs) const {
  return this->value_ != rhs.value_;
}

bool Vertice::IsDummy() const { return value_.empty(); }

const std::string& Vertice::GetKey() const { return value_; }

Edge::Edge() {}

Edge::Edge(const Edge& other) {
  this->src_ = other.src_;
  this->dst_ = other.dst_;
  this->value_ = other.value_;
}

Edge::Edge(const Vertice& src, const Vertice& dst, const std::string& val)
    : src_(src), dst_(dst), value_(val) {}

Edge::~Edge() {}

Edge& Edge::operator=(const Edge& rhs) {
  if (this != &rhs) {
    this->src_ = rhs.src_;
    this->dst_ = rhs.dst_;
    this->value_ = rhs.value_;
  }
  return *this;
}

bool Edge::operator==(const Edge& rhs) const {
  return this->src_ == rhs.src_ && this->dst_ == rhs.dst_ &&
         this->value_ == rhs.value_;
}

bool Edge::IsValid() const {
  if (value_.empty()) {
    return false;
  }
  if (src_.IsDummy() && dst_.IsDummy()) {
    return false;
  }
  return true;
}

std::string Edge::GetKey() const { return value_ + "_" + dst_.GetKey(); }

Graph::Graph() {}

Graph::~Graph() {
  edges_.clear();
  list_.clear();
}

void Graph::Insert(const Edge& e) {
  if (!e.IsValid()) {
    return;
  }
  WriteLockGuard<AtomicRWLock> lock(rw_lock_);
  auto& e_v = e.value();
  if (edges_.find(e_v) == edges_.end()) {
    edges_[e_v] = RelatedVertices();
  }

  if (!e.src().IsDummy()) {
    InsertOutgoingEdge(e);
  }
  if (!e.dst().IsDummy()) {
    InsertIncomingEdge(e);
  }
}

void Graph::Delete(const Edge& e) {
  if (!e.IsValid()) {
    return;
  }
  WriteLockGuard<AtomicRWLock> lock(rw_lock_);
  auto& e_v = e.value();
  if (edges_.find(e_v) == edges_.end()) {
    return;
  }

  if (!e.src().IsDummy()) {
    DeleteOutgoingEdge(e);
  }
  if (!e.dst().IsDummy()) {
    DeleteIncomingEdge(e);
  }
}

uint32_t Graph::GetNumOfEdge() {
  ReadLockGuard<AtomicRWLock> lock(rw_lock_);
  uint32_t num = 0;
  for (auto& item : list_) {
    num += static_cast<int>(item.second.size());
  }
  return num;
}

FlowDirection Graph::GetDirectionOf(const Vertice& lhs, const Vertice& rhs) {
  if (lhs.IsDummy() || rhs.IsDummy()) {
    return UNREACHABLE;
  }
  ReadLockGuard<AtomicRWLock> lock(rw_lock_);
  if (list_.count(lhs.GetKey()) == 0 || list_.count(rhs.GetKey()) == 0) {
    return UNREACHABLE;
  }
  if (LevelTraverse(lhs, rhs)) {
    return UPSTREAM;
  }
  if (LevelTraverse(rhs, lhs)) {
    return DOWNSTREAM;
  }
  return UNREACHABLE;
}

void Graph::InsertOutgoingEdge(const Edge& e) {
  auto& e_v = e.value();
  auto& src_v_set = edges_[e_v].src;
  auto& src_v = e.src();
  auto& v_k = src_v.GetKey();
  if (src_v_set.find(v_k) != src_v_set.end()) {
    return;
  }
  src_v_set[v_k] = src_v;

  auto& dst_v_set = edges_[e_v].dst;
  Edge insert_e;
  insert_e.set_src(src_v);
  insert_e.set_value(e.value());
  for (auto& item : dst_v_set) {
    insert_e.set_dst(item.second);
    InsertCompleteEdge(insert_e);
  }
}

void Graph::InsertIncomingEdge(const Edge& e) {
  auto& e_v = e.value();
  auto& dst_v_set = edges_[e_v].dst;
  auto& dst_v = e.dst();
  auto& v_k = dst_v.GetKey();
  if (dst_v_set.find(v_k) != dst_v_set.end()) {
    return;
  }
  dst_v_set[v_k] = dst_v;

  auto& src_v_set = edges_[e_v].src;
  Edge insert_e;
  insert_e.set_dst(dst_v);
  insert_e.set_value(e.value());
  for (auto& item : src_v_set) {
    insert_e.set_src(item.second);
    InsertCompleteEdge(insert_e);
  }
}

void Graph::InsertCompleteEdge(const Edge& e) {
  auto& src_v_k = e.src().GetKey();
  if (list_.find(src_v_k) == list_.end()) {
    list_[src_v_k] = VerticeSet();
  }
  auto& dst_v_k = e.dst().GetKey();
  if (list_.find(dst_v_k) == list_.end()) {
    list_[dst_v_k] = VerticeSet();
  }
  list_[src_v_k][e.GetKey()] = e.dst();
}

void Graph::DeleteOutgoingEdge(const Edge& e) {
  auto& e_v = e.value();
  auto& src_v_set = edges_[e_v].src;
  auto& src_v = e.src();
  auto& v_k = src_v.GetKey();
  if (src_v_set.find(v_k) == src_v_set.end()) {
    return;
  }
  src_v_set.erase(v_k);

  auto& dst_v_set = edges_[e_v].dst;
  Edge delete_e;
  delete_e.set_src(src_v);
  delete_e.set_value(e.value());
  for (auto& item : dst_v_set) {
    delete_e.set_dst(item.second);
    DeleteCompleteEdge(delete_e);
  }
}

void Graph::DeleteIncomingEdge(const Edge& e) {
  auto& e_v = e.value();
  auto& dst_v_set = edges_[e_v].dst;
  auto& dst_v = e.dst();
  auto& v_k = dst_v.GetKey();
  if (dst_v_set.find(v_k) == dst_v_set.end()) {
    return;
  }
  dst_v_set.erase(v_k);

  auto& src_v_set = edges_[e_v].src;
  Edge delete_e;
  delete_e.set_dst(dst_v);
  delete_e.set_value(e.value());
  for (auto& item : src_v_set) {
    delete_e.set_src(item.second);
    DeleteCompleteEdge(delete_e);
  }
}

void Graph::DeleteCompleteEdge(const Edge& e) {
  auto& src_v_k = e.src().GetKey();
  list_[src_v_k].erase(e.GetKey());
}

bool Graph::LevelTraverse(const Vertice& start, const Vertice& end) {
  std::unordered_map<std::string, bool> visited;
  visited[end.GetKey()] = false;
  std::queue<Vertice> unvisited;
  unvisited.emplace(start);
  while (!unvisited.empty()) {
    auto curr = unvisited.front();
    unvisited.pop();
    if (visited[curr.GetKey()]) {
      continue;
    }
    visited[curr.GetKey()] = true;
    if (curr == end) {
      break;
    }
    for (auto& item : list_[curr.GetKey()]) {
      unvisited.push(item.second);
    }
  }
  return visited[end.GetKey()];
}

}  // namespace service_discovery
}  // namespace cyber
}  // namespace apollo
