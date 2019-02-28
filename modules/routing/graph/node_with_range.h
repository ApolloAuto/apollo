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

#pragma once

#include <string>

#include "modules/routing/graph/topo_node.h"

namespace apollo {
namespace routing {

class NodeWithRange : public NodeSRange {
 public:
  NodeWithRange(const NodeWithRange& other) = default;
  NodeWithRange(const TopoNode* node, double start_s, double end_s);
  NodeWithRange(const TopoNode* node, const NodeSRange& range);
  virtual ~NodeWithRange();
  bool operator<(const NodeWithRange& other) const;

  const TopoNode* GetTopoNode() const;
  bool IsVirtual() const;
  const std::string& RoadId() const;
  const std::string& LaneId() const;
  double FullLength() const;

 private:
  const TopoNode* topo_node_ = nullptr;
};

}  // namespace routing
}  // namespace apollo
