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

#include "modules/routing/graph/topo_range.h"

#include <algorithm>

#include "modules/routing/common/routing_gflags.h"

namespace apollo {
namespace routing {

bool NodeSRange::IsEnoughForChangeLane(double start_s, double end_s) {
  return IsEnoughForChangeLane(end_s - start_s);
}

bool NodeSRange::IsEnoughForChangeLane(double length) {
  return (length > FLAGS_min_length_for_lane_change);
}

NodeSRange::NodeSRange(double s1, double s2) : start_s_(s1), end_s_(s2) {}

bool NodeSRange::operator<(const NodeSRange& other) const {
  return StartS() < other.StartS();
}

bool NodeSRange::IsValid() const { return start_s_ <= end_s_; }

double NodeSRange::StartS() const { return start_s_; }

double NodeSRange::EndS() const { return end_s_; }

double NodeSRange::Length() const { return end_s_ - start_s_; }

bool NodeSRange::IsEnoughForChangeLane() const {
  return NodeSRange::IsEnoughForChangeLane(StartS(), EndS());
}

void NodeSRange::SetStartS(double start_s) { start_s_ = start_s; }

void NodeSRange::SetEndS(double end_s) { end_s_ = end_s; }

bool NodeSRange::MergeRangeOverlap(const NodeSRange& other) {
  if (!IsValid() || !other.IsValid()) {
    return false;
  }
  if (other.StartS() > EndS() || other.EndS() < StartS()) {
    return false;
  }
  SetEndS(std::max(EndS(), other.EndS()));
  SetStartS(std::min(StartS(), other.StartS()));
  return true;
}

}  // namespace routing
}  // namespace apollo
