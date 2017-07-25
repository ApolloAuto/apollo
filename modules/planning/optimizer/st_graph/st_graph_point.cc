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
 * @file: st_graph_point.cc
 **/

#include "modules/planning/optimizer/st_graph/st_graph_point.h"

#include "modules/planning/common/planning_gflags.h"

namespace apollo {
namespace planning {


std::uint32_t STGraphPoint::index_s() const { return _index_s; }

std::uint32_t STGraphPoint::index_t() const { return _index_t; }

const STPoint& STGraphPoint::point() const { return _point; }

const STGraphPoint* STGraphPoint::pre_point() const { return _pre_point; }

double STGraphPoint::reference_cost() const { return _reference_cost; }

double STGraphPoint::obstacle_cost() const { return _obstacle_cost; }

double STGraphPoint::total_cost() const { return _total_cost; }

void STGraphPoint::init(const std::uint32_t index_t, const std::uint32_t index_s,
                        const STPoint& st_point) {
  _index_t = index_t;
  _index_s = index_s;
  _point = st_point;
}

void STGraphPoint::set_reference_cost(const double reference_cost) {
  _reference_cost = reference_cost;
}

void STGraphPoint::set_obstacle_cost(const double obs_cost) {
  _obstacle_cost = obs_cost;
}

void STGraphPoint::set_total_cost(const double total_cost) {
  _total_cost = total_cost;
}

void STGraphPoint::set_pre_point(const STGraphPoint& pre_point) {
  _pre_point = &pre_point;
}

STPoint* STGraphPoint::mutable_point() { return &_point; }

}  // namespace planning
}  // namespace apollo
