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
 * @file frame.cc
 **/

#include "modules/planning/common/frame.h"

#include "modules/common/log.h"

namespace apollo {
namespace planning {

Frame::Frame(uint32_t sequence_num) : _sequence_num(sequence_num) {}

void Frame::set_sequence_num(const uint32_t sequence_num) {
  _sequence_num = sequence_num;
}

void Frame::set_environment(const Environment &environment) {
  _environment = environment;
}

void Frame::set_planning_data(PlanningData *const planning_data) {
  _planning_data.reset(planning_data);
}

uint32_t Frame::sequence_num() const { return _sequence_num; }

const Environment &Frame::environment() const { return _environment; }

const PlanningData &Frame::planning_data() const {
  CHECK_NOTNULL(_planning_data.get());
  return *(_planning_data.get());
}

PlanningData *Frame::mutable_planning_data() {
  CHECK_NOTNULL(_planning_data.get());
  return _planning_data.get();
}

std::string Frame::DebugString() const {
  return std::to_string(_sequence_num);
}

}  // namespace planning
}  // namespace apollo
