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
 * @file decision.cc
 **/

#include "modules/planning/common/decision.h"

#include <sstream>

namespace apollo {
namespace planning {

Decision::Decision(const double& buffer, const DecisionType type)
    : _buffer(buffer), _decision_type(type) {}

std::string Decision::to_json() const {
  std::string type_str = "";
  switch (_decision_type) {
    case DecisionType::GO_LEFT: {
      type_str = "GO_LEFT";
      break;
    }
    case DecisionType::GO_RIGHT: {
      type_str = "GO_RIGHT";
      break;
    }
    case DecisionType::GO_UP: {
      type_str = "GO_UP";
      break;
    }
    case DecisionType::YIELD_DOWN: {
      type_str = "YIELD_DOWN";
      break;
    }
    case DecisionType::STOP_DOWN: {
      type_str = "STOP_DOWN";
      break;
    }
    case DecisionType::FOLLOW_DOWN: {
      type_str = "FOLLOW_DOWN";
      break;
    }
    default:
      type_str = "NO_TYPE";
  }
  std::ostringstream sout;
  sout << "{ \"decision type\" : \"" << type_str
       << "\" , \"buffer\" : " << _buffer << " }";
  sout.flush();
  return sout.str();
}

double Decision::buffer() const { return _buffer; }

Decision::DecisionType Decision::decision_type() const {
  return _decision_type;
}

}  // namespace planning
}  // namespace apollo
