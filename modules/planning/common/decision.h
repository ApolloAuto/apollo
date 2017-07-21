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
 * @file decision.h
 **/

#ifndef MODULES_PLANNING_COMMON_DECISION_H_
#define MODULES_PLANNING_COMMON_DECISION_H_

#include <string>

namespace apollo {
namespace planning {

class Decision {
 public:
  enum class DecisionType {
    GO_LEFT = 0,
    GO_RIGHT = 1,
    GO_UP = 2,
    FOLLOW_DOWN = 3,
    STOP_DOWN = 4,
    YIELD_DOWN = 5,
  };

  Decision(const double& buffer, const DecisionType type);
  double buffer() const;
  DecisionType decision_type() const;
  virtual std::string to_json() const;

 private:
  double _buffer = 0.0;
  DecisionType _decision_type;
};

}  // namespace planning
}  // namespace apollo

#endif  // MODULES_PLANNING_COMMON_DECISION_H_
