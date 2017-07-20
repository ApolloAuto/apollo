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
 * @file optimizer.h
 **/

#ifndef MODULES_PLANNING_OPTIMIZER_OPTIMIZER_H_
#define MODULES_PLANNING_OPTIMIZER_OPTIMIZER_H_

#include <string>

#include "modules/common/proto/error_code.pb.h"

namespace apollo {
namespace planning {

class Optimizer {
 public:
  explicit Optimizer(const std::string& name);
  virtual ~Optimizer() = default;
  virtual const std::string& name() const;

 private:
  const std::string _name;
};

}  // namespace planning
}  // namespace apollo

#endif  // MODULES_PLANNING_OPTIMIZER_OPTIMIZER_H_
