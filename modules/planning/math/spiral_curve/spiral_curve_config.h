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
 * @file: spiral_curve_config.h
 * @brief: header file for path generating parameters
 **/

#ifndef MODULES_PLANNING_SPIRAL_CURVE_SPIRAL_CURVE_CONFIG_H_
#define MODULES_PLANNING_SPIRAL_CURVE_SPIRAL_CURVE_CONFIG_H_

#include "common/json_object.h"

#include <utility>

namespace apollo {
namespace planning {

class SpiralCurveConfig : public JsonObject {
 public:
  SpiralCurveConfig() = default;
  SpiralCurveConfig(const std::size_t simpson_size,
                    const double newton_raphson_tol,
                    const std::size_t newton_raphson_max_iter);
  std::size_t simpson_size() const;
  std::size_t newton_raphson_max_iter() const;
  double newton_raphson_tol() const;
  virtual std::string to_json() const;

 private:
  std::size_t simpson_size_ = 9;
  double newton_raphson_tol_ = 0.01;
  std::size_t newton_raphson_max_iter_ = 20;
};

}  // namespace planning
}  // namespace apollo

#endif  // MODULES_PLANNING_SPIRAL_CURVE_SPIRAL_CURVE_CONFIG_H_
