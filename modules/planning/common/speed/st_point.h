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
 * @file st_point.h
 **/

#pragma once

#include <string>

#include "modules/common/math/vec2d.h"

namespace apollo {
namespace planning {

class STPoint : public common::math::Vec2d {
  // x-axis: t; y-axis: s.
 public:
  STPoint() = default;
  STPoint(const double s, const double t);
  explicit STPoint(const common::math::Vec2d& vec2d_point);

  double x() const = delete;
  double y() const = delete;

  double s() const;
  double t() const;
  void set_s(const double s);
  void set_t(const double t);
  std::string DebugString() const;
};

}  // namespace planning
}  // namespace apollo
