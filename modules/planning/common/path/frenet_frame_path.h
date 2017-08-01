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
 * @file frenet_frame_path.h
 **/

#ifndef BAIDU_ADU_HOUSTON_COMMON_PATH_FRENET_FRAME_PATH_H_
#define BAIDU_ADU_HOUSTON_COMMON_PATH_FRENET_FRAME_PATH_H_

#include <vector>

#include "modules/common/proto/path_point.pb.h"

namespace apollo {
namespace planning {

class FrenetFramePath {
 public:
  FrenetFramePath() = default;
  explicit FrenetFramePath(std::vector<common::FrenetFramePoint> sl_points);
  virtual ~FrenetFramePath() = default;

  void set_points(const std::vector<common::FrenetFramePoint> &points);
  const std::vector<common::FrenetFramePoint> &points() const;
  std::uint32_t number_of_points() const;
  double length() const;
  const common::FrenetFramePoint &point_at(const std::uint32_t index) const;
  common::FrenetFramePoint &point_at(const std::uint32_t index);
  const common::FrenetFramePoint& interpolate(double s) const;

 private:
  std::vector<common::FrenetFramePoint> points_;
};

}  // namespace planning
}  // namespace apollo

#endif /* BAIDU_ADU_HOUSTON_COMMON_PATH_FRENET_FRAME_PATH_H_ */
