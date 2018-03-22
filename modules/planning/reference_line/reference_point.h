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
 * @file reference_point.h
 **/

#ifndef MODULES_PLANNING_REFERENCE_LINE_REFERENCE_POINT_H_
#define MODULES_PLANNING_REFERENCE_LINE_REFERENCE_POINT_H_

#include <string>
#include <vector>

#include "modules/common/proto/pnc_point.pb.h"

#include "modules/map/pnc_map/path.h"

namespace apollo {
namespace planning {

class ReferencePoint : public hdmap::MapPathPoint {
 public:
  ReferencePoint() = default;

  ReferencePoint(const MapPathPoint& map_path_point, const double kappa,
                 const double dkappa);

  common::PathPoint ToPathPoint(double s) const;

  double kappa() const;
  double dkappa() const;

  std::string DebugString() const;

  static void RemoveDuplicates(std::vector<ReferencePoint>* points);

 private:
  double kappa_ = 0.0;
  double dkappa_ = 0.0;
};

}  // namespace planning
}  // namespace apollo

#endif  // MODULES_PLANNING_REFERENCE_LINE_REFERENCE_POINT_H_
