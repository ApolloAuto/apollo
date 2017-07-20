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
 * @file reference_line_decider.h
 **/

#ifndef MODULES_PLANNING_REFERENCE_LINE_REFERENCE_LINE_DECIDER_H_
#define MODULES_PLANNING_REFERENCE_LINE_REFERENCE_LINE_DECIDER_H_

#include <list>
#include <vector>

#include "modules/common/proto/error_code.pb.h"
#include "modules/map/proto/routing.pb.h"

#include "modules/planning/common/data_center.h"
#include "modules/planning/common/environment.h"
#include "modules/planning/reference_line/reference_line.h"

namespace apollo {
namespace planning {

class ReferenceLineDecider {
 public:
  ReferenceLineDecider();
  apollo::common::ErrorCode init(const DataCenter& data_center);
  bool has_next() const;
  std::unique_ptr<ReferenceLine> next_reference_line();
  std::size_t num_of_reference_lines() const;

 private:
  apollo::common::ErrorCode build_reference_lines(
      const DataCenter& data_center,
      const apollo::hdmap::RoutingResult& routing);

  double _last_route_timestamp = 0.0;
  int64_t _last_route_sequence_num = -1;
  std::size_t _current_route_index = 0;
  double _current_s = 0.0;
  std::vector<ReferenceLine> _route_reference_lines;
  std::list<std::unique_ptr<ReferenceLine>> _reference_lines;
  std::list<std::unique_ptr<ReferenceLine>>::iterator _it;
};

}  // namespace planning
}  // namespace apollo

#endif  // MODULES_PLANNING_REFERENCE_LINE_REFERENCE_LINE_DECIDER_H_
