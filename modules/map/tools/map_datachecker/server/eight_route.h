/******************************************************************************
 * Copyright 2019 The Apollo Authors. All Rights Reserved.
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
#ifndef _MODULES_MAP_TOOLS_MAP_DATACHECKER_SERVER_EIGHT_ROUTE_H
#define _MODULES_MAP_TOOLS_MAP_DATACHECKER_SERVER_EIGHT_ROUTE_H
#include <grpc++/grpc++.h>
#include <memory>
#include <vector>
#include "cyber/cyber.h"
#include "modules/map/tools/map_datachecker/server/alignment.hpp"
#include "modules/map/tools/map_datachecker/server/common.hpp"
#include "modules/map/tools/map_datachecker/server/worker_gflags.h"

namespace apollo {
namespace hdmap {
// TODO(yuanyijun): change EightRoute to FigureEight
class EightRoute : public Alignment {
 public:
  explicit EightRoute(std::shared_ptr<JSonConf> sp_conf);
  ErrorCode process(const std::vector<FramePose>& poses);
  double get_progress() const;

 private:
  void reset();
  bool is_eight_route_pose(const std::vector<FramePose>& poses, int pose_index);
  double get_good_pose_during();
  double get_eight_route_progress(const std::vector<FramePose>& poses);

 private:
  double _progress;
  double _last_yaw;
};

}  // namespace hdmap
}  // namespace apollo

#endif  // _MODULES_MAP_TOOLS_MAP_DATACHECKER_SERVER_EIGHT_ROUTE_H
