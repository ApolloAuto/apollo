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
#pragma once

#include <memory>
#include <vector>

#include "grpc++/grpc++.h"

#include "cyber/cyber.h"
#include "modules/map/tools/map_datachecker/server/alignment.h"
#include "modules/map/tools/map_datachecker/server/common.h"
#include "modules/map/tools/map_datachecker/server/worker_gflags.h"

namespace apollo {
namespace hdmap {
// TODO(yuanyijun): change EightRoute to FigureEight
class EightRoute : public Alignment {
 public:
  explicit EightRoute(std::shared_ptr<JsonConf> sp_conf);
  ErrorCode Process(const std::vector<FramePose>& poses);
  double GetProgress() const;

 private:
  void Reset();
  bool IsEightRoutePose(const std::vector<FramePose>& poses, int pose_index);
  double GetGoodPoseDuring();
  double GetEightRouteProgress(const std::vector<FramePose>& poses);

 private:
  double progress_;
  double last_yaw_;
};

}  // namespace hdmap
}  // namespace apollo
