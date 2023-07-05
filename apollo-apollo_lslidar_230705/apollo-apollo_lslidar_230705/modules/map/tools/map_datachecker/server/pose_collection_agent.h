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
#include <mutex>
#include <vector>

#include "modules/common_msgs/sensor_msgs/gnss_best_pose.pb.h"
#include "modules/map/tools/map_datachecker/server/common.h"
#include "modules/map/tools/map_datachecker/server/pj_transformer.h"
#include "modules/map/tools/map_datachecker/server/pose_collection.h"

namespace apollo {
namespace hdmap {

class PoseCollectionAgent {
 public:
  explicit PoseCollectionAgent(std::shared_ptr<JsonConf> sp_conf);

  void OnBestgnssposCallback(
      const std::shared_ptr<const apollo::drivers::gnss::GnssBestPose>
          &bestgnsspos);
  std::shared_ptr<std::vector<FramePose>> GetPoses() const;

 private:
  void Reset();

 private:
  std::mutex mutex_;
  std::shared_ptr<PoseCollection> sp_pose_collection_ = nullptr;
  std::shared_ptr<JsonConf> sp_conf_ = nullptr;
  std::shared_ptr<PJTransformer> sp_pj_transformer_ = nullptr;
};

}  // namespace hdmap
}  // namespace apollo
