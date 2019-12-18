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

#include "cyber/cyber.h"
#include "modules/drivers/gnss/proto/gnss_best_pose.pb.h"

namespace apollo {
namespace hdmap {

class MapDataCheckerAgent;

class MapDataCheckerCyberNode
    : public std::enable_shared_from_this<MapDataCheckerCyberNode> {
 public:
  using GnssBestPose_t = apollo::drivers::gnss::GnssBestPose;

 public:
  MapDataCheckerCyberNode(std::shared_ptr<MapDataCheckerAgent> agent,
                          bool *init_success);

  inline std::shared_ptr<MapDataCheckerCyberNode> GetWorkerCyberNode() {
    return shared_from_this();
  }

 private:
  int CreateChannelSubscriber();

 private:
  std::shared_ptr<apollo::cyber::Node> node_ = nullptr;
  std::shared_ptr<apollo::cyber::Reader<GnssBestPose_t>> bestgnsspos_reader_ =
      nullptr;
  std::shared_ptr<MapDataCheckerAgent> agent_ = nullptr;
};

}  // namespace hdmap
}  // namespace apollo
