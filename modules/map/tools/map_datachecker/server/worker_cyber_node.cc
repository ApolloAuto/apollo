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
#include "modules/map/tools/map_datachecker/server/worker_cyber_node.h"

#include <memory>
#include <string>

#include "modules/map/tools/map_datachecker/server/worker_agent.h"
#include "modules/map/tools/map_datachecker/server/worker_gflags.h"

constexpr double kRADIANS_TO_DEGREES = 180.0 / M_PI;
constexpr double kDEGRESS_TO_RADIANS = M_PI / 180.0;

namespace apollo {
namespace hdmap {

MapDataCheckerCyberNode::MapDataCheckerCyberNode(
    std::shared_ptr<MapDataCheckerAgent> agent, bool *init_success) {
  if (!agent) {
    AFATAL << "MapDataCheckerAgent pointer is nullptr";
    *init_success = false;
    return;
  }

  agent_ = agent->GetWorkerAgent();
  node_ = apollo::cyber::CreateNode(std::string("cybernode_map_datachecker"));
  if (!node_) {
    AFATAL << "Create cybertron node failed.";
    *init_success = false;
    return;
  }

  // Readers
  CreateChannelSubscriber();

  *init_success = true;
  AINFO << "map-datachecker cyber node create successfully";
}

int MapDataCheckerCyberNode::CreateChannelSubscriber() {
  AINFO << "create bestgnsspos reader, topic: " << FLAGS_topic_bestgnsspos;
  bestgnsspos_reader_ = node_->CreateReader<GnssBestPose_t>(
      FLAGS_topic_bestgnsspos,
      [this](const std::shared_ptr<const GnssBestPose_t> &msg) {
        agent_->GetSpPoseCollectionAgent()->OnBestgnssposCallback(msg);
      });
  if (!bestgnsspos_reader_) {
    AFATAL << "create bestgnsspos reader error";
    return -1;
  }
  return 0;
}

}  // namespace hdmap
}  // namespace apollo
