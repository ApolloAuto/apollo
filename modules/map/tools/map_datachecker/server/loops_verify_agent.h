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

#include <grpc++/grpc++.h>
#include <memory>
#include <utility>
#include <vector>

#include "modules/map/tools/map_datachecker/proto/collection_service.pb.h"
#include "modules/map/tools/map_datachecker/server/laps_checker.h"
#include "modules/map/tools/map_datachecker/server/pose_collection_agent.h"

namespace apollo {
namespace hdmap {

enum class LoopsVerifyAgentState { IDLE, RUNNING };

class LoopsVerifyAgent {
 public:
  LoopsVerifyAgent(
      std::shared_ptr<JSonConf> sp_conf,
      std::shared_ptr<PoseCollectionAgent> sp_pose_collection_agent);
  grpc::Status ProcessGrpcRequest(grpc::ServerContext *context,
                                  LoopsVerifyRequest *request,
                                  LoopsVerifyResponse *response);

 private:
  void StartVerify(LoopsVerifyRequest *request, LoopsVerifyResponse *response);
  void CheckVerify(LoopsVerifyRequest *request, LoopsVerifyResponse *response);
  void StopVerify(LoopsVerifyRequest *request, LoopsVerifyResponse *response);
  std::shared_ptr<std::vector<std::pair<double, double>>> get_verify_range(
      LoopsVerifyRequest *request);
  size_t GetLoopsToCheck(LoopsVerifyRequest *request);
  int GetPosesToCheck(
      std::shared_ptr<std::vector<std::pair<double, double>>> sp_range,
      std::vector<FramePose> *poses);
  int DoStartVerify(
      std::shared_ptr<std::vector<std::pair<double, double>>> sp_range,
      double loops_to_check);
  double GetRangeIndex(
      std::shared_ptr<std::vector<std::pair<double, double>>> sp_range,
      std::vector<bool> *range_index,
      std::shared_ptr<std::vector<FramePose>> sp_vec_poses);
  void SetState(LoopsVerifyAgentState state);
  LoopsVerifyAgentState GetState();

 private:
  std::shared_ptr<JSonConf> sp_conf_ = nullptr;
  std::shared_ptr<PoseCollectionAgent> sp_pose_collection_agent_ = nullptr;
  std::shared_ptr<LapsChecker> sp_laps_checker_ = nullptr;
  LoopsVerifyAgentState state_;
};

}  // namespace hdmap
}  // namespace apollo
