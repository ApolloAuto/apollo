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
#ifndef MODULES_MAP_TOOLS_MAP_DATACHECKER_LOOPS_VERIFY_AGENT_H
#define MODULES_MAP_TOOLS_MAP_DATACHECKER_LOOPS_VERIFY_AGENT_H
#include <grpc++/grpc++.h>
#include <memory>
#include <utility>
#include <vector>
#include "modules/map/tools/map_datachecker/proto/collection_service.pb.h"
#include "modules/map/tools/map_datachecker/laps_checker.h"
#include "modules/map/tools/map_datachecker/pose_collection_agent.h"

namespace apollo {
namespace hdmap {

enum class LoopsVerifyAgentState {
  IDLE,
  RUNNING
};
using LOOPS_VERIFY_REQUEST_TYPE
  = const apollo::hdmap::LoopsVerifyRequest;
using LOOPS_VERIFY_RESPONSE_TYPE
  = apollo::hdmap::LoopsVerifyResponse;

class LoopsVerifyAgent {
 public:
  LoopsVerifyAgent(
    std::shared_ptr<JSonConf> sp_conf,
    std::shared_ptr<PoseCollectionAgent> sp_pose_collection_agent);
  grpc::Status process_grpc_request(
    grpc::ServerContext *context,
    LOOPS_VERIFY_REQUEST_TYPE *request,
    LOOPS_VERIFY_RESPONSE_TYPE *response);

 private:
  void StartVerify(
    LOOPS_VERIFY_REQUEST_TYPE *request,
    LOOPS_VERIFY_RESPONSE_TYPE *response);
  void CheckVerify(
    LOOPS_VERIFY_REQUEST_TYPE *request,
    LOOPS_VERIFY_RESPONSE_TYPE *response);
  void StopVerify(
    LOOPS_VERIFY_REQUEST_TYPE *request,
    LOOPS_VERIFY_RESPONSE_TYPE *response);
  std::shared_ptr<std::vector<std::pair<double, double>>>
    get_verify_range(LOOPS_VERIFY_REQUEST_TYPE *request);
  size_t get_loops_to_check(LOOPS_VERIFY_REQUEST_TYPE *request);
  int get_poses_to_check(
    std::shared_ptr<std::vector<std::pair<double, double>>> sp_range,
    std::vector<FramePose> * poses);
  int do_start_verify(
    std::shared_ptr<std::vector<std::pair<double, double>>> sp_range,
    double loops_to_check);
  double get_range_index(
    std::shared_ptr<std::vector<std::pair<double, double>>> sp_range,
    std::vector<bool> * range_index,
    std::shared_ptr<std::vector<FramePose>> sp_vec_poses);
  void set_state(LoopsVerifyAgentState state);
  LoopsVerifyAgentState get_state();

 private:
  std::shared_ptr<JSonConf> _sp_conf = nullptr;
  std::shared_ptr<PoseCollectionAgent> _sp_pose_collection_agent = nullptr;
  std::shared_ptr<LapsChecker> _sp_laps_checker = nullptr;
  LoopsVerifyAgentState _state;
};

}  // namespace hdmap
}  // namespace apollo

#endif  // MODULES_MAP_TOOLS_MAP_DATACHECKER_LOOPS_VERIFY_AGENT_H
