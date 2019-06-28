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
#include <chrono>
#include <memory>
#include <thread>
#include <vector>

#include "modules/map/tools/map_datachecker/proto/collection_error_code.pb.h"
#include "modules/map/tools/map_datachecker/proto/collection_service.pb.h"
#include "modules/map/tools/map_datachecker/server/eight_route.h"
#include "modules/map/tools/map_datachecker/server/pose_collection_agent.h"
#include "modules/map/tools/map_datachecker/server/static_align.h"

namespace apollo {
namespace hdmap {

typedef State AlignmentAgentState;

template <typename ALIGNMENT_TYPE, typename REQUEST_TYPE,
          typename RESPONSE_TYPE>
class AlignmentAgent {
 public:
  AlignmentAgent(
      std::shared_ptr<JSonConf> sp_conf,
      std::shared_ptr<PoseCollectionAgent> sp_pose_collection_agent) {
    sp_conf_ = sp_conf;
    sp_pose_collection_agent_ = sp_pose_collection_agent;
  }

  void Reset() {
    sp_alignment_ = std::make_shared<ALIGNMENT_TYPE>(sp_conf_);
    state_ = AlignmentAgentState::IDLE;
    need_stop_ = false;
  }

  grpc::Status ProcessGrpcRequest(grpc::ServerContext *context,
                                  REQUEST_TYPE *request,
                                  RESPONSE_TYPE *response) {
    AINFO << "AlignmentAgent request: " << request->DebugString();
    switch (request->cmd()) {
      case CmdType::START:
        AINFO << "AlignmentAgent start";
        AlignmentStart(request, response);
        break;
      case CmdType::CHECK:
        AINFO << "AlignmentAgent check";
        AlignmentCheck(request, response);
        break;
      case CmdType::STOP:
        AINFO << "AlignmentAgent stop";
        AlignmentStop(request, response);
        break;
      default:
        response->set_code(ErrorCode::ERROR_REQUEST);
        response->set_progress(sp_alignment_->GetProgress());
        AERROR << "command error";
    }
    AINFO << "AlignmentAgent progress: " << response->progress();
    return grpc::Status::OK;
  }

  int AlignmentStart(REQUEST_TYPE *request, RESPONSE_TYPE *response) {
    if (AlignmentAgentState::RUNNING == GetState()) {
      AINFO << "AlignmentAgent is running. do need start again";
      response->set_code(ErrorCode::ERROR_REPEATED_START);
      response->set_progress(0.0);
      return 0;
    }
    Reset();
    AsyncStartAlignment();
    response->set_code(ErrorCode::SUCCESS);
    response->set_progress(0.0);
    return 0;
  }

  int AsyncStartAlignment() {
    SetState(AlignmentAgentState::RUNNING);
    std::thread alignment_thread([=]() {
      sp_alignment_->SetStartTime(UnixtimeNow());
      AINFO << "set state RUNNING";
      while (!need_stop_ && !apollo::cyber::IsShutdown()) {
        std::shared_ptr<std::vector<FramePose>> sp_poses = GetPoses();
        if (sp_poses == nullptr) {
          AINFO << "error, pose pointer is null";
          return;
        }
        sp_alignment_->Process(*sp_poses);
        ErrorCode code = sp_alignment_->GetReturnState();
        if (code == ErrorCode::ERROR_VERIFY_NO_GNSSPOS ||
            code == ErrorCode::ERROR_GNSS_SIGNAL_FAIL) {
          AERROR << "Some error occurred, while loop will exit";
          break;
        }
        AINFO << "get progress:" << sp_alignment_->GetProgress();
        if (fabs(1 - sp_alignment_->GetProgress()) < 1e-8) {
          AINFO << "alignment progress reached 1.0, thread exit";
          break;
        }
        AINFO << "sleep " << sp_conf_->alignment_featch_pose_sleep << " sec";
        auto seconds =
            std::chrono::seconds(sp_conf_->alignment_featch_pose_sleep);
        std::this_thread::sleep_for(seconds);
      }
      stopped_ = true;
      AINFO << "Align thread complete";
    });
    alignment_thread.detach();
    return 0;
  }

  std::shared_ptr<std::vector<FramePose>> GetPoses() const {
    if (sp_pose_collection_agent_ == nullptr) {
      return nullptr;
    }
    return sp_pose_collection_agent_->GetPoses();
  }

  int AlignmentCheck(REQUEST_TYPE *request, RESPONSE_TYPE *response) {
    if (AlignmentAgentState::IDLE == GetState()) {
      AINFO << "AlignmentAgent is idle. this call will be refused";
      response->set_code(ErrorCode::ERROR_CHECK_BEFORE_START);
      response->set_progress(0.0);
      return 0;
    }
    if (sp_alignment_ == nullptr) {
      AINFO << "sp_alignment_ is null, check later";
      response->set_code(ErrorCode::SUCCESS);
      response->set_progress(0.0);
      return 0;
    }

    ErrorCode code = sp_alignment_->GetReturnState();
    double progress = sp_alignment_->GetProgress();
    response->set_code(code);
    response->set_progress(progress);
    if (code == ErrorCode::ERROR_VERIFY_NO_GNSSPOS ||
        code == ErrorCode::ERROR_GNSS_SIGNAL_FAIL) {
      stopped_ = true;
      SetState(AlignmentAgentState::IDLE);
      return -1;
    }
    return 0;
  }

  int AlignmentStop(REQUEST_TYPE *request, RESPONSE_TYPE *response) {
    response->set_code(ErrorCode::SUCCESS);
    if (sp_alignment_ == nullptr) {
      response->set_progress(0.0);
    } else {
      response->set_progress(sp_alignment_->GetProgress());
      need_stop_ = true;
    }
    SetState(AlignmentAgentState::IDLE);
    return 0;
  }

  void SetState(AlignmentAgentState state) { state_ = state; }
  AlignmentAgentState GetState() const { return state_; }

 private:
  std::shared_ptr<JSonConf> sp_conf_ = nullptr;
  std::shared_ptr<ALIGNMENT_TYPE> sp_alignment_ = nullptr;
  std::shared_ptr<PoseCollectionAgent> sp_pose_collection_agent_ = nullptr;
  AlignmentAgentState state_;
  bool need_stop_;
  bool stopped_;
};

}  // namespace hdmap
}  // namespace apollo
