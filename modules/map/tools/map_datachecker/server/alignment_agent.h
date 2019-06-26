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

  void reset() {
    sp_alignment_ = std::make_shared<ALIGNMENT_TYPE>(sp_conf_);
    state_ = AlignmentAgentState::IDLE;
    need_stop_ = false;
  }

  grpc::Status process_grpc_request(grpc::ServerContext *context,
                                    REQUEST_TYPE *request,
                                    RESPONSE_TYPE *response) {
    AINFO << "AlignmentAgent request: " << request->DebugString();
    switch (request->cmd()) {
      case CmdType::START:
        AINFO << "AlignmentAgent start";
        alignment_start(request, response);
        break;
      case CmdType::CHECK:
        AINFO << "AlignmentAgent check";
        alignment_check(request, response);
        break;
      case CmdType::STOP:
        AINFO << "AlignmentAgent stop";
        alignment_stop(request, response);
        break;
      default:
        response->set_code(ErrorCode::ERROR_REQUEST);
        response->set_progress(sp_alignment_->get_progress());
        AERROR << "command error";
    }
    AINFO << "AlignmentAgent progress: " << response->progress();
    return grpc::Status::OK;
  }

  int alignment_start(REQUEST_TYPE *request, RESPONSE_TYPE *response) {
    if (AlignmentAgentState::RUNNING == get_state()) {
      AINFO << "AlignmentAgent is running. do need start again";
      response->set_code(ErrorCode::ERROR_REPEATED_START);
      response->set_progress(0.0);
      return 0;
    }
    reset();
    async_start_alignment();
    response->set_code(ErrorCode::SUCCESS);
    response->set_progress(0.0);
    return 0;
  }

  int async_start_alignment() {
    set_state(AlignmentAgentState::RUNNING);
    std::thread alignment_thread([=]() {
      sp_alignment_->set_start_time(unixtime_now());
      AINFO << "set state RUNNING";
      while (!need_stop_ && !apollo::cyber::IsShutdown()) {
        std::shared_ptr<std::vector<FramePose>> sp_poses = get_poses();
        if (sp_poses == nullptr) {
          AINFO << "error, pose pointer is null";
          return;
        }
        sp_alignment_->process(*sp_poses);
        ErrorCode code = sp_alignment_->get_return_state();
        if (code == ErrorCode::ERROR_VERIFY_NO_GNSSPOS ||
            code == ErrorCode::ERROR_GNSS_SIGNAL_FAIL) {
          AERROR << "Some error occured, while loop will exit";
          break;
        }
        AINFO << "get progress:" << sp_alignment_->get_progress();
        if (fabs(1 - sp_alignment_->get_progress()) < 1e-8) {
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

  std::shared_ptr<std::vector<FramePose>> get_poses() const {
    if (sp_pose_collection_agent_ == nullptr) {
      return nullptr;
    }
    return sp_pose_collection_agent_->get_poses();
  }

  int alignment_check(REQUEST_TYPE *request, RESPONSE_TYPE *response) {
    if (AlignmentAgentState::IDLE == get_state()) {
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

    ErrorCode code = sp_alignment_->get_return_state();
    double progress = sp_alignment_->get_progress();
    response->set_code(code);
    response->set_progress(progress);
    if (code == ErrorCode::ERROR_VERIFY_NO_GNSSPOS ||
        code == ErrorCode::ERROR_GNSS_SIGNAL_FAIL) {
      stopped_ = true;
      set_state(AlignmentAgentState::IDLE);
      return -1;
    }
    return 0;
  }

  int alignment_stop(REQUEST_TYPE *request, RESPONSE_TYPE *response) {
    response->set_code(ErrorCode::SUCCESS);
    if (sp_alignment_ == nullptr) {
      response->set_progress(0.0);
    } else {
      response->set_progress(sp_alignment_->get_progress());
      need_stop_ = true;
    }
    set_state(AlignmentAgentState::IDLE);
    return 0;
  }

  void set_state(AlignmentAgentState state) { state_ = state; }
  AlignmentAgentState get_state() const { return state_; }

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
