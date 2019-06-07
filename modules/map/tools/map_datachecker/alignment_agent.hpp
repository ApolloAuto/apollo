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
#ifndef _MODULES_MAP_TOOLS_MAP_DATACHECKER_ALIGNMENT_AGENT_HPP
#define _MODULES_MAP_TOOLS_MAP_DATACHECKER_ALIGNMENT_AGENT_HPP
#include <grpc++/grpc++.h>
#include <chrono>
#include <thread>
#include <memory>
#include <vector>
#include "modules/map/tools/map_datachecker/static_align.h"
#include "modules/map/tools/map_datachecker/eight_route.h"
#include "modules/map/tools/map_datachecker/pose_collection_agent.h"
#include "modules/map/tools/map_datachecker/proto/collection_error_code.pb.h"
#include "modules/map/tools/map_datachecker/proto/collection_service.pb.h"

namespace apollo {
namespace hdmap {

typedef State AlignmentAgentState;

using STATIC_REQUEST_TYPE
    = const apollo::hdmap::StaticAlignRequest;
using STATIC_RESPONSE_TYPE
    = apollo::hdmap::StaticAlignResponse;
using EIGHTROUTE_REQUEST_TYPE
    = const apollo::hdmap::EightRouteRequest;
using EIGHTROUTE_RESPONSE_TYPE
    = apollo::hdmap::EightRouteResponse;

template<typename ALIGNMENT_TYPE, typename REQUEST_TYPE, typename RESPONSE_TYPE>
class AlignmentAgent {
 public:
    AlignmentAgent(
        std::shared_ptr<JSonConf> sp_conf,
        std::shared_ptr<PoseCollectionAgent> sp_pose_collection_agent) {
        _sp_conf = sp_conf;
        _sp_pose_collection_agent = sp_pose_collection_agent;
    }

    void reset() {
        _sp_alignment = std::make_shared<ALIGNMENT_TYPE>(_sp_conf);
        _state = AlignmentAgentState::IDLE;
        // _have_result = false;
        _need_stop = false;
    }

    grpc::Status process_grpc_request(
        grpc::ServerContext *context,
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
            response->set_progress(_sp_alignment->get_progress());
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
        std::thread alignment_thread([=](){
            _sp_alignment->set_start_time(unixtime_now());
            _thread_id = std::this_thread::get_id();
            AINFO << "set state RUNNING";
            while (!_need_stop && !apollo::cyber::IsShutdown()) {
                std::shared_ptr<std::vector<FramePose>> sp_poses = get_poses();
                if ( sp_poses == nullptr ) {
                    AINFO << "error, pose pointer is null";
                    return;
                }
                _sp_alignment->process(*sp_poses);
                AINFO << "get progress:" << _sp_alignment->get_progress();
                if (std::abs(1 - _sp_alignment->get_progress()) < 1e-8) {
                    AINFO << "alignment progress reached 1.0, thread exit";
                    break;
                }
                AINFO << "sleep " << _sp_conf->alignment_featch_pose_sleep
                      << " sec";
                std::this_thread::sleep_for(
                    std::chrono::seconds(
                        _sp_conf->alignment_featch_pose_sleep));
            }
            _stopped = true;
            AINFO << "Align thread complete";
            // if (_thread_id == std::this_thread::get_id()) {
            //     AINFO << "set state IDLE";
            // }
        });
        alignment_thread.detach();
        return 0;
    }

    std::shared_ptr<std::vector<FramePose>> get_poses() {
        if (_sp_pose_collection_agent == nullptr) {
            return nullptr;
        }
        return _sp_pose_collection_agent->get_poses();
    }

    int alignment_check(REQUEST_TYPE *request, RESPONSE_TYPE *response) {
        if (AlignmentAgentState::IDLE == get_state()) {
            AINFO << "AlignmentAgent is idle. this call will be refused";
            response->set_code(ErrorCode::ERROR_CHECK_BEFORE_START);
            response->set_progress(0.0);
            return 0;
        }
        if (_sp_alignment == nullptr) {
            AINFO << "_sp_alignment is null, check later";
            response->set_code(ErrorCode::SUCCESS);
            response->set_progress(0.0);
            return 0;
        }

        double progress = _sp_alignment->get_progress();
        response->set_code(_sp_alignment->get_return_state());
        response->set_progress(progress);
        return 0;
    }

    int alignment_stop(REQUEST_TYPE *request, RESPONSE_TYPE *response) {
        response->set_code(ErrorCode::SUCCESS);
        if (_sp_alignment == nullptr) {
            response->set_progress(0.0);
        } else {
            response->set_progress(_sp_alignment->get_progress());
            _need_stop = true;
        }
        set_state(AlignmentAgentState::IDLE);
        return 0;
    }

    void set_state(AlignmentAgentState state) {
        _state = state;
    }

    AlignmentAgentState get_state() {
        return _state;
    }

 private:
    std::shared_ptr<JSonConf> _sp_conf = nullptr;
    std::shared_ptr<ALIGNMENT_TYPE> _sp_alignment = nullptr;
    std::shared_ptr<PoseCollectionAgent> _sp_pose_collection_agent = nullptr;
    AlignmentAgentState _state;
    bool _need_stop, _stopped;
    std::vector<ErrorCode> error_code_map;
    std::thread::id _thread_id;
    // bool _have_result;
};

using STATIC_ALIGN_AGENT_TYPE
    = AlignmentAgent<StaticAlign, STATIC_REQUEST_TYPE, STATIC_RESPONSE_TYPE>;
using EIGHT_ROUTE_AGENT_TYPE
    = AlignmentAgent<EightRoute, EIGHTROUTE_REQUEST_TYPE,
    EIGHTROUTE_RESPONSE_TYPE>;
}  // namespace hdmap
}  // namespace apollo


#endif  // _MODULES_MAP_TOOLS_MAP_DATACHECKER_ALIGNMENT_AGENT_HPP
