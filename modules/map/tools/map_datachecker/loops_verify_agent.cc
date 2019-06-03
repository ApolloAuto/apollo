/******************************************************************************
 * Created on Thu Jan 17 2019
 *
 * Copyright (c) 2019 Baidu.com, Inc. All Rights Reserved
 *
 * @file: filename
 * @desc: description
 * @author: yuanyijun@baidu.com
  *****************************************************************************/

#include "modules/map/tools/map_datachecker/loops_verify_agent.h"
#include <limits>
#include <utility>
#include <vector>

namespace adu {
namespace workers {
namespace collection {

LoopsVerifyAgent::LoopsVerifyAgent(
    std::shared_ptr<JSonConf> sp_conf,
    std::shared_ptr<PoseCollectionAgent> sp_pose_collection_agent) {
    _sp_conf = sp_conf;
    _sp_pose_collection_agent = sp_pose_collection_agent;
}

grpc::Status LoopsVerifyAgent::process_grpc_request(
    grpc::ServerContext *context,
    LOOPS_VERIFY_REQUEST_TYPE *request,
    LOOPS_VERIFY_RESPONSE_TYPE *response) {
    AINFO << "LoopsVerifyAgent request is: " << request->DebugString();
    switch (request->cmd()) {
    case CmdType::START:
        StartVerify(request, response);
        break;
    case CmdType::CHECK:
        CheckVerify(request, response);
        break;
    case CmdType::STOP:
        StopVerify(request, response);
        break;
   default:
        response->set_progress(0.0);
        response->set_code(ErrorCode::ERROR_REQUEST);
        AERROR << "command error";
    }
    AINFO << "LoopsVerifyAgent response is: " << response->DebugString();
    return grpc::Status::OK;
}

void LoopsVerifyAgent::StartVerify(
    LOOPS_VERIFY_REQUEST_TYPE *request,
    LOOPS_VERIFY_RESPONSE_TYPE *response) {
    AINFO << "call StartVerify";
    if (get_state() == LoopsVerifyAgentState::RUNNING) {
        AINFO << "verify is working, do not need start again";
        response->set_progress(0.0);
        response->set_code(ErrorCode::ERROR_REPEATED_START);
    }
    // reset();
    std::shared_ptr<std::vector<std::pair<double, double>>>
        sp_range = get_verify_range(request);
    double loops_to_check = static_cast<double>(get_loops_to_check(request));
    std::thread loop_verify_thread([=]() {
        this->do_start_verify(sp_range, loops_to_check);
    });
    loop_verify_thread.detach();
    set_state(LoopsVerifyAgentState::RUNNING);
    response->set_code(ErrorCode::SUCCESS);
    response->set_progress(0.0);
}

void LoopsVerifyAgent::CheckVerify(
    LOOPS_VERIFY_REQUEST_TYPE *request,
    LOOPS_VERIFY_RESPONSE_TYPE *response) {
    AINFO << "call CheckVerify";
    if (get_state() == LoopsVerifyAgentState::IDLE) {
        AINFO << "verify is not work, start first";
        response->set_progress(0.0);
        response->set_code(ErrorCode::ERROR_CHECK_BEFORE_START);
        return;
    }

    if (_sp_laps_checker == nullptr) {
        AINFO << "_sp_laps_checker is preparing, check later";
        response->set_progress(0.0);
        response->set_code(ErrorCode::SUCCESS);
        return;
    }

    if (_sp_laps_checker->get_return_state() != 0) {
        response->set_progress(0.0);
        response->set_code(_sp_laps_checker->get_return_state());
        return;
    }

    double progress = _sp_laps_checker->get_progress();
    response->set_progress(progress);
    response->set_code(ErrorCode::SUCCESS);
    if (std::abs(1.0 - progress) < 1e-8) {
        double confidence = _sp_laps_checker->get_confidence();
        size_t lap = _sp_laps_checker->get_lap();
        AINFO << "aquired lap: " << lap << ", conf: " << confidence;
        adu::workers::collection::LoopResult
            *loop_result = response->mutable_loop_result();

        loop_result->set_loop_num(static_cast<double>(lap));
        bool is_reached = lap >= get_loops_to_check(request) ? true : false;
        loop_result->set_is_reached(is_reached);

        DataType data_type = request->type();
        if (data_type == DataType::MAP_CHECKOUT) {
            if (is_reached) {
                loop_result->set_loop_num(
                    static_cast<double>(_sp_conf->laps_number_additional));
            } else {
                loop_result->set_loop_num(lap - _sp_conf->laps_number >= 0 ?
                    static_cast<double>(lap - _sp_conf->laps_number) : 0.0);
            }
        }
    }
}

void LoopsVerifyAgent::StopVerify(
    LOOPS_VERIFY_REQUEST_TYPE *request,
    LOOPS_VERIFY_RESPONSE_TYPE *response) {
    AINFO << "call StopVerify";
    response->set_code(ErrorCode::SUCCESS);
    set_state(LoopsVerifyAgentState::IDLE);
    if (_sp_laps_checker == nullptr) {
        response->set_progress(0.0);
        return;
    }
    response->set_progress(_sp_laps_checker->get_progress());
    if (std::abs(1.0 - _sp_laps_checker->get_progress()) < 1e-8) {
        double conf = _sp_laps_checker->get_confidence();
        size_t lap = _sp_laps_checker->get_lap();
        AINFO << "aquired lap: " << lap << ", conf: " << conf;
        adu::workers::collection::LoopResult
            *loop_result = response->mutable_loop_result();
        loop_result->set_loop_num(static_cast<double>(lap));
        bool is_reached = lap >= get_loops_to_check(request) ? true : false;
        loop_result->set_is_reached(is_reached);

        DataType data_type = request->type();
        if (data_type == DataType::MAP_CHECKOUT) {
            if (is_reached) {
                loop_result->set_loop_num(
                    static_cast<double>(_sp_conf->laps_number_additional));
            } else {
                loop_result->set_loop_num(lap - _sp_conf->laps_number >= 0 ?
                    static_cast<double>(lap - _sp_conf->laps_number) : 0.0);
            }
        }
    }
}

std::shared_ptr<std::vector<std::pair<double, double>>>
LoopsVerifyAgent::get_verify_range(LOOPS_VERIFY_REQUEST_TYPE *request) {
    std::shared_ptr<std::vector<std::pair<double, double>>>
        sp_range(new std::vector<std::pair<double, double>>());
    int range_size = request->range_size();
    for (int i = 0; i < range_size; i++) {
        const ::adu::workers::collection::VerifyRange
            &range = request->range(i);
        sp_range->push_back(
            std::make_pair(range.start_time(), range.end_time()));
    }
    return sp_range;
}

size_t LoopsVerifyAgent::get_loops_to_check(
    LOOPS_VERIFY_REQUEST_TYPE *request) {
    size_t loops_to_check = 0;
    DataType data_type = request->type();
    if (data_type == DataType::MAP_MAKING) {
        loops_to_check += _sp_conf->laps_number;
    } else if (data_type == DataType::MAP_CHECKOUT) {
        loops_to_check += _sp_conf->laps_number;
        loops_to_check += _sp_conf->laps_number_additional;
    }
    return loops_to_check;
}


double LoopsVerifyAgent::get_range_index(
    std::shared_ptr<std::vector<std::pair<double, double>>> sp_range,
    std::vector<bool> * sp_range_index,
    std::shared_ptr<std::vector<FramePose>> sp_vec_poses) {
    if (sp_range == nullptr) {
        AINFO << "error, sp_range is null";
        return -1.0;
    }
    std::vector<std::pair<double, double>>& range = *sp_range;
    size_t size = range.size();
    double min_time = std::numeric_limits<double>::max();
    double max_time = std::numeric_limits<double>::min();
    for (size_t i = 0; i < size; i++) {
        if (range[i].first >= range[i].second) {
            AINFO << "range error, [" << std::to_string(range[i].first)
                  << "," << std::to_string(range[i].second) << "]";
            continue;
        }
        if (range[i].first < min_time) {
            min_time = range[i].first;
        }
        if (range[i].second > max_time) {
            max_time = range[i].second;
        }
    }
    AINFO << "[get_range_index] min_time:" << std::to_string(min_time)
          << ", max_time" << std::to_string(max_time);

    std::vector<bool> & range_index = * sp_range_index;
    if (size == 0 || max_time <= 0) {
        AINFO << "time range vector size is 0 or time range error";
        if (sp_vec_poses->size() > 0) {
            AINFO << "set index to check all poses";
            min_time = sp_vec_poses->front().time_stamp;
            max_time = sp_vec_poses->back().time_stamp;
            int index_size = static_cast<int>(max_time - min_time + 1);
            range_index.resize(index_size, true);
        }
    } else {
        AINFO << "time range vector size > 0";
        int index_size = static_cast<int>(max_time - min_time + 1);
        range_index.resize(index_size, false);
        for (int i = 0; i < static_cast<int>(size); i++) {
            int start_time = static_cast<int>(range[i].first - min_time);
            int end_time = static_cast<int>(range[i].second - min_time);
            for (int j = start_time; j <= end_time; j++) {
                range_index[j] = true;
            }
        }
    }
    AINFO << "returned min_time:" << std::to_string(min_time);
    return min_time;
}

int LoopsVerifyAgent::get_poses_to_check(
    std::shared_ptr<std::vector<std::pair<double, double>>> sp_range,
    std::vector<FramePose> * sp_poses) {
    if (_sp_pose_collection_agent == nullptr) {
        AINFO << "error, sp_pose_collection_agent is null";
        return -1;
    }
    std::shared_ptr<std::vector<FramePose>>
        sp_vec_poses = _sp_pose_collection_agent->get_poses();
    if (sp_vec_poses == nullptr || sp_vec_poses->size() == 0) {
        AINFO << "error, no pose";
        return -1;
    }

    std::vector<bool> range_index;
    double min_time = get_range_index(sp_range, &range_index, sp_vec_poses);
    if (min_time == std::numeric_limits<double>::max() ||
        range_index.size() == 0) {
        AINFO << "min_time: " << min_time
              << ", range_index size: " << range_index.size();
        return -1;
    }
    std::vector<FramePose>& vec_poses = *sp_vec_poses;
    size_t pose_size = vec_poses.size();
    size_t range_size = range_index.size();
    std::vector<FramePose> & poses = *sp_poses;
    poses.clear();
    for (size_t i = 0; i < pose_size; i++) {
        int time = static_cast<int>(vec_poses[i].time_stamp - min_time);
        if (time >= static_cast<int>(range_size)) {
            break;
        }
        if (time >= 0 && range_index[time]) {
            poses.push_back(vec_poses[i]);
        }
    }
    return 0;
}

int LoopsVerifyAgent::do_start_verify(
    std::shared_ptr<std::vector<std::pair<double, double>>> sp_range,
    double loops_to_check) {
    clock_t start = clock();
    std::vector<FramePose> all_poses;
    get_poses_to_check(sp_range, &all_poses);

    _sp_laps_checker = std::shared_ptr<LapsChecker>(
        new LapsChecker(all_poses, static_cast<int>(loops_to_check), _sp_conf));
    _sp_laps_checker->check();

    double duration = (static_cast<double>(clock() - start)) / CLOCKS_PER_SEC;
    AINFO << "checking laps cost " << duration << " seconds";
    return 0;
}

void LoopsVerifyAgent::set_state(LoopsVerifyAgentState state) {
    _state = state;
}
LoopsVerifyAgentState LoopsVerifyAgent::get_state() {
    return _state;
}

}  // namespace collection
}  // namespace workers
}  // namespace adu
