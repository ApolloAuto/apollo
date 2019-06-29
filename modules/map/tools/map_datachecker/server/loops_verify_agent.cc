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
#include "modules/map/tools/map_datachecker/server/loops_verify_agent.h"

#include <limits>
#include <utility>
#include <vector>

namespace apollo {
namespace hdmap {

LoopsVerifyAgent::LoopsVerifyAgent(
    std::shared_ptr<JSonConf> sp_conf,
    std::shared_ptr<PoseCollectionAgent> sp_pose_collection_agent) {
  sp_conf_ = sp_conf;
  sp_pose_collection_agent_ = sp_pose_collection_agent;
}

grpc::Status LoopsVerifyAgent::ProcessGrpcRequest(
    grpc::ServerContext *context, LoopsVerifyRequest *request,
    LoopsVerifyResponse *response) {
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

void LoopsVerifyAgent::StartVerify(LoopsVerifyRequest *request,
                                   LoopsVerifyResponse *response) {
  AINFO << "Call StartVerify";
  if (GetState() == LoopsVerifyAgentState::RUNNING) {
    AINFO << "Verify is working, do not need start again";
    response->set_progress(0.0);
    response->set_code(ErrorCode::ERROR_REPEATED_START);
  }

  std::shared_ptr<std::vector<std::pair<double, double>>> sp_range =
      get_verify_range(request);
  double loops_to_check = static_cast<double>(GetLoopsToCheck(request));
  std::thread loop_verify_thread(
      [=]() { this->DoStartVerify(sp_range, loops_to_check); });
  loop_verify_thread.detach();
  SetState(LoopsVerifyAgentState::RUNNING);
  response->set_code(ErrorCode::SUCCESS);
  response->set_progress(0.0);
}

void LoopsVerifyAgent::CheckVerify(LoopsVerifyRequest *request,
                                   LoopsVerifyResponse *response) {
  AINFO << "Call CheckVerify";
  if (GetState() == LoopsVerifyAgentState::IDLE) {
    AINFO << "Verify does not work, start first";
    response->set_progress(0.0);
    response->set_code(ErrorCode::ERROR_CHECK_BEFORE_START);
    return;
  }

  if (sp_laps_checker_ == nullptr) {
    AINFO << "sp_laps_checker_ is preparing, check later";
    response->set_progress(0.0);
    response->set_code(ErrorCode::SUCCESS);
    return;
  }

  if (sp_laps_checker_->GetReturnState() != 0) {
    response->set_progress(0.0);
    response->set_code(sp_laps_checker_->GetReturnState());
    return;
  }

  double progress = sp_laps_checker_->GetProgress();
  response->set_progress(progress);
  response->set_code(ErrorCode::SUCCESS);
  if (std::abs(1.0 - progress) < 1e-8) {
    double confidence = sp_laps_checker_->GetConfidence();
    size_t lap = sp_laps_checker_->GetLap();
    AINFO << "acquired lap: " << lap << ", conf: " << confidence;
    LoopResult *loop_result = response->mutable_loop_result();

    loop_result->set_loop_num(static_cast<double>(lap));
    bool is_reached = lap >= GetLoopsToCheck(request) ? true : false;
    loop_result->set_is_reached(is_reached);

    DataType data_type = request->type();
    if (data_type == DataType::MAP_CHECKOUT) {
      if (is_reached) {
        loop_result->set_loop_num(
            static_cast<double>(sp_conf_->laps_number_additional));
      } else {
        loop_result->set_loop_num(
            lap - sp_conf_->laps_number >= 0
                ? static_cast<double>(lap - sp_conf_->laps_number)
                : 0.0);
      }
    }
  }
}

void LoopsVerifyAgent::StopVerify(LoopsVerifyRequest *request,
                                  LoopsVerifyResponse *response) {
  AINFO << "call StopVerify";
  response->set_code(ErrorCode::SUCCESS);
  SetState(LoopsVerifyAgentState::IDLE);
  if (sp_laps_checker_ == nullptr) {
    response->set_progress(0.0);
    return;
  }
  response->set_progress(sp_laps_checker_->GetProgress());
  if (std::abs(1.0 - sp_laps_checker_->GetProgress()) < 1e-8) {
    double conf = sp_laps_checker_->GetConfidence();
    size_t lap = sp_laps_checker_->GetLap();
    AINFO << "acquired lap: " << lap << ", conf: " << conf;
    LoopResult *loop_result = response->mutable_loop_result();
    loop_result->set_loop_num(static_cast<double>(lap));
    bool is_reached = lap >= GetLoopsToCheck(request) ? true : false;
    loop_result->set_is_reached(is_reached);

    DataType data_type = request->type();
    if (data_type == DataType::MAP_CHECKOUT) {
      if (is_reached) {
        loop_result->set_loop_num(
            static_cast<double>(sp_conf_->laps_number_additional));
      } else {
        loop_result->set_loop_num(
            lap - sp_conf_->laps_number >= 0
                ? static_cast<double>(lap - sp_conf_->laps_number)
                : 0.0);
      }
    }
  }
}

std::shared_ptr<std::vector<std::pair<double, double>>>
LoopsVerifyAgent::get_verify_range(LoopsVerifyRequest *request) {
  std::shared_ptr<std::vector<std::pair<double, double>>> sp_range(
      new std::vector<std::pair<double, double>>());
  for (const VerifyRange &range : request->range()) {
    sp_range->push_back(std::make_pair(range.start_time(), range.end_time()));
  }
  return sp_range;
}

size_t LoopsVerifyAgent::GetLoopsToCheck(LoopsVerifyRequest *request) {
  size_t loops_to_check = 0;
  DataType data_type = request->type();
  if (data_type == DataType::MAP_MAKING) {
    loops_to_check += sp_conf_->laps_number;
  } else if (data_type == DataType::MAP_CHECKOUT) {
    loops_to_check += sp_conf_->laps_number;
    loops_to_check += sp_conf_->laps_number_additional;
  }
  return loops_to_check;
}

double LoopsVerifyAgent::GetRangeIndex(
    std::shared_ptr<std::vector<std::pair<double, double>>> sp_range,
    std::vector<bool> *sp_range_index,
    std::shared_ptr<std::vector<FramePose>> sp_vec_poses) {
  if (sp_range == nullptr) {
    AINFO << "error, sp_range is null";
    return -1.0;
  }
  std::vector<std::pair<double, double>> &range = *sp_range;
  size_t size = range.size();
  double min_time = std::numeric_limits<double>::max();
  double max_time = std::numeric_limits<double>::min();
  for (size_t i = 0; i < size; ++i) {
    if (range[i].first >= range[i].second) {
      AINFO << "range error, [" << std::to_string(range[i].first) << ","
            << std::to_string(range[i].second) << "]";
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

  std::vector<bool> &range_index = *sp_range_index;
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
    for (int i = 0; i < static_cast<int>(size); ++i) {
      int start_time = static_cast<int>(range[i].first - min_time);
      int end_time = static_cast<int>(range[i].second - min_time);
      for (int j = start_time; j <= end_time; ++j) {
        range_index[j] = true;
      }
    }
  }
  AINFO << "returned min_time:" << std::to_string(min_time);
  return min_time;
}

int LoopsVerifyAgent::GetPosesToCheck(
    std::shared_ptr<std::vector<std::pair<double, double>>> sp_range,
    std::vector<FramePose> *sp_poses) {
  if (sp_pose_collection_agent_ == nullptr) {
    AINFO << "error, sp_pose_collection_agent is null";
    return -1;
  }
  std::shared_ptr<std::vector<FramePose>> sp_vec_poses =
      sp_pose_collection_agent_->GetPoses();
  if (sp_vec_poses == nullptr || sp_vec_poses->size() == 0) {
    AINFO << "error, no pose";
    return -1;
  }

  std::vector<bool> range_index;
  double min_time = GetRangeIndex(sp_range, &range_index, sp_vec_poses);
  if (min_time == std::numeric_limits<double>::max() ||
      range_index.size() == 0) {
    AINFO << "min_time: " << min_time
          << ", range_index size: " << range_index.size();
    return -1;
  }
  std::vector<FramePose> &vec_poses = *sp_vec_poses;
  size_t pose_size = vec_poses.size();
  size_t range_size = range_index.size();
  std::vector<FramePose> &poses = *sp_poses;
  poses.clear();
  for (size_t i = 0; i < pose_size; ++i) {
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

int LoopsVerifyAgent::DoStartVerify(
    std::shared_ptr<std::vector<std::pair<double, double>>> sp_range,
    double loops_to_check) {
  clock_t start = clock();
  std::vector<FramePose> all_poses;
  GetPosesToCheck(sp_range, &all_poses);

  sp_laps_checker_ = std::shared_ptr<LapsChecker>(
      new LapsChecker(all_poses, static_cast<int>(loops_to_check), sp_conf_));
  sp_laps_checker_->Check();

  double duration = (static_cast<double>(clock() - start)) / CLOCKS_PER_SEC;
  AINFO << "checking laps cost " << duration << " seconds";
  return 0;
}

void LoopsVerifyAgent::SetState(LoopsVerifyAgentState state) { state_ = state; }
LoopsVerifyAgentState LoopsVerifyAgent::GetState() { return state_; }

}  // namespace hdmap
}  // namespace apollo
