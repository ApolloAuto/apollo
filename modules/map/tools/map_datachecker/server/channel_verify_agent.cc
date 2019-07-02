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
#include "modules/map/tools/map_datachecker/server/channel_verify_agent.h"

#include <chrono>
#include <map>
#include <string>
#include <thread>
#include <utility>
#include <vector>

namespace apollo {
namespace hdmap {

ChannelVerifyAgent::ChannelVerifyAgent(std::shared_ptr<JSonConf> sp_conf) {
  sp_conf_ = sp_conf;
  sp_channel_checker_ = nullptr;
  sp_check_result_ = nullptr;
}

void ChannelVerifyAgent::Reset() {
  std::lock_guard<std::mutex> guard(stop_mutex_);
  need_stop_ = false;
  stopped_ = false;
  sp_channel_checker_ = std::make_shared<ChannelVerify>(sp_conf_);
  sp_check_result_ = nullptr;
  SetState(ChannelVerifyAgentState::IDLE);
}

grpc::Status ChannelVerifyAgent::ProcessGrpcRequest(
    grpc::ServerContext *context, ChannelVerifyRequest *request,
    ChannelVerifyResponse *response) {
  AINFO << "ChannelVerifyAgent Request: " << request->DebugString();
  switch (request->cmd()) {
    case CmdType::START:
      AINFO << "ChannelVerifyAgent start";
      StartCheck(request, response);
      break;
    case CmdType::CHECK:
      AINFO << "ChannelVerifyAgent check";
      CheckResult(request, response);
      break;
    case CmdType::STOP:
      AINFO << "ChannelVerifyAgent stop";
      StopCheck(request, response);
      break;
    default:
      response->set_code(ErrorCode::ERROR_REQUEST);
      AERROR << "command error";
  }
  AINFO << "ChannelVerifyAgent progress: " << response->DebugString();
  return grpc::Status::OK;
}

void ChannelVerifyAgent::StartCheck(ChannelVerifyRequest *request,
                                    ChannelVerifyResponse *response) {
  if (GetState() == ChannelVerifyAgentState::RUNNING) {
    AINFO << "ChannelVerify is RUNNING, do not need start again";
    response->set_code(ErrorCode::ERROR_REPEATED_START);
    return;
  }
  Reset();
  std::string records_path = request->path();
  AsyncCheck(records_path);
  response->set_code(ErrorCode::SUCCESS);
}

void ChannelVerifyAgent::AsyncCheck(const std::string &records_path) {
  SetState(ChannelVerifyAgentState::RUNNING);
  std::thread doctor_strange([=]() {
    check_thread_id_ = std::this_thread::get_id();
    int wait_sec = sp_conf_->channel_check_trigger_gap;
    while (true) {
      {
        std::lock_guard<std::mutex> guard(stop_mutex_);
        if (need_stop_) {
          break;
        }
        DoCheck(records_path);
        AINFO << "thread check done";
      }
      std::this_thread::sleep_for(std::chrono::seconds(wait_sec));
      if (std::this_thread::get_id() != check_thread_id_) {
        break;
      }
    }
    stopped_ = true;
  });
  doctor_strange.detach();
  AINFO << "ChannelVerifyAgent::async_check exit";
}

void ChannelVerifyAgent::DoCheck(const std::string &records_path) {
  if (sp_channel_checker_ == nullptr) {
    sp_channel_checker_ = std::make_shared<ChannelVerify>(sp_conf_);
  }
  SetState(ChannelVerifyAgentState::RUNNING);
  sp_channel_checker_->Check(records_path);
  sp_check_result_ = sp_channel_checker_->get_check_result();
}

int ChannelVerifyAgent::AddTopicLack(
    VerifyResult *result, const std::string &record_path,
    std::vector<std::string> const &lack_channels) {
  TopicResult *topics = result->mutable_topics();
  for (const std::string &channel : lack_channels) {
    topics->add_topic_lack(channel);
    AINFO << record_path << " lack topic: " << channel;
  }
  return static_cast<int>(lack_channels.size());
}

FrameRate *ChannelVerifyAgent::FindRates(VerifyResult *result,
                                         const std::string &channel) {
  for (FrameRate &rate : *result->mutable_rates()) {
    if (rate.topic() == channel) {
      return &rate;
    }
  }
  return nullptr;
}

int ChannelVerifyAgent::AddInadequateRate(
    VerifyResult *result, std::string const &record_path,
    std::map<std::string, std::pair<double, double>> const &inadequate_rate) {
  for (auto it = inadequate_rate.begin(); it != inadequate_rate.end(); ++it) {
    const std::string &channel = it->first;
    double expected_rate = it->second.first;
    double current_rate = it->second.second;
    FrameRate *rate = FindRates(result, channel);
    if (rate == nullptr) {
      rate = result->add_rates();
      rate->add_bad_record_name(record_path);
      rate->set_topic(channel);
      rate->set_expected_rate(expected_rate);
      rate->set_current_rate(current_rate);
    } else {
      rate->set_current_rate(current_rate);
      rate->add_bad_record_name(record_path);
    }
  }
  return result->rates_size();
}

void ChannelVerifyAgent::CheckResult(ChannelVerifyRequest *request,
                                     ChannelVerifyResponse *response) {
  if (GetState() == ChannelVerifyAgentState::IDLE) {
    AINFO << "ChannelVerify is not RUNNING, it should start first";
    response->set_code(ErrorCode::ERROR_CHECK_BEFORE_START);
    return;
  }

  if (sp_channel_checker_ == nullptr || sp_check_result_ == nullptr) {
    AINFO << "check result is null. check later";
    response->set_code(ErrorCode::SUCCESS);
    return;
  }

  if (sp_channel_checker_->GetReturnState() != ErrorCode::SUCCESS) {
    response->set_code(sp_channel_checker_->GetReturnState());
    return;
  }

  response->set_code(ErrorCode::SUCCESS);
  VerifyResult *result = response->mutable_result();
  for (CheckResultIterator it = sp_check_result_->begin();
       it != sp_check_result_->end(); ++it) {
    int res = 0;
    // write rate
    res = AddInadequateRate(result, it->record_path, it->inadequate_rate);
    if (res > 0) {
      response->set_code(ErrorCode::ERROR_CHANNEL_VERIFY_RATES_ABNORMAL);
    }
    // write topic lack
    res = AddTopicLack(result, it->record_path, it->lack_channels);
    if (res > 0) {
      response->set_code(ErrorCode::ERROR_CHANNEL_VERIFY_TOPIC_LACK);
    }
  }
}

void ChannelVerifyAgent::StopCheck(ChannelVerifyRequest *request,
                                   ChannelVerifyResponse *response) {
  std::lock_guard<std::mutex> guard(stop_mutex_);
  need_stop_ = true;
  response->set_code(ErrorCode::SUCCESS);
  SetState(ChannelVerifyAgentState::IDLE);
  VerifyResult *result = response->mutable_result();
  if (sp_check_result_ == nullptr) {
    return;
  }
  for (CheckResultIterator it = sp_check_result_->begin();
       it != sp_check_result_->end(); ++it) {
    int res = 0;
    // write rate
    res = AddInadequateRate(result, it->record_path, it->inadequate_rate);
    if (res > 0) {
      response->set_code(ErrorCode::ERROR_CHANNEL_VERIFY_RATES_ABNORMAL);
    }
    // write topic lack
    res = AddTopicLack(result, it->record_path, it->lack_channels);
    if (res > 0) {
      response->set_code(ErrorCode::ERROR_CHANNEL_VERIFY_TOPIC_LACK);
    }
  }
}

void ChannelVerifyAgent::SetState(ChannelVerifyAgentState state) {
  state_ = state;
}

ChannelVerifyAgentState ChannelVerifyAgent::GetState() const { return state_; }

}  // namespace hdmap
}  // namespace apollo
