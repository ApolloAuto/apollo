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

#include <map>
#include <memory>
#include <mutex>
#include <string>
#include <utility>
#include <vector>

#include "grpc++/grpc++.h"

#include "modules/map/tools/map_datachecker/proto/collection_error_code.pb.h"
#include "modules/map/tools/map_datachecker/proto/collection_service.pb.h"
#include "modules/map/tools/map_datachecker/server/channel_verify.h"

namespace apollo {
namespace hdmap {

enum class ChannelVerifyAgentState { IDLE, RUNNING };

class ChannelVerifyAgent {
 public:
  explicit ChannelVerifyAgent(std::shared_ptr<JSonConf> sp_conf);
  grpc::Status ProcessGrpcRequest(grpc::ServerContext *context,
                                  ChannelVerifyRequest *request,
                                  ChannelVerifyResponse *response);

 private:
  void StartCheck(ChannelVerifyRequest *request,
                  ChannelVerifyResponse *response);
  void AsyncCheck(const std::string &records_path);
  void DoCheck(const std::string &records_path);
  void CheckResult(ChannelVerifyRequest *request,
                   ChannelVerifyResponse *response);
  void StopCheck(ChannelVerifyRequest *request,
                 ChannelVerifyResponse *response);
  void Reset();
  void SetState(ChannelVerifyAgentState state);
  ChannelVerifyAgentState GetState() const;
  int AddTopicLack(VerifyResult *result, const std::string &record_path,
                   std::vector<std::string> const &lack_channels);
  int AddInadequateRate(
      VerifyResult *result, std::string const &record_path,
      std::map<std::string, std::pair<double, double>> const &inadequate_rate);
  FrameRate *FindRates(VerifyResult *result, const std::string &channel);

 private:
  ChannelVerifyAgentState state_;
  std::mutex stop_mutex_;
  bool need_stop_;
  bool stopped_;
  std::shared_ptr<JSonConf> sp_conf_ = nullptr;
  std::shared_ptr<ChannelVerify> sp_channel_checker_ = nullptr;
  CheckedResult sp_check_result_ = nullptr;
  std::thread::id check_thread_id_;
};

}  // namespace hdmap
}  // namespace apollo
