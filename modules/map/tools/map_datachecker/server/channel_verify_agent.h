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
#include <map>
#include <memory>
#include <mutex>
#include <string>
#include <utility>
#include <vector>

#include "modules/map/tools/map_datachecker/proto/collection_error_code.pb.h"
#include "modules/map/tools/map_datachecker/proto/collection_service.pb.h"
#include "modules/map/tools/map_datachecker/server/channel_verify.h"

namespace apollo {
namespace hdmap {

enum class ChannelVerifyAgentState { IDLE, RUNNING };

class ChannelVerifyAgent {
 public:
  explicit ChannelVerifyAgent(std::shared_ptr<JSonConf> sp_conf);
  grpc::Status process_grpc_request(grpc::ServerContext *context,
                                    ChannelVerifyRequest *request,
                                    ChannelVerifyResponse *response);

 private:
  void start_check(ChannelVerifyRequest *request,
                   ChannelVerifyResponse *response);
  void async_check(const std::string &records_path);
  void do_check(const std::string &records_path);
  void check_result(ChannelVerifyRequest *request,
                    ChannelVerifyResponse *response);
  void stop_check(ChannelVerifyRequest *request,
                  ChannelVerifyResponse *response);
  void reset();
  void set_state(ChannelVerifyAgentState state);
  ChannelVerifyAgentState get_state() const;
  int add_topic_lack(apollo::hdmap::VerifyResult *result,
                     const std::string &record_path,
                     std::vector<std::string> const &lack_channels);
  int add_inadequate_rate(
      apollo::hdmap::VerifyResult *result, std::string const &record_path,
      std::map<std::string, std::pair<double, double>> const &inadequate_rate);
  apollo::hdmap::FrameRate *find_rates(apollo::hdmap::VerifyResult *result,
                                       const std::string &channel);

 private:
  ChannelVerifyAgentState state_;
  std::mutex stop_mutex_;
  bool need_stop_;
  bool stopped_;
  std::shared_ptr<JSonConf> sp_conf_ = nullptr;
  std::shared_ptr<ChannelVerify> sp_channel_checker_ = nullptr;
  CheckResult sp_check_result_ = nullptr;
  std::thread::id check_thread_id_;
};

}  // namespace hdmap
}  // namespace apollo
