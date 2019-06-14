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
#ifndef _MODULES_MAP_TOOLS_MAP_DATACHECKER_CHANNEL_CHEKCER_AGENT_H
#define _MODULES_MAP_TOOLS_MAP_DATACHECKER_CHANNEL_CHEKCER_AGENT_H
#include <grpc++/grpc++.h>
#include <memory>
#include <mutex>
#include <utility>
#include <string>
#include <vector>
#include <map>
#include "modules/map/tools/map_datachecker/channel_verify.h"
#include "modules/map/tools/map_datachecker/proto/collection_service.pb.h"
#include "modules/map/tools/map_datachecker/proto/collection_error_code.pb.h"

namespace apollo {
namespace hdmap {

enum class ChannelVerifyAgentState {
  IDLE,
  RUNNING
};

using CHANNEL_VERIFY_REQUEST_TYPE = const apollo::hdmap::ChannelVerifyRequest;
using CHANNEL_VERIFY_RESPONSE_TYPE = apollo::hdmap::ChannelVerifyResponse;

class ChannelVerifyAgent {
 public:
  explicit ChannelVerifyAgent(std::shared_ptr<JSonConf> sp_conf);
  grpc::Status process_grpc_request(
    grpc::ServerContext *context,
    CHANNEL_VERIFY_REQUEST_TYPE *request,
    CHANNEL_VERIFY_RESPONSE_TYPE *response);

 private:
  void start_check(
    CHANNEL_VERIFY_REQUEST_TYPE *request,
    CHANNEL_VERIFY_RESPONSE_TYPE *response);
  void async_check(const std::string& records_path);
  void do_check(const std::string& records_path);
  void check_result(
    CHANNEL_VERIFY_REQUEST_TYPE *request,
    CHANNEL_VERIFY_RESPONSE_TYPE *response);
  void stop_check(
    CHANNEL_VERIFY_REQUEST_TYPE *request,
    CHANNEL_VERIFY_RESPONSE_TYPE *response);
  void reset();
  void set_state(ChannelVerifyAgentState state);
  ChannelVerifyAgentState get_state();
  int add_topic_lack(
    apollo::hdmap::VerifyResult *result,
    const std::string& record_path,
    std::vector<std::string> const& lack_channels);
  int add_inadequate_rate(
    apollo::hdmap::VerifyResult *result,
    std::string const & record_path,
    std::map<std::string,
    std::pair<double, double>> const& inadequate_rate);
  apollo::hdmap::FrameRate* find_rates(
    apollo::hdmap::VerifyResult *result,
    const std::string& channel);

 private:
  ChannelVerifyAgentState _state;
  std::mutex _stop_mutex;
  bool _need_stop, _stopped;
  std::shared_ptr<JSonConf> _sp_conf = nullptr;
  std::shared_ptr<ChannelVerify> _sp_channel_checker = nullptr;
  CheckResult _sp_check_result = nullptr;
  std::thread::id _check_thread_id;
};

}  // namespace hdmap
}  // namespace apollo

#endif  // _MODULES_MAP_TOOLS_MAP_DATACHECKER_CHANNEL_CHEKCER_AGENT_H
