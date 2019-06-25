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
#include "modules/map/tools/map_datachecker/client/client_channel_checker.h"
#include <grpc++/grpc++.h>
#include <yaml-cpp/yaml.h>
#include <boost/filesystem.hpp>
#include <chrono>
#include <fstream>
#include <memory>
#include <string>
#include "cyber/cyber.h"
#include "modules/map/tools/map_datachecker/client/client_common.h"
#include "modules/map/tools/map_datachecker/client/client_gflags.h"
#include "modules/map/tools/map_datachecker/client/exception_handler.h"
#include "modules/map/tools/map_datachecker/proto/collection_service.grpc.pb.h"
namespace apollo {
namespace hdmap {

ChannelChecker::ChannelChecker(const std::string& stop_flag_file)
    : _stop_flag_file(stop_flag_file) {
  YAML::Node node = YAML::LoadFile(FLAGS_client_conf_yaml);
  std::string server_addr =
      node["grpc_host_port"]["grpc_host"].as<std::string>() + ":" +
      node["grpc_host_port"]["grpc_port"].as<std::string>();
  _check_period = node["channel_check"]["check_period"].as<int>();
  _service_stub = CollectionCheckerService::NewStub(
      grpc::CreateChannel(server_addr, grpc::InsecureChannelCredentials()));
}

int ChannelChecker::sync_start(const std::string& record_path) {
  if (!boost::filesystem::exists(record_path)) {
    AERROR << "record_path [" << record_path << "]is not exist";
    return -1;
  }
  int ret = start(record_path);
  if (ret != 0) {
    AERROR << "start check channel failed, record_path [" << record_path << "]";
    return -1;
  }
  return periodic_check();
}

int ChannelChecker::sync_stop() {
  // stop client
  std::ofstream ofs(_stop_flag_file);
  if (!ofs) {
    AERROR << "Create Flag File [" << _stop_flag_file << "] Failed";
    return -1;
  }
  ofs.close();
  // stop server
  return stop();
}

int ChannelChecker::periodic_check() {
  int ret = 0;
  while (!boost::filesystem::exists(_stop_flag_file)) {
    ret = check();
    if (ret != 0) {
      break;
    }
    AINFO << "channel checker sleep " << _check_period << " seconds";
    std::this_thread::sleep_for(std::chrono::seconds(_check_period));
  }
  if (ret == 0) {
    AINFO << "detected stop flag file, periodically checking will exit";
  } else {
    AINFO << "periodically checking will exit because of some errors";
  }
  return ret;
}

int ChannelChecker::grpc_stub(CHANNEL_VERIFY_REQUEST_TYPE* request,
                              CHANNEL_VERIFY_RESPONSE_TYPE* response) {
  grpc::ClientContext context;
  grpc::Status status;
  status = _service_stub->ChannelVerify(&context, *request, response);
  if (status.error_code() == grpc::StatusCode::UNAVAILABLE) {
    AERROR << "FATAL Error. Map grpc service is UNAVAILABLE.";
    fprintf(USER_STREAM, "You should start server first\n");
    return -1;
  }
  ErrorCode error_code = response->code();
  if (error_code != ErrorCode::SUCCESS) {
    return ExceptionHandler::exception_handler(error_code);
  }
  return 0;
}

int ChannelChecker::start(const std::string& record_path) {
  CHANNEL_VERIFY_REQUEST_TYPE request;
  request.set_path(record_path);
  request.set_cmd(CmdType::START);
  AINFO << "channel check request: "
        << "record_path: [" << request.path() << "], "
        << "cmd: [" << request.cmd() << "]";
  CHANNEL_VERIFY_RESPONSE_TYPE response;
  return grpc_stub(&request, &response);
}

int ChannelChecker::check() {
  CHANNEL_VERIFY_REQUEST_TYPE request;
  request.set_cmd(CmdType::CHECK);
  AINFO << "channel check request: "
        << "cmd: [" << request.cmd() << "]";
  CHANNEL_VERIFY_RESPONSE_TYPE response;
  int ret = grpc_stub(&request, &response);
  if (ret != 0) {
    process_abnormal(&response);
  }
  return ret;
}

int ChannelChecker::stop() {
  CHANNEL_VERIFY_REQUEST_TYPE request;
  request.set_cmd(CmdType::STOP);
  AINFO << "channel check request: "
        << "cmd: [" << request.cmd() << "]";
  CHANNEL_VERIFY_RESPONSE_TYPE response;
  return grpc_stub(&request, &response);
}

int ChannelChecker::process_abnormal(CHANNEL_VERIFY_RESPONSE_TYPE* response) {
  ErrorCode code = response->code();
  if (code == ErrorCode::ERROR_CHANNEL_VERIFY_RATES_ABNORMAL) {
    if (response->has_result()) {
      const VerifyResult& result = response->result();
      for (int i = 0; i < result.rates_size(); ++i) {
        const FrameRate& rate = result.rates(i);
        fprintf(USER_STREAM, "Channels with insufficient frame rate:%s\n",
                rate.topic().c_str());
        fprintf(USER_STREAM, "-expected_rate:%lf\n", rate.expected_rate());
        fprintf(USER_STREAM, "-current_rate:%lf\n", rate.current_rate());
        fprintf(USER_STREAM, "-bad_record_name:\n");
        for (int j = 0; j < rate.bad_record_name_size(); ++j) {
          fprintf(USER_STREAM, "--%s\n", rate.bad_record_name(j).c_str());
        }
      }
    } else {
      AERROR << "response content from server is inconsistent";
      return -1;
    }
  } else if (code == ErrorCode::ERROR_CHANNEL_VERIFY_TOPIC_LACK) {
    if (response->has_result() && response->result().has_topics()) {
      const TopicResult& topics = response->result().topics();
      fprintf(USER_STREAM, "Missing channel(s):\n");
      for (int i = 0; i < topics.topic_lack_size(); ++i) {
        fprintf(USER_STREAM, "-%s:\n", topics.topic_lack(i).c_str());
      }
    } else {
      AERROR << "response content from server is inconsistent";
      return -1;
    }
  }
  return 0;
}

}  // namespace hdmap
}  // namespace apollo
