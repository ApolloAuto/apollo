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
#include "modules/map/tools/map_datachecker/client/client_loops_check.h"
#include <grpc++/grpc++.h>
#include <yaml-cpp/yaml.h>
#include <boost/algorithm/string.hpp>
#include <utility>
namespace apollo {
namespace hdmap {
LoopsChecker::LoopsChecker(const std::string& time_flag_file)
    : _time_flag_file(time_flag_file) {
  YAML::Node node = YAML::LoadFile(FLAGS_client_conf_yaml);
  std::string server_addr =
      node["grpc_host_port"]["grpc_host"].as<std::string>() + ":" +
      node["grpc_host_port"]["grpc_port"].as<std::string>();
  _check_period = node["loops_check"]["check_period"].as<int>();
  _service_stub = CollectionCheckerService::NewStub(
      grpc::CreateChannel(server_addr, grpc::InsecureChannelCredentials()));
}
int LoopsChecker::sync_start(bool* reached) {
  std::vector<std::pair<double, double>> time_ranges = get_time_ranges();
  size_t pair_count = time_ranges.size();
  if (pair_count == 0) {
    AINFO << "there is no time range";
    return -1;
  }
  int ret = start(time_ranges);
  if (ret != 0) {
    AINFO << "loops check start failed";
    fprintf(USER_STREAM, "loops check start failed\n");
    return -1;
  }
  return periodic_check(reached);
}
int LoopsChecker::periodic_check(bool* reached) {
  int ret = 0;
  while (true) {
    double progress = 0.0;
    ret = check(&progress, reached);
    if (ret != 0) {
      AERROR << "loops check failed";
      break;
    }
    AINFO << "loops check progress: [" << progress << "]";
    fprintf(USER_STREAM, "loops check progress: %.2lf%%\n", progress * 100);
    if (fabs(progress - 1.0) < 1e-8) {
      break;
    }
    std::this_thread::sleep_for(std::chrono::seconds(_check_period));
  }
  if (ret != 0) {
    return -1;
  }
  ret = stop();
  if (ret != 0) {
    AERROR << "loops check stop failed";
    return -1;
  }
  return 0;
}
std::vector<std::pair<double, double>> LoopsChecker::get_time_ranges() {
  std::vector<std::pair<double, double>> result;
  std::vector<std::pair<double, double>> empty;
  std::vector<std::string> lines = get_file_lines(_time_flag_file);
  size_t record_count = lines.size();
  if (record_count == 0 || (record_count & 1) != 0) {
    AINFO << "record_count should be even number";
    fprintf(USER_STREAM,
            "The command start and stop should be appear in pairs\n");
    return empty;
  }
  for (size_t i = 0; i < record_count; i += 2) {
    std::vector<std::string> split_str;
    boost::split(split_str, lines[i], boost::is_any_of(" ,\t\n"));
    double start_time = 0.0;
    sscanf(split_str[0].c_str(), "%lf", &start_time);
    if (split_str[1] != "start") {
      AERROR << "state machine was broken";
      fprintf(USER_STREAM,
              "The data collect command start and stop should be appear in "
              "pairs\n");
      return empty;
    }
    boost::split(split_str, lines[i + 1], boost::is_any_of(" ,\t\n"));
    double end_time = 0.0;
    sscanf(split_str[0].c_str(), "%lf", &end_time);
    if (split_str[1] != "stop") {
      AERROR << "state machine was broken";
      fprintf(USER_STREAM,
              "The data collect command start and stop should be appear in "
              "pairs\n");
      return empty;
    }
    result.push_back(std::make_pair(start_time, end_time));
  }
  return result;
}
int LoopsChecker::grpc_stub(LOOPS_VERIFY_REQUEST_TYPE* request,
                            LOOPS_VERIFY_RESPONSE_TYPE* response) {
  grpc::ClientContext context;
  grpc::Status status;
  status = _service_stub->LoopsVerify(&context, *request, response);
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
int LoopsChecker::start(
    const std::vector<std::pair<double, double>>& time_ranges) {
  LOOPS_VERIFY_REQUEST_TYPE request;
  request.set_cmd(CmdType::START);
  for (size_t i = 0; i < time_ranges.size(); i++) {
    VerifyRange* range = request.add_range();
    range->set_start_time(time_ranges[i].first);
    range->set_end_time(time_ranges[i].second);
    AINFO << "range[" << i << "] is [" << std::to_string(time_ranges[i].first)
          << "," << std::to_string(time_ranges[i].second) << "]";
  }
  LOOPS_VERIFY_RESPONSE_TYPE response;
  return grpc_stub(&request, &response);
}
int LoopsChecker::check(double* progress, bool* reached) {
  LOOPS_VERIFY_REQUEST_TYPE request;
  request.set_cmd(CmdType::CHECK);
  AINFO << "loops check request: "
        << "cmd: [" << request.cmd() << "]";
  LOOPS_VERIFY_RESPONSE_TYPE response;
  int ret = grpc_stub(&request, &response);
  *progress = response.progress();
  if (response.has_loop_result() && response.loop_result().has_is_reached()) {
    *reached = response.loop_result().is_reached();
  }
  return ret;
}
int LoopsChecker::stop() {
  LOOPS_VERIFY_REQUEST_TYPE request;
  request.set_cmd(CmdType::STOP);
  AINFO << "loops check request: "
        << "cmd: [" << request.cmd() << "]";
  LOOPS_VERIFY_RESPONSE_TYPE response;
  return grpc_stub(&request, &response);
}

}  // namespace hdmap
}  // namespace apollo
