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
#include "modules/map/tools/map_datachecker/client/client.h"
#include <yaml-cpp/yaml.h>
#include <boost/algorithm/string.hpp>
#include <boost/filesystem.hpp>
#include <string>
#include "cyber/cyber.h"
#include "modules/map/tools/map_datachecker/client/client_alignment.h"
#include "modules/map/tools/map_datachecker/client/client_channel_checker.h"
#include "modules/map/tools/map_datachecker/client/client_common.h"
#include "modules/map/tools/map_datachecker/client/client_loops_check.h"
namespace apollo {
namespace hdmap {

Client::Client() {
  YAML::Node node = YAML::LoadFile(FLAGS_client_conf_yaml);
  std::string bin_path = boost::filesystem::current_path().string();
  _data_collect_time_flag_file =
      bin_path + "/" + node["time_flag_file"].as<std::string>();
  _channel_checker_stop_flag_file =
      bin_path + "/" +
      node["channel_check"]["stop_flag_file"].as<std::string>();
  AINFO << "bin_path: " << bin_path
        << ", _data_collect_time_flag_file: " << _data_collect_time_flag_file
        << ", _channel_checker_stop_flag_file: "
        << _channel_checker_stop_flag_file;
}

int Client::run() {
  std::string stage = FLAGS_stage;
  AINFO << "stage [" << stage << "]";
  if ("record_check" == stage) {
    record_check();
  } else if ("static_align" == stage) {
    static_align();
  } else if ("eight_route" == stage) {
    eight_route();
  } else if ("data_collect" == stage) {
    data_collect();
  } else if ("loops_check" == stage) {
    loops_check();
  } else if ("clean" == stage) {
    clean();
  }
  return 0;
}

int Client::record_check() {
  std::string cmd = FLAGS_cmd;
  ChannelChecker channel_checker(_channel_checker_stop_flag_file);
  AINFO << "cmd [" << cmd << "]";
  if ("start" == cmd) {
    std::string record_path = FLAGS_record_path;
    AINFO << "record_path [" << record_path << "]";
    if (!boost::filesystem::exists(record_path)) {
      AERROR << "record_path is not exist";
      return -1;
    }
    int ret = 0;
    ret = channel_checker.sync_start(record_path);
    if (ret != 0) {
      AERROR << "sync_start channel chacker failed, record_path ["
             << record_path << "]";
      return -1;
    }
  } else {
    int ret = 0;
    ret = channel_checker.sync_stop();
    if (ret != 0) {
      AERROR << "sync_stop channel chacker failed";
      return -1;
    }
  }
  return 0;
}

int Client::static_align() {
  std::string cmd = FLAGS_cmd;
  AINFO << "cmd [" << cmd << "]";
  StaticAlign static_align;
  if ("start" == cmd) {
    int ret = 0;
    ret = static_align.sync_start();
    if (ret != 0) {
      AERROR << "sync_start static align failed";
      return -1;
    } else {
      AINFO << "Static aligh succeed";
      fprintf(USER_STREAM,
              "Static aligh succeed. Next, you may want to run: bash client.sh "
              "-- stage eight_route\n");
    }
  } else {
    int ret = 0;
    ret = static_align.sync_stop();
    if (ret != 0) {
      AERROR << "sync_stop static align failed";
      return -1;
    }
  }
  return 0;
}

int Client::eight_route() {
  std::string cmd = FLAGS_cmd;
  AINFO << "cmd [" << cmd << "]";
  EightRoute eight_route;
  if ("start" == cmd) {
    int ret = 0;
    ret = eight_route.sync_start();
    if (ret != 0) {
      AERROR << "sync_start static align failed";
      return -1;
    } else {
      AINFO << "Eight route succeed";
      fprintf(USER_STREAM,
              "Eight route succeed. Next, you may want to run: bash client.sh "
              "--stage data_collect\n");
    }
  } else {
    int ret = 0;
    ret = eight_route.sync_stop();
    if (ret != 0) {
      AERROR << "sync_stop static align failed";
      return -1;
    }
  }
  return 0;
}

int Client::data_collect() {
  std::string cmd = FLAGS_cmd;
  AINFO << "cmd [" << cmd << "]";
  std::vector<std::string> lines = get_file_lines(_data_collect_time_flag_file);
  std::ofstream time_file_handler(_data_collect_time_flag_file);
  double now = unixtime_now();
  if (cmd == "start") {
    if (lines.size() == 0) {
      time_file_handler << std::to_string(now) << " start\n";
      AINFO << "write [" << std::to_string(now) << " start] to file "
            << _data_collect_time_flag_file;
      fprintf(USER_STREAM,
              "Start success. At the end of the collection, you should run: "
              "bash client.sh data_collect stop\n");
    } else {
      std::string& the_last_line = lines.back();
      std::vector<std::string> s;
      boost::split(s, the_last_line, boost::is_any_of(" ,\t\n"));
      if (s[1] == "start") {
        AINFO << "This progress has been already started, this command will be "
                 "ignored";
        fprintf(USER_STREAM,
                "This progress has been already started, this command will be "
                "ignored\n");
      } else {
        time_file_handler << std::to_string(now) << " start\n";
        AINFO << "write [" << std::to_string(now) << " start] to file "
              << _data_collect_time_flag_file;
        fprintf(USER_STREAM,
                "Start success. At the end of the collection, you should run: "
                "bash client.sh --stage data_collect --cmd stop\n");
      }
    }
  } else if (cmd == "stop") {
    if (lines.size() == 0) {
      AINFO << "Start first, this command will be ignored";
    } else {
      std::string& the_last_line = lines.back();
      std::vector<std::string> s;
      boost::split(s, the_last_line, boost::is_any_of(" ,\t\n"));
      if (s[1] == "start") {
        time_file_handler << std::to_string(now) << " stop\n";
        AINFO << "write [" << std::to_string(now) << " stop] to file "
              << _data_collect_time_flag_file;
        fprintf(USER_STREAM,
                "Stop success. Next you may want to run: bash client.sh "
                "loops_check start\n");
      } else {
        AINFO << "This progress has been already stopped, this command will be "
                 "ignored";
        fprintf(USER_STREAM,
                "This progress has been already stopped, this command will be "
                "ignored\n");
      }
    }
  } else {
    AINFO << "Error command, expected command are [start|stop], current "
             "command is ["
          << cmd << "]";
    fprintf(USER_STREAM,
            "Error command, expected command are [start|stop], current command "
            "is [%s]\n",
            cmd.c_str());
  }
  return 0;
}

int Client::loops_check() {
  LoopsChecker loops_checker(_data_collect_time_flag_file);
  bool reached = false;
  int ret = loops_checker.sync_start(reached);
  if (ret != 0) {
    AINFO << "loops_check failed";
    return -1;
  }
  if (reached) {
    AINFO << "loops meet requirements";
    fprintf(USER_STREAM,
            "Loops meet requirements. Next you may want to run: bash client.sh "
            "--stage eight_route\n");
  } else {
    AINFO << "loops do not meet requirements";
    fprintf(USER_STREAM,
            "Next you may need to run: bash client.sh --stage data_collect\n");
  }
  return 0;
}

int Client::clean() {
  if (boost::filesystem::exists(_data_collect_time_flag_file)) {
    boost::filesystem::remove(_data_collect_time_flag_file);
    AINFO << "removed " << _data_collect_time_flag_file;
  }
  if (boost::filesystem::exists(_channel_checker_stop_flag_file)) {
    boost::filesystem::remove(_channel_checker_stop_flag_file);
    AINFO << "removed " << _channel_checker_stop_flag_file;
  }
  fprintf(USER_STREAM, "clean done\n");
  return 0;
}
}  // namespace hdmap
}  // namespace apollo
