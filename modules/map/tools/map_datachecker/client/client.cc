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
#include <vector>

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
  data_collect_time_flag_file_ =
      bin_path + "/" + node["time_flag_file"].as<std::string>();
  channel_checker_stop_flag_file_ =
      bin_path + "/" +
      node["channel_check"]["stop_flag_file"].as<std::string>();
  AINFO << "bin_path: " << bin_path
        << ", data_collect_time_flag_file_: " << data_collect_time_flag_file_
        << ", channel_checker_stop_flag_file_: "
        << channel_checker_stop_flag_file_;
}

int Client::Run() {
  std::string stage = FLAGS_stage;
  AINFO << "stage [" << stage << "]";
  int ret = 0;
  if ("record_check" == stage) {
    ret = RecordCheckStage();
  } else if ("static_align" == stage) {
    ret = StaticAlignStage();
  } else if ("eight_route" == stage) {
    ret = EightRouteStage();
  } else if ("data_collect" == stage) {
    ret = DataCollectStage();
  } else if ("loops_check" == stage) {
    ret = LoopsCheckStage();
  } else if ("clean" == stage) {
    ret = CleanStage();
  }
  return ret;
}

int Client::RecordCheckStage() {
  std::string cmd = FLAGS_cmd;
  ChannelChecker channel_checker(channel_checker_stop_flag_file_);
  AINFO << "cmd [" << cmd << "]";
  if ("start" == cmd) {
    std::string record_path = FLAGS_record_path;
    AINFO << "record_path [" << record_path << "]";
    if (!boost::filesystem::exists(record_path)) {
      AERROR << "record_path does not exist";
      return -1;
    }
    int ret = 0;
    ret = channel_checker.SyncStart(record_path);
    if (ret != 0) {
      AERROR << "SyncStart channel chacker failed, record_path [" << record_path
             << "]";
      return -1;
    }
  } else {
    int ret = 0;
    ret = channel_checker.SyncStop();
    if (ret != 0) {
      AERROR << "SyncStop channel chacker failed";
      return -1;
    }
  }
  return 0;
}

int Client::StaticAlignStage() {
  std::string cmd = FLAGS_cmd;
  AINFO << "cmd [" << cmd << "]";
  StaticAlign static_align;
  if ("start" == cmd) {
    int ret = 0;
    ret = static_align.SyncStart();
    if (ret != 0) {
      AERROR << "SyncStart static align failed";
      return -1;
    } else {
      AINFO << "Static aligh succeed";
      fprintf(USER_STREAM,
              "Static aligh succeed. Next, you may want to run: bash client.sh "
              "-- stage eight_route\n");
    }
  } else {
    int ret = 0;
    ret = static_align.SyncStop();
    if (ret != 0) {
      AERROR << "SyncStop static align failed";
      return -1;
    }
  }
  return 0;
}

int Client::EightRouteStage() {
  std::string cmd = FLAGS_cmd;
  AINFO << "cmd [" << cmd << "]";
  EightRoute eight_route;
  if ("start" == cmd) {
    int ret = 0;
    ret = eight_route.SyncStart();
    if (ret != 0) {
      AERROR << "SyncStart static align failed";
      return -1;
    } else {
      AINFO << "Eight route succeed";
      fprintf(USER_STREAM,
              "Eight route succeed. Next, you may want to run: bash client.sh "
              "--stage data_collect\n");
    }
  } else {
    int ret = 0;
    ret = eight_route.SyncStop();
    if (ret != 0) {
      AERROR << "SyncStop static align failed";
      return -1;
    }
  }
  return 0;
}

int Client::DataCollectStage() {
  std::string cmd = FLAGS_cmd;
  AINFO << "cmd [" << cmd << "]";
  std::vector<std::string> lines = GetFileLines(data_collect_time_flag_file_);
  std::ofstream time_file_handler(data_collect_time_flag_file_);
  double now = UnixtimeNow();
  if (cmd == "start") {
    if (lines.size() == 0) {
      time_file_handler << std::to_string(now) << " start\n";
      AINFO << "write [" << std::to_string(now) << " start] to file "
            << data_collect_time_flag_file_;
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
              << data_collect_time_flag_file_;
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
              << data_collect_time_flag_file_;
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

int Client::LoopsCheckStage() {
  LoopsChecker loops_checker(data_collect_time_flag_file_);
  bool reached = false;
  int ret = loops_checker.SyncStart(&reached);
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

int Client::CleanStage() {
  if (boost::filesystem::exists(data_collect_time_flag_file_)) {
    boost::filesystem::remove(data_collect_time_flag_file_);
    AINFO << "removed " << data_collect_time_flag_file_;
  }
  if (boost::filesystem::exists(channel_checker_stop_flag_file_)) {
    boost::filesystem::remove(channel_checker_stop_flag_file_);
    AINFO << "removed " << channel_checker_stop_flag_file_;
  }
  fprintf(USER_STREAM, "clean done\n");
  return 0;
}
}  // namespace hdmap
}  // namespace apollo
