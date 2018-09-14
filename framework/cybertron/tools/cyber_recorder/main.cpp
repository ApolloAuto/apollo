/******************************************************************************
 * Copyright 2018 The Apollo Authors. All Rights Reserved.
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

#include <getopt.h>
#include <stddef.h>
#include <memory>
#include <stdexcept>
#include <string>
#include <vector>

#include "cybertron/common/file.h"
#include "cybertron/common/time_conversion.h"
#include "cybertron/init.h"
#include "cybertron/tools/cyber_recorder/info.h"
#include "cybertron/tools/cyber_recorder/player.h"
#include "cybertron/tools/cyber_recorder/recorder.h"
#include "cybertron/tools/cyber_recorder/recoverer.h"
#include "cybertron/tools/cyber_recorder/spliter.h"

using apollo::cybertron::common::GetFileName;
using apollo::cybertron::common::StringToUnixSeconds;
using apollo::cybertron::common::UnixSecondsToString;
using apollo::cybertron::record::Info;
using apollo::cybertron::record::Player;
using apollo::cybertron::record::Recorder;
using apollo::cybertron::record::Recoverer;
using apollo::cybertron::record::Spliter;

const char* INFO_OPTIONS("f:ah");
const char* RECORD_OPTIONS("o:ac:h");
const char* PLAY_OPTIONS("f:ac:lr:b:e:s:d:h");
const char* SPLIT_OPTIONS("f:o:ac:b:e:h");
const char* RECOVER_OPTIONS("f:o:h");

void DisplayUsage(const std::string& binary);
void DisplayUsage(const std::string& binary, const std::string& command);
void DisplayUsage(const std::string& binary, const std::string& command,
                  const std::string& options);

void DisplayUsage(const std::string& binary) {
  std::cout << "usage: " << binary << " <command> [<args>]\n"
            << "The " << binary << " commands are:\n"
            << "\tinfo\tShow infomation of an exist record.\n"
            << "\tplay\tPlay an exist record.\n"
            << "\trecord\tRecord same topic.\n"
            << "\tsplit\tSplit an exist record.\n"
            << "\trecover\tRecover an exist record.\n"
            << std::endl;
}

void DisplayUsage(const std::string& binary, const std::string& command) {
  std::cout << "usage: " << binary << " " << command << " [options]"
            << std::endl;
  if (command == "info") {
    DisplayUsage(binary, command, INFO_OPTIONS);
  } else if (command == "record") {
    DisplayUsage(binary, command, RECORD_OPTIONS);
  } else if (command == "play") {
    DisplayUsage(binary, command, PLAY_OPTIONS);
  } else if (command == "split") {
    DisplayUsage(binary, command, SPLIT_OPTIONS);
  } else if (command == "recover") {
    DisplayUsage(binary, command, RECOVER_OPTIONS);
  } else {
    std::cout << "Unknown command: " << command << std::endl;
    DisplayUsage(binary);
  }
}

void DisplayUsage(const std::string& binary, const std::string& command,
                  const std::string& options) {
  for (char option : options) {
    switch (option) {
      case 'f':
        std::cout << "\t-f, --file <file>\t\t\tinput record file" << std::endl;
        break;
      case 'o':
        std::cout << "\t-o, --output <file>\t\t\toutput record file"
                  << std::endl;
        break;
      case 'a':
        std::cout << "\t-a, --all\t\t\t\t" << command << " all" << std::endl;
        break;
      case 'c':
        std::cout << "\t-c, --channel <name>\t\t\tonly " << command
                  << " the specified channel" << std::endl;
        break;
      case 'l':
        std::cout << "\t-l, --loop\t\t\t\tloop " << command << std::endl;
        break;
      case 'r':
        std::cout << "\t-r, --rate <1.0>\t\t\tmultiply the " << command
                  << " rate by FACTOR" << std::endl;
        break;
      case 'b':
        std::cout << "\t-b, --begin <2018-07-01 00:00:00>\t" << command
                  << " the record begin at" << std::endl;
        break;
      case 'e':
        std::cout << "\t-e, --end <2018-07-01 00:01:00>\t\t" << command
                  << " the record end at" << std::endl;
        break;
      case 's':
        std::cout << "\t-s, --start <seconds>\t\t\t" << command
                  << " started at n seconds" << std::endl;
        break;
      case 'd':
        std::cout << "\t-d, --delay <seconds>\t\t\t" << command
                  << " delayed n seconds" << std::endl;
        break;
      case 'h':
        std::cout << "\t-h, --help\t\t\t\tshow help message" << std::endl;
        break;
      case ':':
        break;
      default:
        std::cout << "unknown option: -" << option;
        break;
    }
  }
}

int main(int argc, char** argv) {
  std::string binary = GetFileName(std::string(argv[0]));
  if (argc < 2) {
    DisplayUsage(binary);
    return -1;
  }
  const std::string command(argv[1]);

  int long_index = 0;
  const std::string short_opts = "f:c:o:alr:b:e:s:d:h";
  static const struct option long_opts[] = {
      {"files", required_argument, NULL, 'f'},
      {"channel", required_argument, NULL, 'c'},
      {"output", required_argument, NULL, 'o'},
      {"all", no_argument, NULL, 'a'},
      {"loop", no_argument, NULL, 'l'},
      {"rate", required_argument, NULL, 'r'},
      {"begin", required_argument, NULL, 'b'},
      {"end", required_argument, NULL, 'e'},
      {"start", required_argument, NULL, 's'},
      {"delay", required_argument, NULL, 'd'},
      {"help", no_argument, NULL, 'h'}};

  std::vector<std::string> opt_file_vec;
  std::vector<std::string> opt_output_vec;
  std::vector<std::string> opt_channel_vec;
  bool opt_all = false;
  bool opt_loop = false;
  float opt_rate = 1.0f;
  uint64_t opt_begin = 0;
  uint64_t opt_end = UINT64_MAX;
  uint64_t opt_start = 0;
  uint64_t opt_delay = 0;

  do {
    int opt =
        getopt_long(argc, argv, short_opts.c_str(), long_opts, &long_index);
    if (opt == -1) {
      break;
    }
    switch (opt) {
      case 'f':
        opt_file_vec.push_back(std::string(optarg));
        break;
      case 'c':
        optind--;
        while (optind < argc) {
          if (*argv[optind] != '-') {
            opt_channel_vec.emplace_back(argv[optind]);
            optind++;
          } else {
            optind--;
            break;
          }
        }
        break;
      case 'o':
        opt_output_vec.push_back(std::string(optarg));
        break;
      case 'a':
        opt_all = true;
        break;
      case 'l':
        opt_loop = true;
        break;
      case 'r':
        try {
          opt_rate = std::stof(optarg);
        } catch (std::invalid_argument& ia) {
          std::cout << "Invalid argument: -r/--rate " << std::string(optarg)
                    << std::endl;
          return -1;
        }
        break;
      case 'b':
        opt_begin = StringToUnixSeconds(std::string(optarg)) * 1e9;
        break;
      case 'e':
        opt_end = StringToUnixSeconds(std::string(optarg)) * 1e9;
        break;
      case 's':
        try {
          opt_start = std::stoi(optarg);
        } catch (std::invalid_argument& ia) {
          std::cout << "Invalid argument: -s/--start " << std::string(optarg)
                    << std::endl;
          return -1;
        }
        break;
      case 'd':
        try {
          opt_delay = std::stoi(optarg);
        } catch (std::invalid_argument& ia) {
          std::cout << "Invalid argument: -d/--delay " << std::string(optarg)
                    << std::endl;
          return -1;
        }
        break;
      case 'h':
        DisplayUsage(binary, command);
        return 0;
      default:
        break;
    }
  } while (true);

  // cyber_recorder info
  if (command == "info") {
    if (opt_file_vec.empty()) {
      std::cout << "MUST specify file option (-f)." << std::endl;
      return -1;
    }
    ::apollo::cybertron::Init(argv[0]);
    Info info;
    bool info_result = true;
    for (auto& opt_file : opt_file_vec) {
      info_result =
          info_result && info.Display(opt_file, opt_all) ? true : false;
    }
    ::apollo::cybertron::Shutdown();
    return info_result ? 0 : -1;
  } else if (command == "recover") {
    if (opt_file_vec.empty()) {
      std::cout << "MUST specify file option (-f)." << std::endl;
      return -1;
    }
    if (opt_file_vec.size() > 1 || opt_output_vec.size() > 1) {
      std::cout << "TOO many input/output file option (-f/-o)." << std::endl;
      return -1;
    }
    if (opt_output_vec.empty()) {
      std::string default_output_file = opt_file_vec[0] + ".recover";
      opt_output_vec.push_back(default_output_file);
    }
    ::apollo::cybertron::Init(argv[0]);
    Recoverer recoverer(opt_file_vec[0], opt_output_vec[0]);
    bool recover_result = recoverer.Proc();
    ::apollo::cybertron::Shutdown();
    return recover_result ? 0 : -1;
  }

  // common check
  if (opt_channel_vec.empty() && !opt_all) {
    std::cout
        << "MUST specify channels option (-c) or all channels option (-a)."
        << std::endl;
    return -1;
  }

  if (command == "play") {
    if (opt_file_vec.empty()) {
      std::cout << "MUST specify file option (-f)." << std::endl;
      return -1;
    }
    ::apollo::cybertron::Init(argv[0]);
    // TODO @baownayu order input record file
    bool play_result = true;
    for (auto& opt_file : opt_file_vec) {
      Player player(opt_file_vec[0], opt_all, opt_channel_vec, opt_loop,
                    opt_rate, opt_begin, opt_end, opt_start, opt_delay);
      play_result = play_result && player.Init() ? true : false;
      play_result = play_result && player.Start() ? true : false;
    }
    ::apollo::cybertron::Shutdown();
    return play_result ? 0 : -1;
  } else if (command == "record") {
    if (opt_output_vec.size() > 1) {
      std::cout << "TOO many ouput file option (-o)." << std::endl;
      return -1;
    }
    if (opt_output_vec.empty()) {
      std::string default_output_file =
          UnixSecondsToString(time(nullptr), "%Y%m%d%H%M%S") + ".record";
      opt_output_vec.push_back(default_output_file);
    }
    bool record_result = true;
    ::apollo::cybertron::Init(argv[0]);
    auto recorder =
        std::make_shared<Recorder>(opt_output_vec[0], opt_all, opt_channel_vec);
    record_result = record_result && recorder->Start() ? true : false;
    if (record_result) {
      while (::apollo::cybertron::OK()) {
        std::this_thread::sleep_for(std::chrono::milliseconds(100));
      }
      record_result = record_result && recorder->Stop() ? true : false;
    }
    ::apollo::cybertron::Shutdown();
    return record_result ? 0 : -1;
  } else if (command == "split") {
    if (opt_file_vec.empty()) {
      std::cout << "MUST specify file option (-f)." << std::endl;
      return -1;
    }
    if (opt_file_vec.size() > 1 || opt_output_vec.size() > 1) {
      std::cout << "TOO many input/output file option (-f/-o)." << std::endl;
      return -1;
    }
    if (opt_output_vec.empty()) {
      std::string default_output_file = opt_file_vec[0] + ".split";
      opt_output_vec.push_back(default_output_file);
    }
    ::apollo::cybertron::Init(argv[0]);
    Spliter spliter(opt_file_vec[0], opt_output_vec[0], opt_all,
                    opt_channel_vec, opt_begin, opt_end);
    bool split_result = spliter.Proc();
    ::apollo::cybertron::Shutdown();
    return split_result ? 0 : -1;
  }

  // unknown command
  DisplayUsage(binary, command);
  return -1;
}
