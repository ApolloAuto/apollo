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
#include <cstddef>
#include <memory>
#include <stdexcept>
#include <string>
#include <vector>

#include "cyber/common/file.h"
#include "cyber/common/time_conversion.h"
#include "cyber/init.h"
#include "cyber/tools/cyber_recorder/info.h"
#include "cyber/tools/cyber_recorder/player/player.h"
#include "cyber/tools/cyber_recorder/recorder.h"
#include "cyber/tools/cyber_recorder/recoverer.h"
#include "cyber/tools/cyber_recorder/spliter.h"

using apollo::cyber::common::GetFileName;
using apollo::cyber::common::StringToUnixSeconds;
using apollo::cyber::common::UnixSecondsToString;
using apollo::cyber::record::HeaderBuilder;
using apollo::cyber::record::Info;
using apollo::cyber::record::Player;
using apollo::cyber::record::PlayParam;
using apollo::cyber::record::Recorder;
using apollo::cyber::record::Recoverer;
using apollo::cyber::record::Spliter;

const char INFO_OPTIONS[] = "h";
const char RECORD_OPTIONS[] = "o:ac:i:m:h";
const char PLAY_OPTIONS[] = "f:ac:k:lr:b:e:s:d:p:h";
const char SPLIT_OPTIONS[] = "f:o:c:k:b:e:h";
const char RECOVER_OPTIONS[] = "f:o:h";

void DisplayUsage(const std::string& binary);
void DisplayUsage(const std::string& binary, const std::string& command);
void DisplayUsage(const std::string& binary, const std::string& command,
                  const std::string& options);

void DisplayUsage(const std::string& binary) {
  std::cout << "usage: " << binary << " <command> [<args>]\n"
            << "The " << binary << " commands are:\n"
            << "\tinfo\tShow information of an exist record.\n"
            << "\tplay\tPlay an exist record.\n"
            << "\trecord\tRecord same topic.\n"
            << "\tsplit\tSplit an exist record.\n"
            << "\trecover\tRecover an exist record.\n"
            << std::endl;
}

void DisplayUsage(const std::string& binary, const std::string& command) {
  if (command == "info") {
    std::cout << "usage: cyber_recorder info file" << std::endl;
    std::cout << "usage: " << binary << " " << command << " [options]"
              << std::endl;
    DisplayUsage(binary, command, INFO_OPTIONS);
    return;
  }

  std::cout << "usage: " << binary << " " << command << " [options]"
            << std::endl;
  if (command == "record") {
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
        std::cout << "\t-c, --white-channel <name>\t\tonly " << command
                  << " the specified channel" << std::endl;
        break;
      case 'k':
        std::cout << "\t-k, --black-channel <name>\t\tnot " << command
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
      case 'p':
        std::cout << "\t-p, --preload <seconds>\t\t\t" << command
                  << " after trying to preload n second(s)" << std::endl;
        break;
      case 'i':
        std::cout << "\t-i, --segment-interval <seconds>\t" << command
                  << " segmented every n second(s)" << std::endl;
        break;
      case 'm':
        std::cout << "\t-m, --segment-size <MB>\t\t\t" << command
                  << " segmented every n megabyte(s)" << std::endl;
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
  std::string file_path;
  if (argc >= 3) {
    file_path = std::string(argv[2]);
  }

  int long_index = 0;
  const std::string short_opts = "f:c:k:o:alr:b:e:s:d:p:i:m:h";
  static const struct option long_opts[] = {
      {"files", required_argument, nullptr, 'f'},
      {"white-channel", required_argument, nullptr, 'c'},
      {"black-channel", required_argument, nullptr, 'k'},
      {"output", required_argument, nullptr, 'o'},
      {"all", no_argument, nullptr, 'a'},
      {"loop", no_argument, nullptr, 'l'},
      {"rate", required_argument, nullptr, 'r'},
      {"begin", required_argument, nullptr, 'b'},
      {"end", required_argument, nullptr, 'e'},
      {"start", required_argument, nullptr, 's'},
      {"delay", required_argument, nullptr, 'd'},
      {"preload", required_argument, nullptr, 'p'},
      {"segment-interval", required_argument, nullptr, 'i'},
      {"segment-size", required_argument, nullptr, 'm'},
      {"help", no_argument, nullptr, 'h'}};

  std::vector<std::string> opt_file_vec;
  std::vector<std::string> opt_output_vec;
  std::vector<std::string> opt_white_channels;
  std::vector<std::string> opt_black_channels;
  bool opt_all = false;
  bool opt_loop = false;
  float opt_rate = 1.0f;
  uint64_t opt_begin = 0;
  uint64_t opt_end = UINT64_MAX;
  uint64_t opt_start = 0;
  uint64_t opt_delay = 0;
  uint32_t opt_preload = 3;
  auto opt_header = HeaderBuilder::GetHeader();

  do {
    int opt =
        getopt_long(argc, argv, short_opts.c_str(), long_opts, &long_index);
    if (opt == -1) {
      break;
    }
    switch (opt) {
      case 'f':
        opt_file_vec.emplace_back(std::string(optarg));
        for (int i = optind; i < argc; i++) {
          if (*argv[i] != '-') {
            opt_file_vec.emplace_back(std::string(argv[i]));
          } else {
            break;
          }
        }
        break;
      case 'c':
        opt_white_channels.emplace_back(std::string(optarg));
        for (int i = optind; i < argc; i++) {
          if (*argv[i] != '-') {
            opt_white_channels.emplace_back(std::string(argv[i]));
          } else {
            break;
          }
        }
        break;
      case 'k':
        opt_black_channels.emplace_back(std::string(optarg));
        for (int i = optind; i < argc; i++) {
          if (*argv[i] != '-') {
            opt_black_channels.emplace_back(std::string(argv[i]));
          } else {
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
        } catch (const std::invalid_argument& ia) {
          std::cout << "Invalid argument: -r/--rate " << std::string(optarg)
                    << std::endl;
          return -1;
        } catch (const std::out_of_range& e) {
          std::cout << "Argument is out of range: -r/--rate "
                    << std::string(optarg) << std::endl;
          return -1;
        }
        break;
      case 'b':
        opt_begin =
            StringToUnixSeconds(std::string(optarg)) * 1000 * 1000 * 1000ULL;
        break;
      case 'e':
        opt_end =
            StringToUnixSeconds(std::string(optarg)) * 1000 * 1000 * 1000ULL;
        break;
      case 's':
        try {
          opt_start = std::stoi(optarg);
        } catch (const std::invalid_argument& ia) {
          std::cout << "Invalid argument: -s/--start " << std::string(optarg)
                    << std::endl;
          return -1;
        } catch (const std::out_of_range& e) {
          std::cout << "Argument is out of range: -s/--start "
                    << std::string(optarg) << std::endl;
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
        } catch (const std::out_of_range& e) {
          std::cout << "Argument is out of range: -d/--delay "
                    << std::string(optarg) << std::endl;
          return -1;
        }
        break;
      case 'p':
        try {
          opt_preload = std::stoi(optarg);
        } catch (std::invalid_argument& ia) {
          std::cout << "Invalid argument: -p/--preload " << std::string(optarg)
                    << std::endl;
          return -1;
        } catch (const std::out_of_range& e) {
          std::cout << "Argument is out of range: -p/--preload "
                    << std::string(optarg) << std::endl;
          return -1;
        }
        break;
      case 'i':
        try {
          int interval_s = std::stoi(optarg);
          if (interval_s < 0) {
            std::cout << "Argument is less than zero: -i/--segment-interval "
                      << std::string(optarg) << std::endl;
            return -1;
          }
          opt_header.set_segment_interval(interval_s * 1000000000ULL);
        } catch (std::invalid_argument& ia) {
          std::cout << "Invalid argument: -i/--segment-interval "
                    << std::string(optarg) << std::endl;
          return -1;
        } catch (const std::out_of_range& e) {
          std::cout << "Argument is out of range: -i/--segment-interval "
                    << std::string(optarg) << std::endl;
          return -1;
        }
        break;
      case 'm':
        try {
          int size_mb = std::stoi(optarg);
          if (size_mb < 0) {
            std::cout << "Argument is less than zero: -m/--segment-size "
                      << std::string(optarg) << std::endl;
            return -1;
          }
          opt_header.set_segment_raw_size(size_mb * 1024 * 1024ULL);
        } catch (std::invalid_argument& ia) {
          std::cout << "Invalid argument: -m/--segment-size "
                    << std::string(optarg) << std::endl;
          return -1;
        } catch (const std::out_of_range& e) {
          std::cout << "Argument is out of range: -m/--segment-size "
                    << std::string(optarg) << std::endl;
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
    if (file_path.empty()) {
      std::cout << "usage: cyber_recorder info file" << std::endl;
      return -1;
    }
    ::apollo::cyber::Init(argv[0]);
    Info info;
    bool info_result = info.Display(file_path);
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
    ::apollo::cyber::Init(argv[0]);
    Recoverer recoverer(opt_file_vec[0], opt_output_vec[0]);
    bool recover_result = recoverer.Proc();
    return recover_result ? 0 : -1;
  }

  if (command == "play") {
    if (opt_file_vec.empty()) {
      std::cout << "MUST specify file option (-f)." << std::endl;
      return -1;
    }
    ::apollo::cyber::Init(argv[0]);
    PlayParam play_param;
    play_param.is_play_all_channels = opt_all || opt_white_channels.empty();
    play_param.is_loop_playback = opt_loop;
    play_param.play_rate = opt_rate;
    play_param.begin_time_ns = opt_begin;
    play_param.end_time_ns = opt_end;
    play_param.start_time_s = opt_start;
    play_param.delay_time_s = opt_delay;
    play_param.preload_time_s = opt_preload;
    play_param.files_to_play.insert(opt_file_vec.begin(), opt_file_vec.end());
    play_param.black_channels.insert(opt_black_channels.begin(),
                                     opt_black_channels.end());
    play_param.channels_to_play.insert(opt_white_channels.begin(),
                                       opt_white_channels.end());
    Player player(play_param);
    const bool play_result = player.Init() && player.Start();
    return play_result ? 0 : -1;
  } else if (command == "record") {
    if (opt_white_channels.empty() && !opt_all) {
      std::cout
          << "MUST specify channels option (-c) or all channels option (-a)."
          << std::endl;
      return -1;
    }
    if (opt_output_vec.size() > 1) {
      std::cout << "TOO many output file option (-o)." << std::endl;
      return -1;
    }
    if (opt_output_vec.empty()) {
      std::string default_output_file =
          UnixSecondsToString(time(nullptr), "%Y%m%d%H%M%S") + ".record";
      opt_output_vec.push_back(default_output_file);
    }
    ::apollo::cyber::Init(argv[0]);
    auto recorder = std::make_shared<Recorder>(opt_output_vec[0], opt_all,
                                               opt_white_channels, opt_header);
    bool record_result = recorder->Start();
    if (record_result) {
      while (!::apollo::cyber::IsShutdown()) {
        std::this_thread::sleep_for(std::chrono::milliseconds(100));
      }
      record_result = recorder->Stop();
    }
    return record_result ? 0 : -1;
  } else if (command == "split") {
    if (opt_file_vec.empty()) {
      std::cout << "Must specify file option (-f)." << std::endl;
      return -1;
    }
    if (opt_file_vec.size() > 1 || opt_output_vec.size() > 1) {
      std::cout << "Too many input/output file option (-f/-o)." << std::endl;
      return -1;
    }
    if (opt_output_vec.empty()) {
      std::string default_output_file = opt_file_vec[0] + ".split";
      opt_output_vec.push_back(default_output_file);
    }
    ::apollo::cyber::Init(argv[0]);
    Spliter spliter(opt_file_vec[0], opt_output_vec[0], opt_white_channels,
                    opt_black_channels, opt_begin, opt_end);
    bool split_result = spliter.Proc();
    return split_result ? 0 : -1;
  }

  // unknown command
  DisplayUsage(binary, command);
  return -1;
}
