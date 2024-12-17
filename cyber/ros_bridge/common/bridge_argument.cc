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

#include "cyber/ros_bridge/common/bridge_argument.h"

#include <getopt.h>
#include <libgen.h>

namespace apollo {
namespace cyber {

void BridgeArgument::DisplayUsage() {
  AINFO << "Usage: \n    " << binary_name_ << " [OPTION]...\n"
        << "Description: \n"
        << "    -h, --help: help information \n"
        << "    -c, --cpuprofile: enable gperftools cpu profile\n"
        << "    -o, --profile_filename=filename: the filename to dump the "
            "profile to, default value is ${binary_name}_cpu.prof. Only work "
            "with -c option\n"
        << "    -H, --heapprofile: enable gperftools heap profile\n"
        << "    -O, --heapprofile_filename=filename: the filename to dump the "
            "profile to, default value is ${binary_name}_mem.prof. Only work "
            "with -H option\n"
        << "Example:\n"
        << "    " << binary_name_ << " -h\n"
        << "    " << binary_name_ << " -c -H ";
}

void BridgeArgument::ParseArgument(const int argc, char* const argv[]) {
  binary_name_ = std::string(basename(argv[0]));
  GetOptions(argc, argv);

  if (enable_cpuprofile_ && profile_filename_.empty()) {
    profile_filename_ = binary_name_ + std::string("_cpu.prof");
  }

  if (enable_heapprofile_ && heapprofile_filename_.empty()) {
    heapprofile_filename_ = binary_name_ + std::string("_mem.prof");
  }

  AINFO << "binary_name_ is " << binary_name_;
}

void BridgeArgument::GetOptions(const int argc, char* const argv[]) {
  opterr = 0;  // extern int opterr
  int long_index = 0;
  const std::string short_opts = "hco:HO:";
  static const struct option long_opts[] = {
      {"help", no_argument, nullptr, 'h'},
      {"cpuprofile", no_argument, nullptr, 'c'},
      {"profile_filename", required_argument, nullptr, 'o'},
      {"heapprofile", no_argument, nullptr, 'H'},
      {"heapprofile_filename", required_argument, nullptr, 'O'},
      {NULL, no_argument, nullptr, 0}};

  // log command for info
  std::string cmd("");
  for (int i = 0; i < argc; ++i) {
    cmd += argv[i];
    cmd += " ";
  }
  AINFO << "command: " << cmd;

  do {
    int opt =
        getopt_long(argc, argv, short_opts.c_str(), long_opts, &long_index);
    if (opt == -1) {
      break;
    }
    switch (opt) {
      case 'c':
        enable_cpuprofile_ = true;
        break;
      case 'o':
        profile_filename_ = std::string(optarg);
        break;
      case 'H':
        enable_heapprofile_ = true;
        break;
      case 'O':
        heapprofile_filename_ = std::string(optarg);
        break;
      case 'h':
        DisplayUsage();
        exit(0);
      default:
        break;
    }
  } while (true);

  if (optind < argc) {
    AINFO << "Found non-option ARGV-element \"" << argv[optind++] << "\"";
    DisplayUsage();
    exit(1);
  }
}

}  // namespace cyber
}  // namespace apollo
