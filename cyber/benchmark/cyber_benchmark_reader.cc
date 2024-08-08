/******************************************************************************
 * Copyright 2024 The Apollo Authors. All Rights Reserved.
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
#include <libgen.h>
#include <string>
#include <vector>

#include "cyber/cyber.h"
#include "cyber/benchmark/benchmark_msg.pb.h"

#if __has_include("gperftools/profiler.h")
#include "gperftools/profiler.h"
#include "gperftools/heap-profiler.h"
#include "gperftools/malloc_extension.h"
#endif

using apollo::cyber::benchmark::BenchmarkMsg;

std::string BINARY_NAME = "cyber_benchmark_reader"; // NOLINT

int nums_of_reader = 1;
bool enable_cpuprofile = false;
bool enable_heapprofile = false;
std::string profile_filename = "cyber_benchmark_reader_cpu.prof"; // NOLINT
std::string heapprofile_filename = "cyber_benchmark_reader_mem.prof"; // NOLINT

void DisplayUsage() {
  AINFO << "Usage: \n    " << BINARY_NAME << " [OPTION]...\n"
        << "Description: \n"
        << "    -h, --help: help information \n"
        << "    -n, --nums_of_reader=nums_of_reader: numbers of reader, "
           "default value is 1\n"
        << "    -c, --cpuprofile: enable gperftools cpu profile\n"
        << "    -o, --profile_filename=filename: the filename to dump the "
            "profile to, default value is cyber_benchmark_writer_cpu.prof.  "
            "Only work with -c option\n"
        << "    -H, --heapprofile: enable gperftools heap profile\n"
        << "    -O, --heapprofile_filename=filename: the filename "
            " to dump the profile to, default value is "
            "cyber_benchmark_writer_mem.prof. Only work with -H option\n"
        << "Example:\n"
        << "    " << BINARY_NAME << " -h\n"
        << "    " << BINARY_NAME << " -n 1\n"
        << "    " << BINARY_NAME << " -n 10 -c -H ";
}

void GetOptions(const int argc, char* const argv[]) {
  opterr = 0;  // extern int opterr
  int long_index = 0;
  const std::string short_opts = "hn:co:HO:";
  static const struct option long_opts[] = {
      {"help", no_argument, nullptr, 'h'},
      {"nums_of_reader", required_argument, nullptr, 'n'},
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

  if (1 == argc) {
    DisplayUsage();
    exit(0);
  }

  do {
    int opt =
        getopt_long(argc, argv, short_opts.c_str(), long_opts, &long_index);
    if (opt == -1) {
      break;
    }
    switch (opt) {
      case 'n':
        nums_of_reader = std::stoi(std::string(optarg));
        if (nums_of_reader < 0) {
          AERROR << "Invalid numbers of reader. It should be grater than 0";
          exit(-1);
        }
        break;
      case 'c':
#ifndef BASE_PROFILER_H_
        AWARN << "gperftools not installed, ignore perf parameters";
#endif
        enable_cpuprofile = true;
        break;
      case 'o':
        profile_filename = std::string(optarg);
        break;
      case 'H':
#ifndef BASE_PROFILER_H_
        AWARN << "gperftools not installed, ignore perf parameters";
#endif
        enable_heapprofile = true;
        break;
      case 'O':
        heapprofile_filename = std::string(optarg);
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

int main(int argc, char** argv) {
  GetOptions(argc, argv);
  apollo::cyber::Init(argv[0], BINARY_NAME);

  apollo::cyber::ReaderConfig reader_config;
  reader_config.channel_name = "/apollo/cyber/benchmark";

  std::vector<std::shared_ptr<apollo::cyber::Reader<BenchmarkMsg>>> vec;

  for (int i = 0; i < nums_of_reader; i++) {
    std::string node_name = BINARY_NAME + "-" + std::to_string(i);
    auto node = apollo::cyber::CreateNode(node_name);
    vec.push_back(std::move(node->CreateReader<BenchmarkMsg>(
      reader_config, [](const std::shared_ptr<BenchmarkMsg> m){})));
  }

#ifndef NO_TCMALLOC
#ifdef BASE_PROFILER_H_
  if (enable_cpuprofile) {
    ProfilerStart(profile_filename.c_str());
  }
  if (enable_heapprofile) {
    HeapProfilerStart(heapprofile_filename.c_str());
  }
#endif
#endif

  apollo::cyber::WaitForShutdown();

#ifndef NO_TCMALLOC
#ifdef BASE_PROFILER_H_
  if (enable_cpuprofile) {
    ProfilerStop();
  }
  if (enable_heapprofile) {
    HeapProfilerDump("Befor shutdown");
    HeapProfilerStop();
  }
#endif
#endif

  return 0;
}
