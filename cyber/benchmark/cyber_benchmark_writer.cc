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
#include <memory>

#include "cyber/cyber.h"
#include "cyber/benchmark/benchmark_msg.pb.h"

#if __has_include("gperftools/profiler.h")
#include "gperftools/profiler.h"
#include "gperftools/heap-profiler.h"
#include "gperftools/malloc_extension.h"
#endif

std::string BINARY_NAME = "cyber_benchmark_writer"; // NOLINT

int message_size = -1;
int transport_freq = -1;
int qos_policy = 0;
int data_type = 0;
int running_time = 10;
bool enable_cpuprofile = false;
bool enable_heapprofile = false;
std::string profile_filename = "cyber_benchmark_writer_cpu.prof"; // NOLINT
std::string heapprofile_filename = "cyber_benchmark_writer_mem.prof"; // NOLINT

void DisplayUsage() {
  AINFO << "Usage: \n    " << BINARY_NAME << " [OPTION]...\n"
        << "Description: \n"
        << "    -h, --help: help information \n"
        << "    -s, --message_size=message_size: transport message size\n"
        << "    -t, --transport_freq=transmission_frequency: transmission frequency\n" // NOLINT
        << "    -q, --qos_policy=qos_reliable_policy: set qos reliable policy, "
            "0 is Reliable, 1 is Best effort, default value is 0\n"
        << "    -d, --data_type=data_type: transport data type, "
            "0 is bytes, 1 is repeated field, default value is 0\n"
        << "    -T, --time=time: running time, default value is 10 seconds\n"
        << "    -c, --cpuprofile: enable gperftools cpu profile\n"
        << "    -o, --profile_filename=filename: the filename to dump the "
            "profile to, default value is cyber_benchmark_writer_cpu.prof. Only work " // NOLINT
            "with -c option\n"
        << "    -H, --heapprofile: enable gperftools heap profile\n"
        << "    -O, --heapprofile_filename=filename: the filename to dump the "
            "profile to, default value is cyber_benchmark_writer_mem.prof. Only work " // NOLINT
            "with -H option\n"
        << "Example:\n"
        << "    " << BINARY_NAME << " -h\n"
        << "    " << BINARY_NAME << " -s 64K -t 10\n"
        << "    " << BINARY_NAME << " -s 64K -t 10 -c -H ";
}

void GetOptions(const int argc, char* const argv[]) {
  opterr = 0;  // extern int opterr
  int long_index = 0;
  const std::string short_opts = "hs:t:q:d:T:co:HO:";
  static const struct option long_opts[] = {
      {"help", no_argument, nullptr, 'h'},
      {"message_size", required_argument, nullptr, 's'},
      {"transport_freq", required_argument, nullptr, 't'},
      {"qos_policy", required_argument, nullptr, 'q'},
      {"data_type", required_argument, nullptr, 'd'},
      {"time", required_argument, nullptr, 'T'},
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
    int base_size = 1;
    std::string arg;
    switch (opt) {
      case 's':
        arg = std::string(optarg);
        switch (arg[arg.length()-1]) {
          case 'K':
            base_size = 1024;
            break;
          case 'M':
            base_size = 1024 * 1024;
            break;
          default:
            AERROR << "Invalid identifier. It should be 'K' or 'M' or 'B'";
            exit(-1);
        }
        message_size = std::stoi(arg.substr(0, arg.length()-1)) * base_size;
        if (message_size < 0 || message_size % 4 != 0) {
          AERROR << "Invalid message size.";
          exit(-1);
        }
        break;
      case 't':
        transport_freq = std::stoi(std::string(optarg));
        if (transport_freq < 0) {
          AERROR << "Invalid transport frequency. It should greater than 0";
          exit(-1);
        }
        break;
      case 'T':
        running_time = std::stoi(std::string(optarg));
        if (running_time < 0) {
          AERROR << "Invalid running time. It should greater than 0";
          exit(-1);
        }
        break;
      case 'q':
        qos_policy = std::stoi(std::string(optarg));
        if (qos_policy != 0 && qos_policy != 1) {
          AERROR << "Invalid qos_policy. It should be 0 or 1";
          exit(-1);
        }
        break;
      case 'd':
        data_type = std::stoi(std::string(optarg));
        if (data_type != 0 && data_type != 1) {
          AERROR << "Invalid data_type. It should be 0 or 1";
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

  if (message_size == -1 || transport_freq == -1) {
    AINFO << "-s and -t parameters must be specified";
    DisplayUsage();
    exit(1);
  }
}

int main(int argc, char** argv) {
  GetOptions(argc, argv);

  apollo::cyber::Init(argv[0], BINARY_NAME);

  auto node = apollo::cyber::CreateNode(BINARY_NAME);

  apollo::cyber::proto::RoleAttributes attrs;
  attrs.set_channel_name("/apollo/cyber/benchmark");
  auto qos = attrs.mutable_qos_profile();

  if (qos_policy == 1) {
    qos->set_reliability(
      apollo::cyber::proto::QosReliabilityPolicy::RELIABILITY_BEST_EFFORT);
  } else {
    qos->set_reliability(
      apollo::cyber::proto::QosReliabilityPolicy::RELIABILITY_RELIABLE);
  }
  auto writer = node->CreateWriter<
    apollo::cyber::benchmark::BenchmarkMsg>(attrs);

  // sleep a while for initialization, aboout 2 seconds
  apollo::cyber::Rate rate_init(0.5);
  apollo::cyber::Rate rate_ctl(static_cast<float>(transport_freq));
  rate_init.Sleep();

  uint64_t send_msg_total = transport_freq * running_time;

  // std::vector<uint32_t> trans_vec;
  // int num_of_instance = message_size / 4;
  // for (int i = 0; i < num_of_instance; i++) {
  //   trans_vec.push_back(rand());
  // }

  // char* data = (char*)malloc(message_size);
  // for (int i = 0; i < num_of_instance; i++) {
  //   *(uint32_t*)(data + i * 4) = rand();
  // }

  // if (data_type == 0) {
  //   trans_unit->set_data_bytes(data, message_size);
  // } else {
  //   for (int i = 0; i < num_of_instance; i++) {
  //     trans_unit->add_data(trans_vec[i]);
  //   }
  // }
  // free(data);

  int send_msg = 0;

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

std::vector<uint32_t> trans_vec;
int num_of_instance = message_size / 4;
for (int i = 0; i < num_of_instance; i++) {
  trans_vec.push_back(rand());  // NOLINT
}

char* data = (char*)malloc(message_size); // NOLINT

  while (send_msg < send_msg_total) {
    auto trans_unit = std::make_shared<
      apollo::cyber::benchmark::BenchmarkMsg>();
    int base = rand();          // NOLINT

    for (int i = 0; i < num_of_instance; i++) {
      trans_vec[i] = base * i;
    }

    for (int i = 0; i < num_of_instance; i++) {
      *(uint32_t*)(data + i * 4) = base * i;  // NOLINT
    }

    if (data_type == 0) {
      trans_unit->set_data_bytes(data, message_size);
    } else {
      for (int i = 0; i < num_of_instance; i++) {
        trans_unit->add_data(trans_vec[i]);
      }
    }

    writer->Write(trans_unit);
    ++send_msg;
    rate_ctl.Sleep();
  }

free(data);

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

  apollo::cyber::Clear();

  return 0;
}
