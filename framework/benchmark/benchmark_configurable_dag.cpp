/******************************************************************************
 * Copyright 2018 The Apollo Authors. All Rights Reserved.
 *
0 * Licensed under the Apache License, Version 2.0 (the "License");
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

#include "cybertron/common/environment.h"
#include "cybertron/common/file.h"
#include "cybertron/common/log.h"
#include "cybertron/croutine/routine_factory.h"
#include "cybertron/cybertron.h"
#include "cybertron/data/data_cache.h"
#include "cybertron/data/data_visitor.h"
#include "cybertron/node/node.h"
#include "cybertron/node/reader.h"
#include "cybertron/node/writer.h"
#include "cybertron/scheduler/scheduler.h"

#include "cybertron/component/component.h"
#include "cybertron/component/component_base.h"

#include "cybertron/proto/benchmark_dag_config.pb.h"
#include "cybertron/proto/chatter.pb.h"
#include "cybertron/proto/component_config.pb.h"

using apollo::cybertron::Component;
using apollo::cybertron::ComponentBase;
using apollo::cybertron::Node;
using apollo::cybertron::Reader;
using apollo::cybertron::Time;
using apollo::cybertron::Writer;
using apollo::cybertron::common::GetAbsolutePath;
using apollo::cybertron::common::GetProtoFromFile;
using apollo::cybertron::common::WorkRoot;
using apollo::cybertron::proto::BenchmarkDagConfig;
using apollo::cybertron::proto::BProcType;
using apollo::cybertron::proto::Chatter;
using apollo::cybertron::scheduler::Scheduler;

struct LocalWriteInfo {
  int msg_num = 0;
  uint64_t write_gap_us = 0;
  std::string channel_name = "";
  std::shared_ptr<Writer<Chatter>> writer = nullptr;
};

struct LocalReadInfo {
  uint64_t proc_gap_us = 0;
  std::string channel_name = "";
  BProcType proc_type;
  std::shared_ptr<Reader<Chatter>> reader = nullptr;
};

std::vector<std::shared_ptr<Reader<Chatter>>> reader_list;
std::vector<std::shared_ptr<ComponentBase>> component_base_list;
std::vector<uint64_t> latency_list;
std::map<std::string, std::vector<LocalWriteInfo>> writer_info_map;
std::map<std::string, std::vector<LocalWriteInfo>> start_writer_info_map;
std::map<std::string, std::vector<LocalReadInfo>> reader_info_map;

void nsec_spin(uint64_t spin_time) {
  if (spin_time == 0) {
    return;
  }

  timespec now;
  timespec start;
  uint64_t start_ns;
  clock_gettime(CLOCK_MONOTONIC, &start);
  start_ns = static_cast<uint64_t>(start.tv_sec) * 1000000000L +
             static_cast<uint64_t>(start.tv_nsec);
  while (1) {
    clock_gettime(CLOCK_MONOTONIC, &now);
    uint64_t now_ns = static_cast<uint64_t>(now.tv_sec) * 1000000000L +
                      static_cast<uint64_t>(now.tv_nsec);
    if ((now_ns - start_ns) >= spin_time) {
      break;
    }
  }
  return;
}

void LocalProc(const std::string& node_name,
               const std::shared_ptr<Chatter>& msg) {
  std::string channel_name_out;
  std::shared_ptr<Writer<Chatter>> writer;
  if (writer_info_map.find(node_name) != writer_info_map.end()) {
    auto itr = writer_info_map.find(node_name);
    auto write_info = itr->second.at(0);
    channel_name_out = write_info.channel_name;
    writer = write_info.writer;
  } else {
    channel_name_out = "";
  }

  if (reader_info_map.find(node_name) != reader_info_map.end()) {
    auto reader_info = reader_info_map[node_name].at(0);
    switch (reader_info.proc_type) {
      case apollo::cybertron::proto::BCPU:
        nsec_spin(reader_info.proc_gap_us * 1000);
        break;
      case apollo::cybertron::proto::BSLEEP:
        std::this_thread::sleep_for(
            std::chrono::milliseconds(reader_info.proc_gap_us / 1000));
        break;
      case apollo::cybertron::proto::BGPU:
      case apollo::cybertron::proto::BUSLEEP:
        usleep(reader_info.proc_gap_us);
        break;
      default:
        std::this_thread::sleep_for(
            std::chrono::milliseconds(reader_info.proc_gap_us / 1000));
    }
  } else {
    AERROR << "node[" << node_name << "] has node reader in proc func";
    return;
  }

  if (channel_name_out == "") {
    uint64_t lidar_stamp = msg->lidar_timestamp();
    uint64_t end_stamp = Time::MonoTime().ToNanosecond();
    uint64_t latency = end_stamp - lidar_stamp;
    ADEBUG << "proc node[" << node_name << "] seq[" << msg->seq()
           << "] start_stamp[" << lidar_stamp << "] end_stamp[" << end_stamp
           << "] latency[" << end_stamp - lidar_stamp << "]";
    latency_list.emplace_back(latency);
  } else {
    uint64_t lidar_stamp = msg->lidar_timestamp();
    uint64_t end_stamp = Time::MonoTime().ToNanosecond();
    writer->Write(msg);
    ADEBUG << "proc node[" << node_name << "] seq[" << msg->seq()
           << "] latency[" << end_stamp - lidar_stamp << "]";
  }
}

class CommonComponent2 : public apollo::cybertron::Component<Chatter, Chatter> {
 public:
  CommonComponent2(){};
  ~CommonComponent2() override{};
  bool Init() { return true; }
  bool Proc(const std::shared_ptr<Chatter>& msg1,
            const std::shared_ptr<Chatter>& msg2) {
    LocalProc(node_->Name(), msg1);
    return true;
  }
};

class CommonComponent1 : public apollo::cybertron::Component<Chatter> {
 public:
  CommonComponent1(){};
  ~CommonComponent1() override{};
  bool Init() { return true; }
  bool Proc(const std::shared_ptr<Chatter>& msg) {
    LocalProc(node_->Name(), msg);
    return true;
  }
};

class CommonComponent3
    : public apollo::cybertron::Component<Chatter, Chatter, Chatter> {
 public:
  CommonComponent3(){};
  ~CommonComponent3() override{};
  bool Init() { return true; }
  bool Proc(const std::shared_ptr<Chatter>& msg1,
            const std::shared_ptr<Chatter>& msg2,
            const std::shared_ptr<Chatter>& msg3) {
    LocalProc(node_->Name(), msg1);
    return true;
  }
};

void GenDag() {
  auto config_path =
      GetAbsolutePath(WorkRoot(), "conf/benchmark_dag_config.pb.conf");
  BenchmarkDagConfig config;
  assert(GetProtoFromFile(config_path, &config));
  for (int i = 0; i < config.node_conf_size(); ++i) {
    auto node_conf = config.node_conf(i);
    std::string node_name = node_conf.node_name();
    switch (node_conf.type()) {
      case apollo::cybertron::proto::START_NODE: {
        std::shared_ptr<Node> start_node =
            apollo::cybertron::CreateNode(node_name);
        if (node_conf.writer_conf_size() < 1) {
          AERROR << "node[" << node_name << "] conf error, has no writer";
          return;
        }
        for (auto write_conf : node_conf.writer_conf()) {
          std::shared_ptr<Writer<Chatter>> writer;
          apollo::cybertron::proto::RoleAttributes attr;
          attr.set_channel_name(write_conf.channel_name());
          auto qos = attr.mutable_qos_profile();
          qos->CopyFrom(apollo::cybertron::transport::QosProfileConf::
                            QOS_PROFILE_DEFAULT);
          writer =
              start_node->CreateWriter<apollo::cybertron::proto::Chatter>(attr);
          LocalWriteInfo writer_info;
          writer_info.channel_name = write_conf.channel_name();
          writer_info.writer = writer;
          writer_info.msg_num = write_conf.msg_num();
          writer_info.write_gap_us = write_conf.write_gap_us();
          start_writer_info_map[node_name].emplace_back(writer_info);
          AINFO << "writer channel[" << write_conf.channel_name()
                << "] created!";
        }
        break;
      }
      case apollo::cybertron::proto::END_NODE:
      case apollo::cybertron::proto::COMMON_NODE: {
        std::shared_ptr<Node> node = apollo::cybertron::CreateNode(node_name);
        if (node_conf.reader_conf_size() < 1) {
          AERROR << "node[" << node_name << "] conf error, has no reader";
          return;
        }
        switch (node_conf.trige_type()) {
          case apollo::cybertron::proto::MULTI_READER: {
            for (int i = 0; i < node_conf.reader_conf_size(); ++i) {
              auto read_conf = node_conf.reader_conf(i);
              apollo::cybertron::proto::RoleAttributes attr;
              attr.set_channel_name(read_conf.channel_name());
              attr.set_node_name(node_name);
              auto reader =
                  node->CreateReader<apollo::cybertron::proto::Chatter>(
                      attr, [=](const std::shared_ptr<
                                apollo::cybertron::proto::Chatter>& msg1) {
                        LocalProc(node_name, msg1);
                      });
              reader_list.emplace_back(std::move(reader));
              LocalReadInfo reader_info;
              reader_info.channel_name = read_conf.channel_name();
              reader_info.proc_gap_us = read_conf.proc_gap_us();
              reader_info_map[node_name].emplace_back(reader_info);
              AINFO << "reader channel[" << read_conf.channel_name()
                    << "] created!";
            }
            break;
          }
          case apollo::cybertron::proto::FUSION: {
            node_name = node_name + "_component";
            std::shared_ptr<apollo::cybertron::ComponentBase> base;
            int size = node_conf.reader_conf_size();
            if (size == 1) {
              base = std::make_shared<CommonComponent1>();
            } else if (size == 2) {
              base = std::make_shared<CommonComponent2>();
            } else if (size == 3) {
              base = std::make_shared<CommonComponent3>();
            } else {
              AERROR << "node[" << node_name
                     << "] config error more than 3 channel not supported";
            }
            apollo::cybertron::proto::ComponentConfig config;
            config.set_name(node_name);
            for (auto read_conf : node_conf.reader_conf()) {
              auto r_conf = config.add_readers();
              r_conf->set_channel(read_conf.channel_name());
            }
            AINFO << "component main_channel["
                  << node_conf.reader_conf(0).channel_name() << "] created!";
            LocalReadInfo reader_info;
            reader_info.channel_name = node_conf.reader_conf(0).channel_name();
            reader_info.proc_gap_us = node_conf.reader_conf(0).proc_gap_us();
            reader_info.proc_type = node_conf.reader_conf(0).proc_type();
            reader_info_map[node_name].emplace_back(reader_info);
            base->Initialize(config);
            component_base_list.emplace_back(std::move(base));
            break;
          }
          default:
            break;
        }
        for (auto write_conf : node_conf.writer_conf()) {
          std::shared_ptr<Writer<Chatter>> writer;
          apollo::cybertron::proto::RoleAttributes attr;
          attr.set_channel_name(write_conf.channel_name());
          AINFO << write_conf.channel_name();
          auto qos = attr.mutable_qos_profile();
          qos->CopyFrom(apollo::cybertron::transport::QosProfileConf::
                            QOS_PROFILE_DEFAULT);
          writer = node->CreateWriter<apollo::cybertron::proto::Chatter>(attr);
          LocalWriteInfo writer_info;
          writer_info.channel_name = write_conf.channel_name();
          writer_info.writer = writer;
          writer_info.write_gap_us = write_conf.write_gap_us();
          writer_info_map[node_name].emplace_back(writer_info);
        }
      } break;
      default:
        break;
    }
  }

  std::this_thread::sleep_for(std::chrono::milliseconds(200));

  for (auto itr = start_writer_info_map.begin();
       itr != start_writer_info_map.end(); ++itr) {
    for (auto writer_info : itr->second) {
      std::thread th([=]() {
        int msg_num = writer_info.msg_num;
        auto writer = writer_info.writer;
        if (writer == nullptr) {
          return;
        }
        for (int i = 0; i < msg_num; ++i) {
          usleep(writer_info.write_gap_us);
          auto msg = std::make_shared<apollo::cybertron::proto::Chatter>();
          auto lidar_stamp = apollo::cybertron::Time::MonoTime().ToNanosecond();
          msg->set_timestamp(lidar_stamp);
          msg->set_seq(i);
          msg->set_content("Hello, apollo!");
          msg->set_lidar_timestamp(lidar_stamp);
          writer->Write(msg);
          ADEBUG << "send msg to channel[" << writer_info.channel_name
                 << "] write_gap_us[" << writer_info.write_gap_us << "] msg_id["
                 << i << "] stamp[" << lidar_stamp << "]";
        }
      });
      th.detach();
    }
  }
}

int main(int argc, char* argv[]) {
  apollo::cybertron::Init(argv[0]);
  GenDag();
  while (!apollo::cybertron::IsShutdown()) {
    std::this_thread::sleep_for(std::chrono::milliseconds(200));
  }
  apollo::cybertron::PrintSchedulerStatistics();
  int size = latency_list.size();
  if (size <= 0) {
    AINFO << "benchmark down without e2e finished";
    return 0;
  }
  apollo::cybertron::PrintSchedulerStatistics();

  std::sort(latency_list.begin(), latency_list.end());
  int nn_index = size * 0.99;
  int fifth_index = size * 0.5;
  AINFO << "e2e_num: " << latency_list.size()
        << ", min_lantecy(ns): " << latency_list[0]
        << ", max_latency(ns): " << latency_list[size - 1]
        << ", 50_latency(ns): " << latency_list[fifth_index]
        << ", 99_latency(ns): " << latency_list[nn_index];
}
