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
#include <math.h>

#include "cybertron/common/log.h"
#include "cybertron/component/component.h"
#include "cybertron/component/component_base.h"
#include "cybertron/cybertron.h"
#include "cybertron/proto/chatter.pb.h"
#include "cybertron/proto/component_config.pb.h"
#include "cybertron/scheduler/scheduler.h"

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

struct LocalWriteInfo {
  std::string out_channel_name = "";
  std::shared_ptr<apollo::cybertron::Writer<apollo::cybertron::proto::Chatter>>
      writer = nullptr;
  int processor_id = 0;
};

std::vector<std::shared_ptr<apollo::cybertron::ComponentBase>>
    component_base_list;
std::vector<std::shared_ptr<
    apollo::cybertron::Reader<apollo::cybertron::proto::Chatter>>>
    reader_list;
std::vector<uint64_t> latency_list;
std::vector<uint64_t> chain_latency_list;
std::map<std::string, LocalWriteInfo> writer_info_map;
std::vector<uint64_t> sum_list;
int LAYER_NUM = 0;
int MSG_SEND_GAP_US = 0;
int PROC_TIME_US = 0;
// 0: usleep; 1: cpu computer; 2: gpu computer
int PROC_TYPE = 0;
int MSG_NUM = 0;

void LocalProc(const std::string& node_name,
               const std::shared_ptr<apollo::cybertron::proto::Chatter>& msg) {
  auto proc_stamp = apollo::cybertron::Time::Now().ToNanosecond();
  std::string channel_name_out;
  std::shared_ptr<apollo::cybertron::Writer<apollo::cybertron::proto::Chatter>>
      writer;
  int proc_id;
  if (writer_info_map.find(node_name) != writer_info_map.end()) {
    auto itr = writer_info_map.find(node_name);
    channel_name_out = itr->second.out_channel_name;
    writer = itr->second.writer;
    proc_id = itr->second.processor_id;
  } else {
    return;
  }
  if (PROC_TYPE == 0) {
    usleep(PROC_TIME_US);
  } else if (PROC_TYPE == 1) {
    nsec_spin(PROC_TIME_US * 1000);
  } else if (PROC_TYPE == 2) {
    std::this_thread::sleep_for(std::chrono::milliseconds(PROC_TIME_US / 1000));
  } else if (PROC_TYPE == 3) {
    uint64_t sum = 0;
    for (uint64_t i = 0; i < 20000000; i++) {
        sum += i;
    }
    sum_list.emplace_back(sum);
  }

  if (channel_name_out == "") {
    uint64_t lidar_stamp = msg->lidar_timestamp();
    auto end_stamp = apollo::cybertron::Time::Now().ToNanosecond();
    uint64_t latency = end_stamp - lidar_stamp;
    AINFO << "e2e ended seq[" << msg->seq() << "] latency[" << latency
           << "] chain_latency[" << proc_stamp - lidar_stamp
           << "] proc_latency[" << end_stamp - proc_stamp << "] lidar_stamp["
           << lidar_stamp << "] end_stamp[" << end_stamp << "] proc_stamp["
           << proc_stamp << "] proc_id[" << proc_id << "]";
    chain_latency_list.emplace_back(proc_stamp - lidar_stamp);
    if (msg->seq() == 0 && channel_name_out == "") {
      return;
    }
    latency_list.emplace_back(latency);
    if (msg->seq() == MSG_NUM - 1 && channel_name_out == "") {
      AINFO << "benchmark done";
    }
  } else {
    uint64_t lidar_stamp = msg->lidar_timestamp();
    auto e_stamp = apollo::cybertron::Time::Now().ToNanosecond();
    writer->Write(msg);
    auto end_stamp = apollo::cybertron::Time::Now().ToNanosecond();
    AINFO << "end_proc node[" << node_name << "] seq[" << msg->seq()
           << "] latency[" << end_stamp - lidar_stamp << "] chain_latency["
           << proc_stamp - lidar_stamp << "] proc_latency["
           << (end_stamp - proc_stamp) << "] write_msg_latency[" << (end_stamp - e_stamp)
           << "] lidar_stamp[" << lidar_stamp
           << "] end_stamp[" << end_stamp << "] proc_stamp[" << proc_stamp
           << "] proc_id[" << proc_id << "]";
  }
}

class GaussianComponent
    : public apollo::cybertron::Component<apollo::cybertron::proto::Chatter,
                                          apollo::cybertron::proto::Chatter> {
 public:
  GaussianComponent(){};
  ~GaussianComponent() override{};
  bool Init() { return true; }
  bool Proc(const std::shared_ptr<apollo::cybertron::proto::Chatter>& msg1,
            const std::shared_ptr<apollo::cybertron::proto::Chatter>& msg2) {
    LocalProc(node_->Name(), msg1);
    return true;
  }
};

int GenGaussianDag() {
  std::shared_ptr<apollo::cybertron::Node> start_node;
  std::shared_ptr<apollo::cybertron::Writer<apollo::cybertron::proto::Chatter>>
      start_node_writer;
  int idx = 0;
  for (int i = 1; i <= LAYER_NUM; ++i) {
    for (int j = 0; j < i + 1; ++j) {
      // start node
      if (i == LAYER_NUM && j == 0) {
        start_node = apollo::cybertron::CreateNode("start_node");
        apollo::cybertron::proto::RoleAttributes attr;
        attr.set_channel_name("channel/chatter_" + std::to_string(LAYER_NUM) +
                              "_" + std::to_string(0));

        auto qos = attr.mutable_qos_profile();
        qos->CopyFrom(
            apollo::cybertron::transport::QosProfileConf::QOS_PROFILE_DEFAULT);
        start_node_writer =
            start_node->CreateWriter<apollo::cybertron::proto::Chatter>(attr);
        continue;
      }
      std::string node_name;
      std::string in_channel_name1;
      std::string in_channel_name2;
      std::string out_channel_name;
      // end node
      if (i == 1 && j == 1) {
        node_name = "end_node";
        out_channel_name = "";
        // common node
      } else {
        node_name =
            "common_node_" + std::to_string(i) + "_" + std::to_string(j);
        out_channel_name =
            "channel/chatter_" + std::to_string(i) + "_" + std::to_string(j);
      }
      if (j == 0) {
        in_channel_name1 = "channel/chatter_" + std::to_string(i + 1) + "_" +
                           std::to_string(0);
      } else {
        in_channel_name1 =
            "channel/chatter_" + std::to_string(i) + "_" + std::to_string(0);
      }

      if (i != LAYER_NUM) {
        in_channel_name2 = "channel/chatter_" + std::to_string(i + 1) + "_" +
                           std::to_string(j + 1);
      }

      std::shared_ptr<apollo::cybertron::Node> node =
          apollo::cybertron::CreateNode(node_name);
      node_name = node_name + "_component";
      std::shared_ptr<
          apollo::cybertron::Writer<apollo::cybertron::proto::Chatter>>
          writer = nullptr;
      if (out_channel_name != "") {
        apollo::cybertron::proto::RoleAttributes attr;
        attr.set_channel_name(out_channel_name);
        writer = node->CreateWriter<apollo::cybertron::proto::Chatter>(attr);
      }
      LocalWriteInfo writer_info;
      writer_info.out_channel_name = out_channel_name;
      writer_info.writer = writer;
      writer_info.processor_id = idx % 4;
      writer_info_map[node_name] = writer_info;

      if (i == LAYER_NUM) {
        apollo::cybertron::proto::RoleAttributes attr;
        attr.set_channel_name(in_channel_name1);
        attr.set_node_name(node_name);
        auto reader = node->CreateReader<apollo::cybertron::proto::Chatter>(
            attr, [=](const std::shared_ptr<apollo::cybertron::proto::Chatter>&
                          msg1) { LocalProc(node_name, msg1); });
        reader_list.emplace_back(std::move(reader));
      } else {
        std::shared_ptr<apollo::cybertron::ComponentBase> base =
            std::make_shared<GaussianComponent>();
        apollo::cybertron::proto::ComponentConfig config;
        config.set_name(node_name);
        auto reader1 = config.add_readers();
        reader1->set_channel(in_channel_name1);
        auto reader2 = config.add_readers();
        reader2->set_channel(in_channel_name2);
        base->Initialize(config);
        component_base_list.emplace_back(std::move(base));
      }
      ++idx;
    }
  }
  for (int i = 0; i < MSG_NUM; ++i) {
    if (apollo::cybertron::IsShutdown()) {
      break;
    }
    auto msg = std::make_shared<apollo::cybertron::proto::Chatter>();
    auto stamp = apollo::cybertron::Time::Now().ToNanosecond();
    msg->set_timestamp(stamp);
    msg->set_seq(i);
    msg->set_content("Hello, apollo!");
    msg->set_lidar_timestamp(stamp);
    start_node_writer->Write(msg);
    auto end_stamp = apollo::cybertron::Time::Now().ToNanosecond();
    AINFO << "start_node seq[" << msg->seq() << "] write gap: " << (end_stamp - stamp);
    if (i == 0) {
      usleep(MSG_SEND_GAP_US * 10);
    } else {
      usleep(MSG_SEND_GAP_US);
    }
  }
}

int main(int argc, char* argv[]) {
  apollo::cybertron::Init(argv[0]);
  if (argc < 6) {
    AINFO << "Usage: benchmark_gaussian"
          << " <LAYER_NUM>"
          << " <MSG_TIME_GAP>"
          << " <PROC_TIME_US>"
          << " <PROC_TYPE>"
          << " <MSG_NUM>";
    return 1;
  }
  LAYER_NUM = std::stoi(argv[1]);
  MSG_SEND_GAP_US = std::stoi(argv[2]);
  PROC_TIME_US = std::stoi(argv[3]);
  // 0: usleep; 1: cpu computer; 2: gpu computer
  PROC_TYPE = std::stoi(argv[4]);
  MSG_NUM = std::stoi(argv[5]);


  GenGaussianDag();
  while (!apollo::cybertron::IsShutdown()) {
    std::this_thread::sleep_for(std::chrono::milliseconds(200));
  }
  apollo::cybertron::PrintSchedulerStatistics();

  int size = latency_list.size();
  if (size <= 0) {
    AINFO << "benchmark down without e2e finished";
    return 0;
  }
  uint64_t avg = 0;
  uint64_t count = 0;
  double pfcount = 0;
  for (auto& x : latency_list) {
    count += x;
  }
  avg = count / latency_list.size();
  for (auto& x : latency_list) {
    uint64_t cha = abs(x - avg);
    pfcount += pow((double)cha, 2);
  }
  pfcount = pfcount / latency_list.size();

  std::sort(latency_list.begin(), latency_list.end());
  std::sort(chain_latency_list.begin(), chain_latency_list.end());
  int nn_index = size * 0.99;
  int fifth_index = size * 0.5;
  AINFO << "e2e_num: " << latency_list.size()
        << ", min_latency(ns): " << latency_list[0]
        << ", max_latency(ns): " << latency_list[size - 1]
        << ", 50_latency(ns): " << latency_list[fifth_index]
        << ", 99_latency(ns): " << latency_list[nn_index]
        << ", the avg latency(ns): " << avg
        << ", standard deviation:" << sqrt(pfcount);
  if (LAYER_NUM == 1) {
    AINFO << "chain latency -- min_latency(ns): " << chain_latency_list[0]
        << ", max_latency(ns): " << chain_latency_list[size - 1]
        << ", 50_latency(ns): " << chain_latency_list[fifth_index]
        << ", 99_latency(ns): " << chain_latency_list[nn_index];
  }

}
