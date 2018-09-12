#include <signal.h>
#include <time.h>
#include <unistd.h>

#include <cstdint>
#include <iostream>
#include <memory>
#include <string>

#include "cybertron/common/global_data.h"
#include "cybertron/message/raw_message.h"
#include "cybertron/proto/role_attributes.pb.h"
#include "cybertron/time/time.h"
#include "cybertron/transport/qos/qos_profile_conf.h"
#include "cybertron/transport/transport.h"

using apollo::cybertron::Time;
using namespace apollo::cybertron::common;
using namespace apollo::cybertron::message;
using namespace apollo::cybertron::transport;
using namespace apollo::cybertron::proto;

struct RunConfig {
  RunConfig() : lower_reach_num(1), mode(OptionalMode::SHM) {}
  int lower_reach_num;
  OptionalMode mode;
};

struct RunStatistic {
  RunStatistic()
      : recv_num(0),
        min_latency(1000000000L),
        max_latency(0),
        total_latency(0),
        start_time(0),
        end_time(0) {}
  uint64_t recv_num;
  uint64_t min_latency;    // nano second
  uint64_t max_latency;    // nano second
  uint64_t total_latency;  // nano second
  uint64_t start_time;
  uint64_t end_time;
};

RunConfig cfg;
const int MAX_LOWER_REACH_NUM = 30;
std::shared_ptr<LowerReach<RawMessage>> lower_reach[MAX_LOWER_REACH_NUM];
std::map<std::string, RunStatistic> statistic[MAX_LOWER_REACH_NUM];
volatile bool shut_down = false;

bool Parse(const std::string& src, std::string& writer_name,
           uint64_t* timestamp) {
  if (timestamp == nullptr) {
    return false;
  }
  auto pos = src.rfind('+');
  if (pos == std::string::npos) {
    return false;
  }
  auto timestamp_str = src.substr(pos + 1);
  *timestamp = std::stoll(timestamp_str);
  writer_name = src.substr(0, 3);
  return true;
}

void SignalHandler(int signal) {
  RunStatistic total;

  // show result
  for (int i = 0; i < cfg.lower_reach_num; ++i) {
    RunStatistic each_total;
    std::cout << "lower_reach[" << i << "]" << std::endl;
    for (auto each_lower : statistic[i]) {
      std::string writer_name = each_lower.first;
      RunStatistic& each_statistic = each_lower.second;
      std::cout << "  writer name: " << writer_name << std::endl;
      std::cout << "    recv num: " << each_statistic.recv_num << std::endl;
      std::cout << "    min latency(ns): " << each_statistic.min_latency
                << std::endl;
      std::cout << "    max latency(ns): " << each_statistic.max_latency
                << std::endl;
      if (each_statistic.recv_num != 0) {
        std::cout << "    avg latency(ns): "
                  << each_statistic.total_latency / each_statistic.recv_num
                  << std::endl;
      }
      std::cout << "    total used(ns): "
                << each_statistic.end_time - each_statistic.start_time
                << std::endl;

      each_total.recv_num += each_statistic.recv_num;
      if (each_statistic.min_latency < each_total.min_latency) {
        each_total.min_latency = each_statistic.min_latency;
      }
      if (each_statistic.max_latency > each_total.max_latency) {
        each_total.max_latency = each_statistic.max_latency;
      }
      each_total.total_latency += each_statistic.total_latency;
    }
    std::cout << "receiver total min latency(ns): " << each_total.min_latency
              << std::endl;
    std::cout << "receiver total max latency(ns): " << each_total.max_latency
              << std::endl;
    if (each_total.recv_num != 0) {
      std::cout << "receiver total avg latency(ns): "
                << each_total.total_latency / each_total.recv_num << std::endl;
    }
    if (each_total.min_latency < total.min_latency) {
      total.min_latency = each_total.min_latency;
    }
    if (each_total.max_latency > total.max_latency) {
      total.max_latency = each_total.max_latency;
    }
    total.recv_num += each_total.recv_num;
    total.total_latency += each_total.total_latency;
    std::cout << "=================================\n" << std::endl;
  }

  std::cout << "\n\n---------------------------------" << std::endl;
  std::cout << "total min latency(ns): " << total.min_latency << std::endl;
  std::cout << "total max latency(ns): " << total.max_latency << std::endl;
  if (total.recv_num != 0) {
    std::cout << "total avg latency(ns): "
              << total.total_latency / total.recv_num << std::endl;
  }

  shut_down = true;
}

bool Init(char* argv[], RunConfig* cfg) {
  int tmp = 0;
  tmp = atoi(argv[1]);
  if (tmp < 1 || tmp > MAX_LOWER_REACH_NUM) {
    return false;
  } else {
    cfg->lower_reach_num = tmp;
  }

  std::string mode(argv[2], strlen(argv[2]));
  if (mode == "shm") {
    cfg->mode = OptionalMode::SHM;
  } else if (mode == "rtps") {
    cfg->mode = OptionalMode::RTPS;
  } else {
    return false;
  }

  return true;
}

int main(int argc, char* argv[]) {
  if (argc != 3) {
    std::cout << "Usage:" << std::endl;
    std::cout << "     argv[0] program name" << std::endl;
    std::cout << "     argv[1] lower_reach number(1~" << MAX_LOWER_REACH_NUM
              << ")" << std::endl;
    std::cout << "     argv[2] transport mode(shm or rtps)" << std::endl;
    return 0;
  }

  if (!Init(argv, &cfg)) {
    std::cout << "Please check content of arguments." << std::endl;
    return 0;
  }

  std::cout << "***** Run Config *****" << std::endl;
  std::cout << "lower_reach num: " << cfg.lower_reach_num << std::endl;
  std::cout << " transport mode: " << argv[2] << std::endl;

  signal(SIGINT, SignalHandler);

  RoleAttributes attr;
  attr.set_host_name(GlobalData::Instance()->HostName());
  attr.set_process_id(GlobalData::Instance()->ProcessId());
  attr.set_channel_name("channel");
  auto channel_id = GlobalData::Instance()->RegisterChannel("channel");
  attr.set_channel_id(channel_id);
  auto qos = attr.mutable_qos_profile();
  qos->CopyFrom(QosProfileConf::QOS_PROFILE_DEFAULT);

  // create lower_reach
  for (int i = 0; i < cfg.lower_reach_num; ++i) {
    try {
      lower_reach[i] = Transport::CreateLowerReach<RawMessage>(
          attr,
          [i](const std::shared_ptr<RawMessage>& msg,
              const MessageInfo& msg_info, const RoleAttributes& attr) {
            uint64_t recv = Time::Now().ToNanosecond();
            uint64_t send = 0;
            std::string writer_name;
            Parse(msg->message, writer_name, &send);
            uint64_t diff = recv - send;
            if (statistic[i].count(writer_name) == 0) {
              statistic[i][writer_name] = RunStatistic();
            }
            RunStatistic& each = statistic[i][writer_name];
            ++each.recv_num;
            each.total_latency += diff;
            if (diff < each.min_latency) {
              each.min_latency = diff;
            }
            if (diff > each.max_latency) {
              each.max_latency = diff;
            }
            if (each.start_time == 0) {
              each.start_time = recv;
            }
            each.end_time = recv;
          },
          cfg.mode);
    } catch (...) {
      return -1;
    }
  }

  std::cout << "-----Start-----" << std::endl;
  std::cout << "If you want to stop this program, please enter CTRL+C"
            << std::endl;

  while (!shut_down) {
    sleep(1);
  }

  std::cout << "------End------" << std::endl;
  if (cfg.mode == OptionalMode::SHM) {
    ShmDispatcher::Instance()->Shutdown();
  }

  return 0;
}
