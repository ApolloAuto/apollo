#include <time.h>
#include <unistd.h>

#include <cstdint>
#include <iostream>
#include <memory>
#include <string>

#include "cybertron/message/raw_message.h"
#include "cybertron/proto/role_attributes.pb.h"
#include "cybertron/time/time.h"
#include "cybertron/transport/transport.h"

using apollo::cybertron::Time;
using namespace apollo::cybertron::common;
using namespace apollo::cybertron::message;
using namespace apollo::cybertron::transport;
using namespace apollo::cybertron::proto;

struct RunConfig {
  RunConfig()
      : msg_size(1000), msg_num(1000), sleep_us(10000), receiver_num(1) {}
  int msg_size;
  int msg_num;
  int sleep_us;
  int receiver_num;
};

struct RunStatistic {
  RunStatistic()
      : recv_num(0),
        min_latency(1000000000L),
        max_latency(0),
        total_latency(0) {}
  uint64_t recv_num;
  uint64_t min_latency;  // nano second
  uint64_t max_latency;
  uint64_t total_latency;
};

const int MAX_RECEIVER_NUM = 10;
std::shared_ptr<Receiver<RawMessage>> receiver[MAX_RECEIVER_NUM];
RunStatistic statistic[MAX_RECEIVER_NUM];

bool Fill(std::string* dst) {
  if (dst == nullptr) {
    return false;
  }
  auto timestamp_str = std::to_string(Time::Now().ToNanosecond());
  auto pos = dst->rfind('+');
  if (pos == std::string::npos) {
    return false;
  }
  dst->replace(dst->begin() + pos + 1, dst->end(), timestamp_str);
  return true;
}

bool Parse(const std::string& src, uint64_t* timestamp) {
  if (timestamp == nullptr) {
    return false;
  }
  auto pos = src.rfind('+');
  if (pos == std::string::npos) {
    return false;
  }
  auto timestamp_str = src.substr(pos + 1);
  *timestamp = std::stoll(timestamp_str);
  return true;
}

bool Init(char* argv[], RunConfig* cfg) {
  int tmp = 0;
  tmp = atoi(argv[1]);
  if (tmp <= 0) {
    return false;
  } else {
    cfg->msg_size = tmp;
  }
  tmp = atoi(argv[2]);
  if (tmp <= 0) {
    return false;
  } else {
    cfg->msg_num = tmp;
  }

  tmp = atoi(argv[3]);
  if (tmp < 0) {
    return false;
  } else if (tmp == 0) {
    cfg->sleep_us = 0;
  } else {
    cfg->sleep_us = 1000000 / tmp;
  }

  tmp = atoi(argv[4]);
  if (tmp < 1 || tmp > MAX_RECEIVER_NUM) {
    return false;
  } else {
    cfg->receiver_num = tmp;
  }

  return true;
}

int main(int argc, char* argv[]) {
  if (argc != 5) {
    std::cout << "Usage:" << std::endl;
    std::cout << "     argv[0] program name" << std::endl;
    std::cout << "     argv[1] message size(bytes)" << std::endl;
    std::cout << "     argv[2] message transmit number" << std::endl;
    std::cout << "     argv[3] message transmit frequency(Hz, 0 means max)"
              << std::endl;
    std::cout << "     argv[4] receiver number(1~" << MAX_RECEIVER_NUM << ")"
              << std::endl;
    return 0;
  }

  RunConfig cfg;
  if (!Init(argv, &cfg)) {
    std::cout << "Please check content of arguments." << std::endl;
    return 0;
  }

  std::cout << "***** Run Config *****" << std::endl;
  std::cout << "   message size: " << cfg.msg_size << std::endl;
  std::cout << "   transmit num: " << cfg.msg_num << std::endl;
  std::cout << "  interval (us): " << cfg.sleep_us << std::endl;
  std::cout << "receiver num: " << cfg.receiver_num << std::endl;

  RoleAttributes attr;
  attr.set_channel_name("channel");

  // create transmitter
  std::shared_ptr<apollo::cybertron::transport::Transmitter<RawMessage>>
      transmitter;
  try {
    transmitter =
        Transport::CreateTransmitter<RawMessage>(attr, OptionalMode::INTRA);
  } catch (...) {
    return -1;
  }

  // create receiver
  for (int i = 0; i < cfg.receiver_num; ++i) {
    try {
      receiver[i] = Transport::CreateReceiver<RawMessage>(
          attr,
          [i](const std::shared_ptr<RawMessage>& msg,
              const MessageInfo& msg_info, const RoleAttributes& attr) {
            uint64_t recv = Time::Now().ToNanosecond();
            uint64_t send = 0;
            Parse(msg->message, &send);
            uint64_t diff = recv - send;
            ++statistic[i].recv_num;
            statistic[i].total_latency += diff;
            if (diff > statistic[i].max_latency) {
              statistic[i].max_latency = diff;
            }
            if (diff < statistic[i].min_latency) {
              statistic[i].min_latency = diff;
            }
          },
          OptionalMode::INTRA);
    } catch (...) {
    }
  }

  auto msg = std::make_shared<RawMessage>();
  msg->message = std::string(cfg.msg_size, '+');
  // transmit msg
  for (int i = 0; i < cfg.msg_num; ++i) {
    Fill(&msg->message);
    transmitter->Transmit(msg);
    usleep(cfg.sleep_us);

    static int last_percent = 0;
    static int this_percent = 0;
    this_percent = i * 100 / cfg.msg_num;
    if (this_percent != last_percent) {
      std::cout << this_percent << "%" << std::endl;
      last_percent = this_percent;
    }
  }

  std::cout << "100%" << std::endl;

  sleep(2);

  RunStatistic total;

  // show result
  for (int i = 0; i < cfg.receiver_num; ++i) {
    std::cout << "receiver[" << i << "]" << std::endl;
    std::cout << "  recv num: " << statistic[i].recv_num << std::endl;
    std::cout << "  min latency(ns): " << statistic[i].min_latency << std::endl;
    std::cout << "  max latency(ns): " << statistic[i].max_latency << std::endl;
    if (statistic[i].recv_num != 0) {
      std::cout << "  avg latency(ns): "
                << statistic[i].total_latency / statistic[i].recv_num
                << std::endl;
    }

    total.recv_num += statistic[i].recv_num;
    if (statistic[i].min_latency < total.min_latency) {
      total.min_latency = statistic[i].min_latency;
    }
    if (statistic[i].max_latency > total.max_latency) {
      total.max_latency = statistic[i].max_latency;
    }
    total.total_latency += statistic[i].total_latency;
  }

  std::cout << "-----------------------------" << std::endl;
  std::cout << "total transmit msg num: " << cfg.msg_num << std::endl;
  std::cout << "total min latency(ns): " << total.min_latency << std::endl;
  std::cout << "total max latency(ns): " << total.max_latency << std::endl;
  if (total.recv_num != 0) {
    std::cout << "total avg latency(ns): "
              << total.total_latency / total.recv_num << std::endl;
  }

  return 0;
}
