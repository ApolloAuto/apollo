#include <time.h>
#include <unistd.h>

#include <cstdint>
#include <cstring>
#include <iostream>
#include <memory>
#include <sstream>
#include <string>

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
  RunConfig()
      : msg_size(1000),
        msg_num(1000),
        sleep_us(10000),
        mode(OptionalMode::SHM) {}
  int msg_size;
  int msg_num;
  int sleep_us;
  OptionalMode mode;
};

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

  std::string mode(argv[4], strlen(argv[4]));
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
  if (argc < 5) {
    std::cout << "Usage:" << std::endl;
    std::cout << "     argv[0] program name" << std::endl;
    std::cout << "     argv[1] message size(bytes)" << std::endl;
    std::cout << "     argv[2] message transmit number" << std::endl;
    std::cout << "     argv[3] message transmit frequency(Hz, 0 means max)"
              << std::endl;
    std::cout << "     argv[4] transport mode(shm or rtps)" << std::endl;
    std::cout << "     argv[5] writer num(default 1)" << std::endl;
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
  std::cout << " transport mode: " << argv[4] << std::endl;
  int writer_num = 1;
  if (argc == 6) {
    writer_num = atoi(argv[5]);
  }
  std::cout << " writer num: " << writer_num << std::endl;

  RoleAttributes attr;
  attr.set_channel_name("channel");
  auto qos = attr.mutable_qos_profile();
  qos->CopyFrom(QosProfileConf::QOS_PROFILE_DEFAULT);

  std::vector<std::thread> ths;
  for (int i = 0; i < writer_num; i++) {
    ths.push_back(std::thread([i, attr, cfg] {
      // create upper_reach
      std::shared_ptr<apollo::cybertron::transport::UpperReach<RawMessage>>
          upper_reach;
      try {
        upper_reach = Transport::CreateUpperReach<RawMessage>(attr, cfg.mode);
      } catch (...) {
        return -1;
      }

      sleep(2);
      std::cout << "-----Start " << i << " -----" << std::endl;
      // write msg
      auto msg = std::make_shared<RawMessage>();
      msg->message = std::string(cfg.msg_size, '+');
      std::ostringstream name;
      name << std::setfill('0') << std::setw(3) << i;
      msg->message.replace(0, 3, name.str());
      for (int i = 0; i < cfg.msg_num; ++i) {
        Fill(&msg->message);
        upper_reach->Transmit(msg);
        usleep(cfg.sleep_us);

        /*static int last_percent = 0;
        static int this_percent = 0;
        this_percent = i * 100 / cfg.msg_num;
        if (this_percent != last_percent) {
          std::cout << this_percent << "%" << std::endl;
          last_percent = this_percent;
        }*/
      }
    }));
  }

  for (auto& th : ths) {
    th.join();
  }

  // std::cout << "100%" << std::endl;
  // std::cout << "Program will end in 5 seconds." << std::endl;

  sleep(5);

  std::cout << "------End------" << std::endl;
  std::cout << "total transmit msg num: " << cfg.msg_num << std::endl;

  return 0;
}
