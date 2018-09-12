#include <chrono>
#include <functional>
#include <iostream>
#include <memory>
#include <mutex>
#include <thread>

#include <unistd.h>
#include <cmath>

#include "benchmark/component/perception_component.h"
#include "benchmark/types.h"
#include "cybertron/croutine/routine_factory.h"
#include "cybertron/scheduler/scheduler.h"
#include "cybertron/common/util.h"
#include "cybertron/common/global_data.h"

using apollo::cybertron::croutine::CRoutine;
using apollo::cybertron::croutine::RoutineFactory;
using apollo::cybertron::croutine::CreateRoutineFactory;
using apollo::cybertron::data::DataDispatcher;
using apollo::cybertron::scheduler::Scheduler;
using apollo::cybertron::Component;
using apollo::cybertron::ComponentBase;
using apollo::cybertron::proto::ComponentConfig;
using apollo::cybertron::common::GlobalData;

int ROUTINE_NUM = 1;
int MESSAGE_NUM = 1;
int YIELD_TIMES = 10000;
int count[10] = {0, 0, 0, 0, 0, 0, 0, 0, 0, 0};

std::vector<std::shared_ptr<ComponentBase>> vec;

void RunDriver() {
  // Mock Drivers
  std::string channel_1 = "driver/channel1";
  std::string channel_2 = "driver/channel2";
  std::string channel_3 = "driver/channel3";
  std::string channel_4 = "driver/channel4";
  uint64_t channel_id_1 = apollo::cybertron::common::Hash(channel_1);
  uint64_t channel_id_2 = apollo::cybertron::common::Hash(channel_2);
  uint64_t channel_id_3 = apollo::cybertron::common::Hash(channel_3);
  uint64_t channel_id_4 = apollo::cybertron::common::Hash(channel_4);
  for (int i = 0; i < MESSAGE_NUM; ++i) {
    auto msg1 = std::make_shared<Driver>();
    auto msg2 = std::make_shared<Driver>();
    auto msg3 = std::make_shared<Driver>();
    auto msg4 = std::make_shared<Driver>();
    msg1->set_msg_id(1);
    msg2->set_msg_id(2);
    msg3->set_msg_id(3);
    msg4->set_msg_id(4);
    {
      DataDispatcher<Driver>::Instance()->Dispatch(channel_id_1, msg1);
      DataDispatcher<Driver>::Instance()->Dispatch(channel_id_2, msg2);
      DataDispatcher<Driver>::Instance()->Dispatch(channel_id_3, msg3);
      DataDispatcher<Driver>::Instance()->Dispatch(channel_id_4, msg4);
    }
  }
}

int main(int argc, char *argv[]) {
  if (argc < 3) {
    std::cout << "Usage: benchmark_switch <routine_num> <switch_num>"
              << std::endl;
    return 1;
  }
  ROUTINE_NUM = std::stoi(argv[1]);
  YIELD_TIMES = std::stoi(argv[2]);
  RunDriver();
  std::this_thread::sleep_for(std::chrono::milliseconds(100));
  {
    for (int i = 0; i < ROUTINE_NUM; ++i) {
      ComponentConfig config;
      config.add_readers()->set_channel("driver/channel" +
                                        std::to_string(i + 1));
      // Ignore memory leak here lol.
      auto pc = std::make_shared<PerceptionComponent>();
      pc->Initialize(config);
      vec.push_back(pc);
    }

    int total = 0;
    while (total < ROUTINE_NUM) {
      total = 0;
      for (int i = 0; i < ROUTINE_NUM; ++i) {
        total += count[i + 1];
      }
      std::this_thread::sleep_for(std::chrono::milliseconds(1));
    }
    std::cout << "********** Timer Statisitcs **********" << std::endl;
  }

  Scheduler::Instance()->PrintStatistics();
  return 0;
}
