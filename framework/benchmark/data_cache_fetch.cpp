#include <unistd.h>
#include <algorithm>
#include <chrono>
#include <cmath>
#include <functional>
#include <iostream>
#include <memory>
#include <queue>
#include <thread>

#include "cybertron/common/global_data.h"
#include "cybertron/common/util.h"
#include "cybertron/croutine/routine_factory.h"
#include "cybertron/data/data_visitor.h"
#include "cybertron/scheduler/scheduler.h"

using apollo::cybertron::common::GlobalData;
using apollo::cybertron::croutine::CreateRoutineFactory;
using apollo::cybertron::croutine::CRoutine;
using apollo::cybertron::croutine::RoutineFactory;
using apollo::cybertron::data::DataCache;
using apollo::cybertron::data::DataVisitor;
using apollo::cybertron::Time;
using Msg = std::shared_ptr<int>;

std::string channel_per = "perception/channel_name";
std::string channel_dri = "channel_name";

std::string channel_1 = "channel1";
std::string channel_2 = "channel2";
std::string channel_3 = "channel3";
std::string channel_4 = "channel4";
uint64_t channel_id_1 = apollo::cybertron::common::Hash(channel_1);
uint64_t channel_id_2 = apollo::cybertron::common::Hash(channel_2);
uint64_t channel_id_3 = apollo::cybertron::common::Hash(channel_3);
uint64_t channel_id_4 = apollo::cybertron::common::Hash(channel_4);
auto data_cache = DataCache::Instance();
/*
 * end mock
 */

void InitDataCache() {
  data_cache->InitChannelCache(channel_id_1);
  data_cache->InitChannelCache(channel_id_2);
  data_cache->InitChannelCache(channel_id_3);
  data_cache->InitChannelCache(channel_id_4);

  for (int i = 0; i < DataCache::FUSION_QUEUE_SIZE; ++i) {
    auto msg1 = std::make_shared<int>(channel_id_1);
    auto msg2 = std::make_shared<int>(channel_id_2);
    auto msg3 = std::make_shared<int>(channel_id_3);
    auto msg4 = std::make_shared<int>(channel_id_4);

    data_cache->WriteDataCache(channel_id_1, msg1);
    data_cache->WriteDataCache(channel_id_2, msg2);
    data_cache->WriteDataCache(channel_id_3, msg3);
    data_cache->WriteDataCache(channel_id_4, msg4);
  }
}

int main(int argc, char* argv[]) {
  if (argc < 2) {
    std::cout << "Usage: data_cache_fetch <times>" << std::endl;
    return 0;
  }

  auto times = std::stoi(argv[1]);

  InitDataCache();
  std::vector<uint64_t> channel_ids = {channel_id_1, channel_id_2, channel_id_3,
                                       channel_id_4};
  auto data_visitor = std::make_shared<DataVisitor>(std::move(channel_ids), 1);
  std::shared_ptr<int> msg;
  auto start = Time::MonoTime().ToNanosecond();
  auto tmp = start;
  std::vector<uint64_t> costs;
  for (int i = 0; i < times; i++) {
    data_visitor->Flag().seq_id = 0;
    data_visitor->TryFetch<int>(msg);
    auto now = Time::MonoTime().ToNanosecond();
    costs.emplace_back(now - tmp);
    tmp = now;
  }

  std::sort(costs.begin(), costs.end());
  uint64_t sum = 0;
  for (auto cost : costs) {
    sum += cost;
  }
  auto avg = sum / costs.size();

  std::cout << "max time: " << costs.back() << std::endl;
  std::cout << "min time: " << costs.front() << std::endl;
  std::cout << "avg time: " << avg << std::endl;

  return 0;
}
