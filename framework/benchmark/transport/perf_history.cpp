/******************************************************************************
 * Copyright 2017 The Apollo Authors. All Rights Reserved.
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

#include <cstdint>
#include <cstdlib>
#include <vector>

#include "cybertron/common/log.h"
#include "cybertron/message/raw_message.h"
#include "cybertron/time/time.h"
#include "cybertron/transport/message/history.h"

using apollo::cybertron::Time;
using apollo::cybertron::message::RawMessage;
using apollo::cybertron::proto::QosHistoryPolicy;
using apollo::cybertron::transport::HistoryAttributes;
using apollo::cybertron::transport::MessageInfo;
using RawMsgHistory = apollo::cybertron::transport::History<RawMessage>;

struct RunStat {
  RunStat()
      : min_add_cost(123456789),
        max_add_cost(0),
        avg_add_cost(0),
        total_add_cost(0),
        get_cached_cost(0) {}

  void Display() {
    AINFO << "***** History Run Statistics *****";
    AINFO << "   min_add_cost(ns): " << min_add_cost;
    AINFO << "   max_add_cost(ns): " << max_add_cost;
    AINFO << "   avg_add_cost(ns): " << avg_add_cost;
    AINFO << "get_cached_cost(ns): " << get_cached_cost;
  }

  // unit: ns
  uint64_t min_add_cost;
  uint64_t max_add_cost;
  uint64_t avg_add_cost;
  uint64_t total_add_cost;
  uint64_t get_cached_cost;
};

void Usage() {
  AINFO << "Usage:";
  AINFO << "    argv[0] program name";
  AINFO << "    argv[1] message number(default:10000)";
}

void Run(int msg_num) {
  HistoryAttributes attr(QosHistoryPolicy::HISTORY_KEEP_ALL, 1000);
  RawMsgHistory history(attr);
  history.Enable();

  RunStat stat;
  MessageInfo info;
  auto msg = std::make_shared<RawMessage>(std::string(1000, 'h'));

  uint64_t start = 0;
  uint64_t end = 0;

  for (int i = 0; i < msg_num; ++i) {
    start = Time::Now().ToNanosecond();
    history.Add(msg, info);
    end = Time::Now().ToNanosecond();
    uint64_t cost = end - start;
    if (cost < stat.min_add_cost) {
      stat.min_add_cost = cost;
    }
    if (cost > stat.max_add_cost) {
      stat.max_add_cost = cost;
    }
    stat.total_add_cost += cost;
  }

  std::vector<RawMsgHistory::CachedMessage> msgs;
  start = Time::Now().ToNanosecond();
  history.GetCachedMessage(&msgs);
  end = Time::Now().ToNanosecond();
  stat.get_cached_cost = end - start;
  stat.avg_add_cost = stat.total_add_cost / msg_num;
  stat.Display();
}

int main(int argc, char* argv[]) {
  if (argc > 2) {
    Usage();
    return -1;
  }

  int msg_num = 10000;
  if (argc == 2) {
    int tmp = 0;
    tmp = atoi(argv[1]);
    if (tmp <= 0) {
      AINFO << "Please enter valid message number(a integer larger than zero).";
      return -1;
    }
    msg_num = tmp;
  }

  AINFO << "total msg num: " << msg_num;
  Run(msg_num);
  return 0;
}