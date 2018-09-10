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

#ifndef EXAMPLES_COMPONENT_WATCHDOG_WATCHDOG_H_
#define EXAMPLES_COMPONENT_WATCHDOG_WATCHDOG_H_

#include <cstdint>
#include <mutex>
#include <string>

enum SurvivalCondition { DEAD = 0, ALIVE };

struct DogConfig {
  DogConfig()
      : food("pork"), min_feed_interval(0), max_feed_interval(1000000) {}

  std::string food;
  uint64_t min_feed_interval;  // unit: us
  uint64_t max_feed_interval;  // unit: us
};

struct DogStat {
  DogStat()
      : activated(false),
        condition(SurvivalCondition::ALIVE),
        last_eat_timestamp(0) {}
  bool activated;
  SurvivalCondition condition;
  uint64_t last_eat_timestamp;  // unit: us
};

class Watchdog {
 public:
  Watchdog();
  Watchdog(const DogConfig& cfg);
  virtual ~Watchdog();

  SurvivalCondition Feed(const std::string& food);

 private:
  DogConfig cfg_;
  DogStat stat_;
  std::mutex mutex_;
};

#endif  // EXAMPLES_COMPONENT_WATCHDOG_WATCHDOG_H_
