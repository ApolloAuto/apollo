/******************************************************************************
 * Copyright 2021 The Apollo Authors. All Rights Reserved.
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

#include <cstdlib>
#include <iostream>
#include <string>
#include <vector>

#include "cyber/cyber.h"
#include "cyber/croutine/croutine.h"
#include "cyber/init.h"
#include "cyber/io/session.h"
#include "cyber/io/timer.h"
#include "cyber/scheduler/scheduler_factory.h"

int main(int argc, char* argv[]) {
  using apollo::cyber::croutine::CRoutine;
  using apollo::cyber::croutine::RoutineState;
  using apollo::cyber::io::Timer;
  using apollo::cyber::scheduler::Scheduler;
  apollo::cyber::Init(argv[0]);
  apollo::cyber::scheduler::Instance()->CreateTask(
    []() {
      while (true) {
        auto t0 = std::chrono::system_clock::now();
        Timer::WaitFor(std::chrono::microseconds(500000));
        auto t1 = std::chrono::system_clock::now();
        auto duration =
          std::chrono::duration_cast<std::chrono::milliseconds>(t1 - t0);
        AINFO << "task1 sleep duration " << duration.count() << "ms";
      }
    },
    "timer_task1");
  apollo::cyber::scheduler::Instance()->CreateTask(
    []() {
      while (true) {
        auto t0 = std::chrono::system_clock::now();
        Timer::WaitFor(std::chrono::microseconds(300000));
        auto t1 = std::chrono::system_clock::now();
        auto duration =
          std::chrono::duration_cast<std::chrono::milliseconds>(t1 - t0);
        AINFO << "task2 sleep duration " << duration.count() << "ms";
      }
    },
    "timer_task2");
  apollo::cyber::WaitForShutdown();
  return 0;
}
