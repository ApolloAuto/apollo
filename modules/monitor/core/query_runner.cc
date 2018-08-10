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

#include <chrono>
#include <thread>

#include "query_runner.h"

namespace apollo {
namespace monitor {
namespace sysmon {

void PeriodicQueryRunner::do_task()
{
  DBG_VAR(int, i=0);

  for(auto& t : _tasks) {
    // @todo: with timestamp
    DBG_ONLY(
        ADEBUG << "to run task " << i++;
    );

    t();
  }
}

void UntilSuccessQueryRunner::do_task()
{
  DBG_VAR(int, i=0);

  auto it = _tasks.begin();
  while (it != _tasks.end()) {
    // @todo: with timestamp
    DBG_ONLY(
        ADEBUG << "to run task " << i++;
    )

    if ((*it)() >= 0) {
      std::list<QueryTask>::iterator to_del = it;
      ++it;
      _tasks.erase(to_del);

      DBG_ONLY(
        ADEBUG << "task done, #tasks: " << _tasks.size();
      );
    } else {
      ++it;
    }
  }

  if (_tasks.empty()) {
    DBG_ONLY(
        ADEBUG << "no more tasks, stop";
    )

    this->stop();
  }
}

void QueryOnceThread::start()
{
  _thrd.reset(new std::thread(std::bind(&QueryOnceThread::do_tasks, this)));
}

void QueryOnceThread::do_tasks()
{
  DBG_VAR(int, i=0);

  for(auto& t : _tasks) {
    // @todo: with timestamp
    DBG_ONLY(
        ADEBUG << "to run task " << i++;
    )

    t();
  }
}

}  // namespace sysmon
}  // namespace monitor
}  // namespace apollo
