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

#ifndef CYBER_SCHEDULER_SCHEDULER_FACTORY_H_
#define CYBER_SCHEDULER_SCHEDULER_FACTORY_H_

#include "cyber/scheduler/scheduler_factory.h"

#include <string>

#include "cyber/common/environment.h"
#include "cyber/common/file.h"
#include "cyber/common/global_data.h"
#include "cyber/common/util.h"
#include "cyber/scheduler/policy/scheduler_choreography.h"
#include "cyber/scheduler/policy/scheduler_classic.h"
#include "cyber/scheduler/scheduler.h"

namespace apollo {
namespace cyber {
namespace scheduler {

using apollo::cyber::common::GetAbsolutePath;
using apollo::cyber::common::GetProtoFromFile;
using apollo::cyber::common::GlobalData;
using apollo::cyber::common::PathExists;
using apollo::cyber::common::WorkRoot;

Scheduler* Instance() {
  static Scheduler* instance = nullptr;

  if (unlikely(!instance)) {
    std::string policy = "classic";

    // Get sched policy from conf
    std::string conf("conf/");
    conf.append(GlobalData::Instance()->ProcessGroup()).append(".conf");
    auto cfg_file = GetAbsolutePath(WorkRoot(), conf);

    apollo::cyber::proto::CyberConfig cfg;
    if (PathExists(cfg_file) && GetProtoFromFile(cfg_file, &cfg)) {
      policy = cfg.scheduler_conf().policy();
    } else {
      ADEBUG << "Pls make sure schedconf exist and which format is correct.\n";
    }

    if (!policy.compare("classic")) {
      instance = new SchedulerClassic();
    } else if (!policy.compare("choreography")) {
      instance = new SchedulerChoreography();
    } else {
      instance = new SchedulerClassic();
    }
  }
  return instance;
}

}  // namespace scheduler
}  // namespace cyber
}  // namespace apollo

#endif  // CYBER_SCHEDULER_SCHEDULER_FACTORY_H_
