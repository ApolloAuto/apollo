/******************************************************************************
 * Copyright 2019 The Apollo Authors. All Rights Reserved.
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

/**
 * @file stage_test_base.cc
 **/

#include "modules/planning/scenarios/test/stage_test_base.h"

#include "cyber/common/file.h"
#include "cyber/common/log.h"
#include "modules/planning/common/planning_gflags.h"
#include "modules/planning/tasks/task_factory.h"

namespace apollo {
namespace planning {

using apollo::cyber::common::GetProtoFromFile;

void StageTestBase::SetUp() {
  CHECK(GetProtoFromFile(FLAGS_planning_config_file, &planning_config_))
      << "failed to load planning config file " << FLAGS_planning_config_file;
  TaskFactory::Init(planning_config_);
  CHECK(GetProtoFromFile(scenario_config_file_, &scenario_config_))
      << "failed to load config file " << scenario_config_file_;
  for (const auto& stage_config : scenario_config_.stage_config()) {
    stage_config_map_[stage_config.stage_type()] = &stage_config;
  }
}

}  // namespace planning
}  // namespace apollo
