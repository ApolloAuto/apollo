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

#include "gflags/gflags.h"
#include "modules/common/log.h"
#include "modules/data/recorder/info_collector.h"

DEFINE_string(docker_image, "", "Current container's docker image.");
DEFINE_string(commit_id, "", "Current commit ID.");

namespace apollo {
namespace data {

void UpdateTaskInfo() {
  Task task = InfoCollector::LoadTaskInfoTemplate();

  // Update software information.
  auto *software = task.mutable_software();
  if (!FLAGS_docker_image.empty()) {
    software->set_docker_image(FLAGS_docker_image);
  }
  if (!FLAGS_commit_id.empty()) {
    software->set_commit_id(FLAGS_commit_id);
  }

  CHECK(InfoCollector::SaveTaskInfoTemplate(task));
}

}  // namespace data
}  // namespace apollo

int main(int argc, char **argv) {
  google::InitGoogleLogging(argv[0]);
  google::ParseCommandLineFlags(&argc, &argv, true);

  apollo::data::UpdateTaskInfo();

  return 0;
}
